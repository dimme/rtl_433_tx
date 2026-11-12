/** @file
    FSK 8 byte Manchester encoded TPMS with simple checksum.

    Copyright (C) 2017 Christian W. Zuckschwerdt <zany@triq.net>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/
/**
FSK 8 byte Manchester encoded TPMS with simple checksum.
Seen on Ford Fiesta, Focus, Kuga, Escape, Transit...

Seen on 315.00 MHz (United States).

Seen on 433.92 MHz.
Likely VDO-Sensors, Type "S180084730Z", built by "Continental Automotive GmbH".

Typically a transmission is sent 4 times.  Sometimes the T/P values
differ (slightly) among those.

Sensor has 3 modes:
  moving: while being driven
  atrest: once after stopping, and every 6h thereafter (for months)
  learn: 12 transmissions, caused by using learn tool

Packet nibbles:

    II II II II PP TT FF CC

- I = ID
- P = Pressure, as PSI * 4
- T = Temperature, as C + 56, except:
      When 0x80 is on, value is not temperature, meaning the full 8
      bits is not temperature, and the lower 7 bits is also not
      temperature.  Pattern of low 7 bits in this case seems more like
      codepoints than a measurement.
- F = Flags:
      0x80 not seen
      0x40 ON for vehicle moving
        Is strongly correlated with 0x80 being set in TT
      0x20: 9th bit of pressure.  Seen on Transit very high pressure, otherwise not.
      0x10: not seen

      0x08: ON for learn
      0x04: ON for moving (0x08 and 0x04 both OFF for at rest)
      0x02: ~always NOT 0x01 (meaning of 0x3 not understood, but MOVING
            tends to have 0x02)
      0x01: about 19% of samples
- C = Checksum, SUM bytes 0 to 6 = byte 7
*/

#include "decoder.h"
#include "capture.h"
#include "r_encoder.h"
#include <stdlib.h>
#include <math.h>

static int tpms_ford_decode(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos)
{
    bitbuffer_t packet_bits = {0};
    bitbuffer_manchester_decode(bitbuffer, row, bitpos, &packet_bits, 160);

    // require 64 data bits
    if (packet_bits.bits_per_row[0] < 64) {
        return 0;
    }
    uint8_t *b = packet_bits.bb[0];

    if (((b[0] + b[1] + b[2] + b[3] + b[4] + b[5] + b[6]) & 0xff) != b[7]) {
        return 0;
    }

    unsigned id = (unsigned)b[0] << 24 | b[1] << 16 | b[2] << 8 | b[3];

    // Extract and log code to aid in debugging.
    int code = b[4] << 16 | b[5] << 8 | b[6];

    // Formula is a combination of regression and plausible, observed
    // from roughly 31 to 36 psi.  (The bit at byte6-0x20 is shifted
    // to 0x100.)
    int psibits        = (((b[6] & 0x20) << 3) | b[4]);
    float pressure_psi = psibits * 0.25f;

    // Working theory is that temperature bits are temp + 56,
    // encoding -56 to 71 C.  Validated as close around 15 C.
    int temperature_valid = 0;
    int temperature_c     = -1000.0;

    if ((b[5] & 0x80) == 0) {
        temperature_valid = 1;
        temperature_c     = (b[5] & 0x7f) - 56;
    }

    // Set up syndrome of unexpected bits.  The point is to have a
    // variable unknown which is zero if this packet matches the
    // code's understanding, and to be non-zero if anything is unusual,
    // to aid finding logged packets for manual study.

    // Examine moving, learn and normal bits.
    int learn  = 0;
    int moving = 0;
    int unknown = 0;
    switch (b[6] & 0x4c) {
    case 0x8:
        // In response to learn tool
        learn = 1;
        break;

    case 0x4:
        // At rest.
        break;

    case 0x44:
        // Moving.
        moving = 1;
        break;

    default:
        // These three bits taken together do not match a known
        // pattern.  Therefore set all of them as the unknown
        // syndrome.
        unknown = (b[6] & 0x4c);
        break;
    }

    // We've accounted for
    // 0x40(moving) 0x20(temp) 0x8(learn) 04(normal)
    // 0x3(separate, next)
    // so that leaves 0x80 and 0x10, which are expected to be 0.
    unknown |= (b[6] & 0x90);

    // All real captures (tests/Ford_TPMS) have unknown == 0; this syndrome
    // was previously only logged for manual study, never enforced, letting
    // any state/flag combination through the 8-bit checksum unfiltered.
    // Reject the combinations the decoder itself doesn't recognize.
    if (unknown != 0) {
        decoder_logf(decoder, 2, __func__, "unexpected flag bits: %02x", unknown);
        return 0;
    }

    /* Low-order 2 bits are variously 01, 10. */
    int unknown_3 = b[6] & 0x3;

    char id_str[9];
    snprintf(id_str, sizeof(id_str), "%08x", id);
    char code_str[7];
    snprintf(code_str, sizeof(code_str), "%06x", code);
    char unknown_str[3];
    snprintf(unknown_str, sizeof(unknown_str), "%02x", unknown);
    char unknown_3_str[2];
    snprintf(unknown_3_str, sizeof(unknown_3_str), "%01x", unknown_3);

    /* clang-format off */
    data_t *data = data_make(
            "model",            "",             DATA_STRING, "Ford",
            "type",             "",             DATA_STRING, "TPMS",
            "id",               "",             DATA_STRING, id_str,
            "pressure_PSI",     "Pressure",     DATA_FORMAT, "%.2f PSI", DATA_DOUBLE, pressure_psi,
            "temperature_C",    "Temperature",  DATA_COND, temperature_valid, DATA_FORMAT, "%.1f C",   DATA_DOUBLE, (float)temperature_c,
            "moving",           "Moving",       DATA_INT,    moving,
            "learn",            "Learn",        DATA_INT,    learn,
            "code",             "",             DATA_STRING, code_str,
            "unknown",          "",             DATA_STRING, unknown_str,
            "unknown_3",        "",             DATA_STRING, unknown_3_str,
            "mic",              "Integrity",    DATA_STRING, "CHECKSUM",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

/** Encoder: inverse of tpms_ford_decode().

    Starts from the captured 8 message bytes, patches the edited primary fields
    (id, pressure, temperature, moving, learn), recomputes the derived display
    fields (code, unknown, unknown_3) and the byte-sum checksum, then re-encodes
    the Manchester payload in place keeping the captured preamble.
*/
static int tpms_ford_encode(r_device const *decoder, struct capture *cap, bitbuffer_t *out_bits)
{
    if (cap->bits.num_rows < 1)
        return -1;

    uint8_t const preamble_pattern[2] = {0xaa, 0xa9}; // inverted 55 56

    bitbuffer_t src = cap->bits;
    bitbuffer_t inv = src; // inverted copy to locate the preamble
    bitbuffer_invert(&inv);
    unsigned bitpos = bitbuffer_search(&inv, 0, 0, preamble_pattern, 16);
    if (bitpos + 16 + 128 > src.bits_per_row[0])
        return -1;
    unsigned pstart = bitpos + 16;

    // decode the original payload to obtain the 8 message bytes as a base
    bitbuffer_t packet = {0};
    bitbuffer_manchester_decode(&inv, 0, pstart, &packet, 160);
    if (packet.bits_per_row[0] < 64)
        return -1;
    uint8_t b[8];
    memcpy(b, packet.bb[0], 8);

    char const *f;
    if ((f = capture_get_field(cap, "id"))) {
        unsigned id = (unsigned)strtoul(f, NULL, 16);
        b[0] = (uint8_t)(id >> 24);
        b[1] = (uint8_t)(id >> 16);
        b[2] = (uint8_t)(id >> 8);
        b[3] = (uint8_t)(id);
    }
    if ((f = capture_get_field(cap, "pressure_PSI"))) {
        int psibits = (int)lround(atof(f) / 0.25);
        if (psibits < 0) psibits = 0;
        if (psibits > 0x1ff) psibits = 0x1ff;
        b[4] = (uint8_t)(psibits & 0xff);
        if (psibits & 0x100)
            b[6] |= 0x20;
        else
            b[6] &= (uint8_t)~0x20;
    }
    if ((f = capture_get_field(cap, "temperature_C"))) {
        int t = (int)lround(atof(f)) + 56;
        if (t < 0) t = 0;
        if (t > 0x7f) t = 0x7f;
        b[5] = (uint8_t)(t & 0x7f); // clears 0x80 -> temperature valid
    }
    int moving = 0, learn = 0;
    encoder_field_int(cap, "moving", &moving);
    encoder_field_int(cap, "learn", &learn);
    b[6] &= (uint8_t)~0x4c;
    if (learn)
        b[6] |= 0x08;
    else if (moving)
        b[6] |= 0x44;
    else
        b[6] |= 0x04;
    b[6] &= (uint8_t)~0x90; // unknown bits must be 0

    // recompute byte-sum checksum
    b[7] = (uint8_t)((b[0] + b[1] + b[2] + b[3] + b[4] + b[5] + b[6]) & 0xff);

    // re-derive display/redundant fields so verification matches
    char tmp[16];
    snprintf(tmp, sizeof(tmp), "%06x", (b[4] << 16) | (b[5] << 8) | b[6]);
    capture_set_field(cap, "code", tmp);
    capture_set_field(cap, "unknown", "00");
    snprintf(tmp, sizeof(tmp), "%01x", b[6] & 0x3);
    capture_set_field(cap, "unknown_3", tmp);

    // re-encode 64 data bits as Manchester; decode inverts the whole buffer
    // first, so emit (d, 1-d) half-bits (invert=1)
    encoder_manchester_encode(src.bb[0], pstart, b, 64, 1);

    *out_bits = src;

    if (!encoder_verify(decoder, out_bits, cap))
        return -1;
    return 1;
}

/** @sa tpms_ford_decode() */
static int tpms_ford_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    // full preamble is 55 55 55 56 (inverted: aa aa aa a9)
    uint8_t const preamble_pattern[2] = {0xaa, 0xa9}; // 16 bits

    int row;
    unsigned bitpos;
    int ret    = 0;
    int events = 0;

    bitbuffer_invert(bitbuffer);

    for (row = 0; row < bitbuffer->num_rows; ++row) {
        bitpos = 0;
        // Find a preamble with enough bits after it that it could be a complete packet
        while ((bitpos = bitbuffer_search(bitbuffer, row, bitpos,
                preamble_pattern, 16)) + 144 <=
                bitbuffer->bits_per_row[row]) {
            ret = tpms_ford_decode(decoder, bitbuffer, row, bitpos + 16);
            if (ret > 0)
                events += ret;
            bitpos += 15;
        }
    }

    return events > 0 ? events : ret;
}

static char const *const output_fields[] = {
        "model",
        "type",
        "id",
        "flags",
        "pressure_PSI",
        "temperature_C",
        "moving",
        "learn",
        "code",
        "unknown",
        "unknown_3",
        "mic",
        NULL,
};

r_device const tpms_ford = {
        .name        = "Ford TPMS",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 52,  // 12-13 samples @250k
        .long_width  = 52,  // FSK
        .reset_limit = 150, // Maximum gap size before End Of Message [us].
        .decode_fn   = &tpms_ford_callback,
        .encode_fn   = &tpms_ford_encode,
        .fields      = output_fields,
};
