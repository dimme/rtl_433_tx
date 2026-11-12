/**  @file
    Porsche Boxster/Cayman TPMS.

    Copyright (C) 2021 Christian W. Zuckschwerdt <zany@triq.net>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

/**
Porsche Boxster/Cayman TPMS.
Seen on Porsche second generation (Typ 987) Boxster/Cayman.

Full preamble is {30}ccccccca (33333332).
The data is Differential Manchester Coded (DMC).

Example data:

    {193}333333354ab32d334b2d4ab2cab54cb34cb2aab4cd552ccd8
    {193}333333354ab32d334b2d4ab2caaab34cb34d554aacd2b2cd8

    23d7 ad23 623b bb02 f05f
    23d7 ad23 603b bb02 1d37

Data layout (nibbles):

    II II II II PP TT SS SS CC

- I: 32 bit ID
- P: 8 bit Pressure (scale 2.5 offset 100, minimum seen 41 = 0 kPa)
- T: 8 bit Temperature (deg. C offset by 40)
- S: Status?
- C: 16 bit Checksum, CRC-16 poly 0x1021 init 0xffff
*/

#include "decoder.h"
#include "r_encoder.h"
#include <stdlib.h>
#include <math.h>

static int tpms_porsche_decode(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos)
{
    bitbuffer_t packet_bits = {0};
    bitbuffer_differential_manchester_decode(bitbuffer, row, bitpos, &packet_bits, 80);

    // make sure we decoded the expected number of bits
    if (packet_bits.bits_per_row[0] < 80) {
        // decoder_logf(decoder, 0, __func__, "bitpos=%u start_pos=%u = %u", bitpos, start_pos, (start_pos - bitpos));
        return 0; // DECODE_FAIL_SANITY;
    }

    uint8_t *b = packet_bits.bb[0];

    // Checksum is CRC-16 poly 0x1021 init 0xffff over 8 bytes
    int checksum = crc16(b, 10, 0x1021, 0xffff);
    if (checksum != 0) {
        return 0; // DECODE_FAIL_MIC;
    }

    int id          = (unsigned)b[0] << 24 | b[1] << 16 | b[2] << 8 | b[3];
    int pressure    = b[4];
    int temperature = b[5];
    int flags       = b[6] << 8 | b[7];

    int pressure_kpa  = pressure * 5 / 2 - 100;
    int temperature_c = temperature - 40;

    char id_str[4 * 2 + 1];
    snprintf(id_str, sizeof(id_str), "%08x", id);

    /* clang-format off */
    data_t *data = data_make(
            "model",            "",             DATA_STRING, "Porsche",
            "type",             "",             DATA_STRING, "TPMS",
            "id",               "",             DATA_STRING, id_str,
            "pressure_kPa",     "Pressure",     DATA_FORMAT, "%.1f kPa",    DATA_DOUBLE, (float)pressure_kpa,
            "temperature_C",    "Temperature",  DATA_FORMAT, "%.0f C",      DATA_DOUBLE, (float)temperature_c,
            "flags",            "",             DATA_FORMAT, "%04x",        DATA_INT,    flags,
            "mic",              "Integrity",    DATA_STRING, "CRC",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

/** @sa tpms_porsche_decode() */
static int tpms_porsche_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    // Full preamble is {30}ccccccca (33333332).
    uint8_t const preamble_pattern[] = {0x33, 0x33, 0x20}; // 20 bit

    int events = 0;

    // Find a preamble with enough bits after it that it could be a complete packet
    unsigned bitpos = 0;
    while ((bitpos = bitbuffer_search(bitbuffer, 0, bitpos, preamble_pattern, 20)) + 100 <=
            bitbuffer->bits_per_row[0]) {
        events += tpms_porsche_decode(decoder, bitbuffer, 0, bitpos + 20);
        bitpos += 2;
    }

    return events;
}

/** Encoder: inverse of tpms_porsche_decode().

    Patches id, pressure, temperature and flags in every packet, recomputes
    the CRC-16 and re-encodes the differential Manchester payload in place.
*/
static int tpms_porsche_encode(r_device const *decoder, struct capture *cap, bitbuffer_t *out_bits)
{
    uint8_t const preamble_pattern[] = {0x33, 0x33, 0x20};

    if (cap->bits.num_rows < 1)
        return -1;
    bitbuffer_t bits = cap->bits;

    int patched     = 0;
    unsigned bitpos = 0;
    while ((bitpos = bitbuffer_search(&bits, 0, bitpos, preamble_pattern, 20)) + 100 <= bits.bits_per_row[0]) {
        unsigned start     = bitpos + 20;
        bitbuffer_t packet = {0};
        bitbuffer_differential_manchester_decode(&bits, 0, start, &packet, 80);
        if (packet.bits_per_row[0] < 80 || crc16(packet.bb[0], 10, 0x1021, 0xffff) != 0) {
            bitpos += 2;
            continue;
        }
        uint8_t b[10];
        memcpy(b, packet.bb[0], 10);

        char const *f;
        int v;
        double d;
        if ((f = capture_get_field(cap, "id"))) {
            uint32_t id = (uint32_t)strtoul(f, NULL, 16);
            b[0]        = (uint8_t)(id >> 24);
            b[1]        = (uint8_t)(id >> 16);
            b[2]        = (uint8_t)(id >> 8);
            b[3]        = (uint8_t)id;
        }
        if (encoder_field_dbl(cap, "pressure_kPa", &d)) {
            int best = 0;
            for (int r = 1; r < 256; r++)
                if (fabs(r * 5 / 2 - 100 - d) < fabs(best * 5 / 2 - 100 - d))
                    best = r;
            b[4] = (uint8_t)best;
            encoder_set_dbl(cap, "pressure_kPa", best * 5 / 2 - 100);
        }
        if (encoder_field_dbl(cap, "temperature_C", &d)) {
            long t = lround(d) + 40;
            b[5]   = (uint8_t)(t < 0 ? 0 : t > 0xff ? 0xff : t);
            encoder_set_dbl(cap, "temperature_C", b[5] - 40);
        }
        if (encoder_field_int(cap, "flags", &v)) {
            b[6] = (uint8_t)(v >> 8);
            b[7] = (uint8_t)v;
        }
        uint16_t crc = crc16(b, 8, 0x1021, 0xffff);
        b[8]         = (uint8_t)(crc >> 8);
        b[9]         = (uint8_t)crc;

        int prev = bitrow_get_bit(bits.bb[0], start - 1);
        encoder_diff_manchester_encode(bits.bb[0], start, &prev, b, 80);
        patched++;
        bitpos = start + 160;
    }
    if (!patched)
        return -1;

    *out_bits = bits;
    return encoder_verify(decoder, out_bits, cap) ? 1 : -1;
}

static char const *const output_fields[] = {
        "model",
        "type",
        "id",
        "pressure",
        "temperature_C",
        "flags",
        "mic",
        NULL,
};

r_device const tpms_porsche = {
        .name        = "Porsche Boxster/Cayman TPMS",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 52,  // 12-13 samples @250k
        .long_width  = 52,  // FSK
        .reset_limit = 150, // Maximum gap size before End Of Message [us].
        .decode_fn   = &tpms_porsche_callback,
        .encode_fn   = &tpms_porsche_encode,
        .fields      = output_fields,
};
