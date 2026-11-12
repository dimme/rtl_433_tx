/** @file
    Citroen FSK 10 byte Manchester encoded checksummed TPMS data.

    Copyright (C) 2017 Christian W. Zuckschwerdt <zany@triq.net>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/
/**
Citroen FSK 10 byte Manchester encoded checksummed TPMS data
also Peugeot and likely Fiat, Mitsubishi, VDO-types.


Packet nibbles:

    UU  IIIIIIII FR  PP TT BB  CC

- U = state, decoding unknown, not included in checksum
- I = id
- F = flags, (seen: 0: 69.4% 1: 0.8% 6: 0.4% 8: 1.1% b: 1.9% c: 25.8% e: 0.8%)
- R = repeat counter (seen: 0,1,2,3)
- P = Pressure (kPa in 1.364 steps, about fifth PSI?)
- T = Temperature (deg C offset by 50)
- B = Battery?
- C = Checksum, XOR bytes 1 to 9 = 0
*/

#include "decoder.h"
#include "r_encoder.h"
#include <stdlib.h>
#include <math.h>

static int tpms_citroen_decode(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos)
{
    bitbuffer_t packet_bits = {0};
    uint8_t *b;
    int state;
    unsigned id;
    int flags;
    int repeat;
    int pressure;
    int temperature;
    int maybe_battery;
    int crc;

    bitbuffer_manchester_decode(bitbuffer, row, bitpos, &packet_bits, 88);

    // decoder_logf(decoder, 3, __func__, "bits %d", packet_bits.bits_per_row[0]);
    if (packet_bits.bits_per_row[0] < 80) {
        return DECODE_FAIL_SANITY; // sanity check failed
    }

    b = packet_bits.bb[0];

    if (b[6] == 0 || b[7] == 0) {
        return DECODE_ABORT_EARLY; // sanity check failed
    }

    crc = b[1] ^ b[2] ^ b[3] ^ b[4] ^ b[5] ^ b[6] ^ b[7] ^ b[8] ^ b[9];
    if (crc != 0) {
        return DECODE_FAIL_MIC; // bad checksum
    }

    state         = b[0]; // not covered by CRC
    id            = (unsigned)b[1] << 24 | b[2] << 16 | b[3] << 8 | b[4];
    flags         = b[5] >> 4;
    repeat        = b[5] & 0x0f;
    pressure      = b[6];
    temperature   = b[7];
    maybe_battery = b[8];

    char state_str[3];
    snprintf(state_str, sizeof(state_str), "%02x", state);
    char id_str[9];
    snprintf(id_str, sizeof(id_str), "%08x", id);

    /* clang-format off */
    data_t *data = data_make(
            "model",            "",             DATA_STRING, "Citroen",
            "type",             "",             DATA_STRING, "TPMS",
            "id",               "",             DATA_STRING, id_str,
            "state",            "",             DATA_STRING, state_str,
            "flags",            "",             DATA_INT,    flags,
            "repeat",           "",             DATA_INT,    repeat,
            "pressure_kPa",     "Pressure",     DATA_FORMAT, "%.0f kPa", DATA_DOUBLE, (double)pressure * 1.364,
            "temperature_C",    "Temperature",  DATA_FORMAT, "%.0f C", DATA_DOUBLE, (double)temperature - 50.0,
            "maybe_battery",    "",             DATA_INT,    maybe_battery,
            "mic",              "Integrity",    DATA_STRING, "CHECKSUM",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

/** @sa tpms_citroen_decode() */
static int tpms_citroen_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    // full preamble is 55 55 55 56 (inverted: aa aa aa a9)
    uint8_t const preamble_pattern[2] = {0xaa, 0xa9}; // 16 bits
    // full trailer is 01111110

    unsigned bitpos = 0;
    int ret         = 0;
    int events      = 0;

    bitbuffer_invert(bitbuffer);

    // Find a preamble with enough bits after it that it could be a complete packet
    while ((bitpos = bitbuffer_search(bitbuffer, 0, bitpos, preamble_pattern, 16)) + 178 <=
            bitbuffer->bits_per_row[0]) {
        ret = tpms_citroen_decode(decoder, bitbuffer, 0, bitpos + 16);
        if (ret > 0)
            events += ret;
        bitpos += 2;
    }

    return events > 0 ? events : ret;
}

/** Encoder: inverse of tpms_citroen_decode().

    Works on the inverted row like the decoder. For each preamble with a
    valid payload, patches state, id, flags, repeat, pressure, temperature and
    maybe_battery, recomputes the XOR checksum and re-encodes the Manchester
    payload in place.
*/
static int tpms_citroen_encode(r_device const *decoder, struct capture *cap, bitbuffer_t *out_bits)
{
    uint8_t const preamble_pattern[2] = {0xaa, 0xa9};

    if (cap->bits.num_rows < 1)
        return -1;

    bitbuffer_t inv = cap->bits;
    bitbuffer_invert(&inv);

    char const *f;
    int v;
    double d;
    int found = 0;
    unsigned bitpos = 0;
    while ((bitpos = bitbuffer_search(&inv, 0, bitpos, preamble_pattern, 16)) + 178 <= inv.bits_per_row[0]) {
        unsigned start = bitpos + 16;
        bitpos += 2;
        bitbuffer_t packet = {0};
        bitbuffer_manchester_decode(&inv, 0, start, &packet, 80);
        if (packet.bits_per_row[0] < 80)
            continue;
        uint8_t b[10];
        memcpy(b, packet.bb[0], 10);
        if (b[6] == 0 || b[7] == 0 || xor_bytes(&b[1], 9) != 0)
            continue;

        if ((f = capture_get_field(cap, "state")))
            b[0] = (uint8_t)strtoul(f, NULL, 16);
        if ((f = capture_get_field(cap, "id"))) {
            uint32_t id = (uint32_t)strtoul(f, NULL, 16);
            b[1] = (uint8_t)(id >> 24);
            b[2] = (uint8_t)(id >> 16);
            b[3] = (uint8_t)(id >> 8);
            b[4] = (uint8_t)id;
        }
        if (encoder_field_int(cap, "flags", &v))
            b[5] = (uint8_t)((b[5] & 0x0f) | ((v & 0x0f) << 4));
        if (encoder_field_int(cap, "repeat", &v))
            b[5] = (uint8_t)((b[5] & 0xf0) | (v & 0x0f));
        if (encoder_field_dbl(cap, "pressure_kPa", &d)) {
            long p = lround(d / 1.364);
            b[6] = (uint8_t)(p < 1 ? 1 : p > 0xff ? 0xff : p); // 0 is rejected by the decoder
            encoder_set_dbl(cap, "pressure_kPa", b[6] * 1.364);
        }
        if (encoder_field_dbl(cap, "temperature_C", &d)) {
            long t = lround(d) + 50;
            b[7] = (uint8_t)(t < 1 ? 1 : t > 0xff ? 0xff : t);
            encoder_set_dbl(cap, "temperature_C", b[7] - 50);
        }
        if (encoder_field_int(cap, "maybe_battery", &v))
            b[8] = (uint8_t)v;

        b[9] = (uint8_t)xor_bytes(&b[1], 8);

        encoder_manchester_encode(inv.bb[0], start, b, 80, 0);
        found = 1;
    }
    if (!found)
        return -1;

    bitbuffer_invert(&inv);
    *out_bits = inv;

    if (!encoder_verify(decoder, out_bits, cap))
        return -1;
    return 1;
}

static char const *const output_fields[] = {
        "model",
        "type",
        "id",
        "state",
        "flags",
        "repeat",
        "pressure_kPa",
        "temperature_C",
        "maybe_battery",
        "code",
        "mic",
        NULL,
};

r_device const tpms_citroen = {
        .name        = "Citroen TPMS",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 52,  // 12-13 samples @250k
        .long_width  = 52,  // FSK
        .reset_limit = 150, // Maximum gap size before End Of Message [us].
        .decode_fn   = &tpms_citroen_callback,
        .encode_fn   = &tpms_citroen_encode,
        .fields      = output_fields,
};
