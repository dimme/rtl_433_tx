/** @file
    Mercedes Benz Sprinter TPMS data.

    Copyright (C) 2026 Bruno OCTAU (ProfBoc75)

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/
/**
Mercedes Benz Sprinter TPMS data.

issue #3539:
- 2025 Mercedes Benz Sprinter 4500 TPMS sensor
- Signal is FSK MC and pulse width 25us.
- Preamble {12} 0x002 or 0xff2

Data Layout:

    Preamble    Byte  0  1  2  3  4  5  6  7  8  9
    002 or ff2        ST II II II II PP TT CC FF XX

- ST:  {8} Family/state byte, observed as 0x83 while stationary and 0xa3 while
           moving. These capture-derived values are not known to be exhaustive.
- II: {32} ID / Serial number
- PP:  {8} Pressure, PSI, scale 2.75
- TT:  {8} Temperature, C, offset 51
- CC:  {8} Counter, increase by 1 each message
- FF:  {8} Status, observed as 0x6b, 0xe9, or 0xea
- XX:  {8} CRC-8 of the previous 9 bytes, poly 0x2f, init 0xaa, final XOR 0x00

*/

#include "decoder.h"
#include "r_encoder.h"
#include <stdlib.h>
#include <math.h>

static int tpms_mercedes_benz_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    uint8_t const preamble[] = {0x00,0x20}; // 12 bit , 0x002
    uint8_t b[10];

    if (bitbuffer->num_rows != 1) {
        decoder_logf(decoder, 2, __func__, "row error");
        return DECODE_ABORT_EARLY;
    }

    int pos = bitbuffer_search(bitbuffer, 0, 0, preamble, 12);
    if (pos >= bitbuffer->bits_per_row[0]) {
        decoder_logf(decoder, 2, __func__, "Preamble 0x002 not found");
        return DECODE_ABORT_EARLY;
    }

    if (bitbuffer->bits_per_row[0] < 80) {
        decoder_logf(decoder, 1, __func__, "Too short");
        return DECODE_ABORT_LENGTH;
    }

    pos += 12;
    bitbuffer_extract_bytes(bitbuffer, 0, pos, b, 80);

    decoder_log_bitrow(decoder, 2, __func__, b, 80, "MSG");

    if (crc8(b, 10, 0x2f, 0xaa)) {
        decoder_logf(decoder, 1, __func__, "crc error, expected %02x, calculated %02x", b[9], crc8(b, 9, 0x2f, 0xaa));
        return DECODE_FAIL_MIC; // crc mismatch
    }

    if (b[0] != 0x83 && b[0] != 0xa3) {
        decoder_logf(decoder, 1, __func__, "unexpected family/state byte %02x", b[0]);
        return DECODE_FAIL_SANITY;
    }

    char id_str[4 * 2 + 1];
    snprintf(id_str, sizeof(id_str), "%02x%02x%02x%02x", b[1], b[2], b[3], b[4]);
    float pressure_PSI = b[5] / 2.75f;
    int temperature_C  = b[6] - 51;
    int counter        = b[7] & 0x1f;
    int flags1         = b[7] >> 5;
    int flags2         = b[8];

    /* clang-format off */
    data_t *data = data_make(
            "model",           "",            DATA_STRING, "MercedesBenz-Sprinter",
            "type",            "",            DATA_STRING, "TPMS",
            "id",              "",            DATA_STRING, id_str,
            "pressure_PSI",    "Pressure",    DATA_FORMAT, "%.1f PSI", DATA_DOUBLE, (double)pressure_PSI,
            "temperature_C",   "Temperature", DATA_FORMAT, "%.1f C",   DATA_DOUBLE, (double)temperature_C,
            "counter",         "Counter",     DATA_INT,    counter,
            "flags1",          "Flags 1",     DATA_FORMAT, "0b%03b",     DATA_INT,    flags1,
            "flags2",          "Flags 2",     DATA_INT,    flags2,
            "mic",             "Integrity",   DATA_STRING, "CRC",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

/** Encoder: inverse of tpms_mercedes_benz_decode(). */
static int tpms_mercedes_benz_encode(r_device const *decoder, struct capture *cap, bitbuffer_t *out_bits)
{
    uint8_t const preamble[] = {0x00, 0x20};

    if (cap->bits.num_rows != 1)
        return -1;
    bitbuffer_t bits = cap->bits;

    unsigned pos = bitbuffer_search(&bits, 0, 0, preamble, 12);
    if (pos + 12 + 80 > bits.bits_per_row[0])
        return -1;
    pos += 12;

    uint8_t b[10];
    bitbuffer_extract_bytes(&bits, 0, pos, b, 80);
    if (crc8(b, 10, 0x2f, 0xaa))
        return -1;

    int v;
    double d;
    char const *f = capture_get_field(cap, "id");
    if (f) {
        unsigned id = (unsigned)strtoul(f, NULL, 16);
        b[1]        = (uint8_t)(id >> 24);
        b[2]        = (uint8_t)(id >> 16);
        b[3]        = (uint8_t)(id >> 8);
        b[4]        = (uint8_t)id;
    }
    if (encoder_field_dbl(cap, "pressure_PSI", &d)) {
        long p = lround(d * 2.75);
        b[5]   = (uint8_t)(p < 0 ? 0 : p > 0xff ? 0xff : p);
    }
    if (encoder_field_dbl(cap, "temperature_C", &d)) {
        long t = lround(d) + 51;
        b[6]   = (uint8_t)(t < 0 ? 0 : t > 0xff ? 0xff : t);
    }
    if (encoder_field_int(cap, "counter", &v))
        b[7] = (uint8_t)((b[7] & 0xe0) | (v & 0x1f));
    if (encoder_field_int(cap, "flags1", &v))
        b[7] = (uint8_t)((v & 0x07) << 5 | (b[7] & 0x1f));
    if (encoder_field_int(cap, "flags2", &v))
        b[8] = (uint8_t)v;
    b[9] = crc8(b, 9, 0x2f, 0xaa);

    for (unsigned k = 0; k < 80; k++)
        encoder_set_bit(bits.bb[0], pos + k, (b[k / 8] >> (7 - (k % 8))) & 1);

    char buf[16];
    snprintf(buf, sizeof(buf), "%02x%02x%02x%02x", b[1], b[2], b[3], b[4]);
    capture_set_field(cap, "id", buf);
    encoder_set_dbl(cap, "pressure_PSI", (float)(b[5] / 2.75f));
    encoder_set_dbl(cap, "temperature_C", b[6] - 51.0);
    snprintf(buf, sizeof(buf), "%d", b[7] & 0x1f);
    capture_set_field(cap, "counter", buf);
    snprintf(buf, sizeof(buf), "%d", b[7] >> 5);
    capture_set_field(cap, "flags1", buf);
    snprintf(buf, sizeof(buf), "%d", b[8]);
    capture_set_field(cap, "flags2", buf);

    *out_bits = bits;
    return encoder_verify(decoder, out_bits, cap) ? 1 : -1;
}

static char const *const output_fields[] = {
        "model",
        "type",
        "id",
        "pressure_PSI",
        "temperature_C",
        "counter",
        "flags1",
        "flags2",
        "mic",
        NULL,
};

r_device const tpms_mercedes_benz = {
        .name        = "Mercedes Benz Sprinter 4500 TPMS sensor",
        .modulation  = FSK_PULSE_MANCHESTER_ZEROBIT,
        .short_width = 25,
        .long_width  = 25,
        .reset_limit = 2000,
        .decode_fn   = &tpms_mercedes_benz_decode,
        .encode_fn   = &tpms_mercedes_benz_encode,
        .fields      = output_fields,
};
