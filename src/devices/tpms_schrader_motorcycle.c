/** @file
    Schrader Motorcycle TPMS sensor.

    Copyright (C) 2026 Bruno OCTAU (ProfBoc75)

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/
/**
Schrader Motorcycle TPMS sensor.

issue #3512

Sensor information on the photos:

    PN:3142M / 29074
    R-C-SRD-RDC3
    CNC ID: H-16089
    ANATEL:0651-16-8001
    FCC ID MRXRDC3
    IC: 2546A-RDC3
    M:RDC3                 CIDF15000486

Flex decoder:

    rtl_433 rtl_433 -X 'n=name,m=OOK_MC_ZEROBIT,s=122,l=122,r=375,y=500,bits>=64,preamble={13}7fff'

    codes:   {56}fd420b3ddc5352
    codes:   {56}fd420b3dd95313
    codes:   {56}fd420b3de55316
    codes:   {56}fd420b3de55403

RF signal:

    OOK, Manchester Code with fixed leading zero bit, s=l=122 µs,

Data layout{56} 7 byte:

    Byte Position                            0         1         2         3         4         5         6
    Bit Position                     8765 4321 8765 4321 8765 4321 8765 4321 8765 4321 8765 4321 8765 4321
    Data Layout   [pppp pppp pppp p][FFFF FFII IIII IIII IIII IIII IIII IIPP PPPP PPPP TTTT TTTT]CCCC CCCC
    Sample            7    f    f 8     f    d    4    2    0    b    3    d    d    c    5    3    5    2

- pp:{13} Preamble, fixed 0b0111111111111 = {13}0x7ff8
- FF: {6} Looks fixed, was always 0b111111 from sample codes
- II:{24} Sensor ID, decimal value
- PP:{10} Tire pressure, kPa scale 2
- TT:{8}: Temperature in C offset 50
- CC: CRC-8 bits,poly 0x07, init 0xe0, final xor 0x00 [from previous bytes].

*/

#include "decoder.h"
#include "r_encoder.h"
#include <stdlib.h>
#include <math.h>

static int tpms_schrader_motorcycle_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    uint8_t b[7]  = {0};
    // preamble off 13 bit
    uint8_t const preamble_pattern[] = {0x7f, 0xf8};

    if (bitbuffer->num_rows != 1) {
        decoder_logf(decoder, 2, __func__, "Row error");
        return DECODE_ABORT_EARLY;
    }

    int pos = 0;
    int len = bitbuffer->bits_per_row[0];
    pos = bitbuffer_search(bitbuffer, 0, 0, preamble_pattern, 13); // preamble based on 13 bit
    if (pos >= len) {
        decoder_logf(decoder, 2, __func__, "Preamble not found");
        return DECODE_ABORT_EARLY;
    }

    pos += 13;

    if (len - pos < 56) {
        decoder_logf(decoder, 2, __func__, "Too short");
        return DECODE_ABORT_LENGTH;
    }

    bitbuffer_extract_bytes(bitbuffer, 0, pos, b, sizeof(b) * 8);

    decoder_log_bitrow(decoder, 2, __func__, b, 56, "MSG");

    if (crc8(b, 7, 0x07, 0xe0)) {
        decoder_logf(decoder, 2, __func__, "CRC error");
        return DECODE_FAIL_MIC; // crc mismatch
    }

    uint32_t id             = ((uint32_t)(b[0] & 0x03) << 22) | (b[1] << 14) | (b[2] << 6) | (b[3] >> 2);
    uint16_t pressure_raw   = ((b[3] & 0x03) << 8) | b[4];
    float pressure_kPa      = pressure_raw * 0.5f;
    float temperature_C     = (b[5] - 50);

    /* clang-format off */
    data_t *data = data_make(
            "model",               "",                DATA_STRING, "Schrader-Motorcycle",
            "type",                "",                DATA_STRING, "TPMS",
            "id",                  "",                DATA_FORMAT, "%u", DATA_INT, id,
            "pressure_kPa",        "Pressure",        DATA_FORMAT, "%.1f kPa", DATA_DOUBLE, (double)pressure_kPa,
            "temperature_C",       "Temperature",     DATA_FORMAT, "%.1f C",   DATA_DOUBLE, temperature_C,
            "mic",                 "Integrity",       DATA_STRING, "CRC",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

/** Encoder: inverse of tpms_schrader_motorcycle_decode().

    The demodulator already Manchester-decodes, so the row holds NRZ bits.
    Keeps the captured preamble and fixed bits, patches id, pressure and
    temperature, recomputes the CRC-8 and writes the 7 message bytes in place.
*/
static int tpms_schrader_motorcycle_encode(r_device const *decoder, struct capture *cap, bitbuffer_t *out_bits)
{
    uint8_t const preamble_pattern[] = {0x7f, 0xf8};

    if (cap->bits.num_rows != 1)
        return -1;

    bitbuffer_t src = cap->bits;
    unsigned len = src.bits_per_row[0];
    unsigned pos = bitbuffer_search(&src, 0, 0, preamble_pattern, 13);
    if (pos + 13 + 56 > len)
        return -1;
    pos += 13;

    uint8_t b[7] = {0};
    bitbuffer_extract_bytes(&src, 0, pos, b, 56);

    char const *f;
    if ((f = capture_get_field(cap, "id"))) {
        uint32_t id = (uint32_t)strtoul(f, NULL, 10) & 0xffffff;
        b[0] = (uint8_t)((b[0] & 0xfc) | (id >> 22));
        b[1] = (uint8_t)(id >> 14);
        b[2] = (uint8_t)(id >> 6);
        b[3] = (uint8_t)((b[3] & 0x03) | ((id & 0x3f) << 2));
    }
    double d;
    if (encoder_field_dbl(cap, "pressure_kPa", &d)) {
        long p = lround(d * 2.0);
        if (p < 0) p = 0;
        if (p > 0x3ff) p = 0x3ff;
        b[3] = (uint8_t)((b[3] & 0xfc) | (p >> 8));
        b[4] = (uint8_t)(p & 0xff);
        encoder_set_dbl(cap, "pressure_kPa", p * 0.5);
    }
    if (encoder_field_dbl(cap, "temperature_C", &d)) {
        long t = lround(d) + 50;
        if (t < 0) t = 0;
        if (t > 0xff) t = 0xff;
        b[5] = (uint8_t)t;
        encoder_set_dbl(cap, "temperature_C", t - 50);
    }

    b[6] = crc8(b, 6, 0x07, 0xe0);

    for (unsigned k = 0; k < 56; ++k)
        encoder_set_bit(src.bb[0], pos + k, (b[k / 8] >> (7 - (k % 8))) & 1);

    *out_bits = src;

    if (!encoder_verify(decoder, out_bits, cap))
        return -1;
    return 1;
}

static char const *const output_fields[] = {
        "model",
        "type",
        "id",
        "pressure_kPa",
        "temperature_C",
        "mic",
        NULL,
};

r_device const tpms_schrader_motorcycle = {
        .name        = "Schrader Motorcycle TPMS sensor",
        .modulation  = OOK_PULSE_MANCHESTER_ZEROBIT,
        .short_width = 122,
        .long_width  = 122,
        .reset_limit = 375,
        .decode_fn   = &tpms_schrader_motorcycle_decode,
        .encode_fn   = &tpms_schrader_motorcycle_encode,
        .fields      = output_fields,
};
