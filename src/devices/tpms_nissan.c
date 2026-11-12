/** @file
    Nissan FSK 37 bit Manchester encoded checksummed TPMS data.
    Reference issue: https://github.com/merbanan/rtl_433/issues/1024

    Copyright (C) 2021 Alex Wilson <alex.david.wilson@gmail.com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

#include "decoder.h"
#include "capture.h"
#include "r_encoder.h"
#include <math.h>
#include <stdlib.h>

// sum2N checksum, expects a 0x03 result for a valid message
// See https://github.com/merbanan/rtl_433/issues/1024#issuecomment-2592665745
static uint8_t tpms_nissan_checksum(uint8_t const *b)
{
    uint8_t chk = 0;
    for (int i = 0; i < 4; i++) {
        chk += b[i] >> 7;
        chk += b[i] >> 5;
        chk += b[i] >> 3;
        chk += b[i] >> 1;
        chk += (uint8_t)(b[i] << 1);
    }
    chk += b[4] >> 7;
    chk += b[4] >> 5;
    chk += b[4] >> 3;
    return ~chk & 0x03;
}

/**
Nissan FSK 37 bit Manchester encoded checksummed TPMS data.

Data format:

    MODE:3d TPMS_ID:24h (PSI+THREE)*FOUR=8d UNKNOWN:2b
*/
static int tpms_nissan_decode(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos)
{
    bitbuffer_t packet_bits = {0};

    bitbuffer_manchester_decode(bitbuffer, row, bitpos, &packet_bits, 113);

    if (packet_bits.bits_per_row[0] < 37) {
        return DECODE_FAIL_SANITY;
    }

    bitbuffer_invert(&packet_bits); // Manchester (G.E. Thomas) Decoded

    uint8_t *b = packet_bits.bb[0];

    if (tpms_nissan_checksum(b) != 0) {
        return DECODE_FAIL_MIC;
    }

    // MODE:3d
    int mode = b[0] >> 5;

    // TPMS_ID:24h
    int id = (unsigned)((b[0] & 0x1F) << 19) | (b[1] << 11) | (b[2] << 3) | (b[3] >> 5);

    // (PSI+THREE)*FOUR=8d
    int pressure_raw   = ((b[3] & 0x1F) << 3) | (b[4] >> 5);
    float pressure_psi = (float)(pressure_raw / 4.0f - 3.0f);

    // UNKNOWN:2b
    int unknown = (b[4] & 0x1F) >> 3;

    char id_str[7];
    snprintf(id_str, sizeof(id_str), "%06x", id);

    /* clang-format off */
    data_t *data = data_make(
            "model",            "",             DATA_STRING, "Nissan",
            "type",             "",             DATA_STRING, "TPMS",
            "id",               "",             DATA_STRING, id_str,
            "mode",             "",             DATA_INT,    mode,
            "pressure_PSI",     "Pressure",     DATA_FORMAT, "%.1f PSI", DATA_DOUBLE, pressure_psi,
            "unknown",          "",             DATA_INT,    unknown,
            "mic",              "Integrity",    DATA_STRING, "CHECKSUM",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

/** Encoder: inverse of tpms_nissan_decode().

    Rebuilds the raw FSK-PCM/Manchester bitstream from (edited) decoded fields,
    keeping the captured preamble and re-encoding the 37-bit Manchester payload
    in place. The two "unknown" bits double as checksum bits and are solved so
    the sum2N checksum validates.
*/
static int tpms_nissan_encode(r_device const *decoder, struct capture *cap, bitbuffer_t *out_bits)
{
    if (cap->bits.num_rows < 1)
        return -1;

    uint8_t const preamble_pattern[5] = {0xf5, 0x55, 0x55, 0x55, 0xe0}; // 36 bits

    bitbuffer_t src = cap->bits;
    unsigned bitpos = bitbuffer_search(&src, 0, 0, preamble_pattern, 36);
    if (bitpos + 36 + 74 > src.bits_per_row[0])
        return -1;
    unsigned pstart = bitpos + 36;

    int mode = 0, unknown = 0;
    unsigned id = 0;
    double psi = 0.0;
    char const *f;
    encoder_field_int(cap, "mode", &mode);
    encoder_field_int(cap, "unknown", &unknown);
    if ((f = capture_get_field(cap, "id")))
        id = (unsigned)strtoul(f, NULL, 16);
    encoder_field_dbl(cap, "pressure_PSI", &psi);

    int pressure_raw = (int)lround((psi + 3.0) * 4.0);
    if (pressure_raw < 0) pressure_raw = 0;
    if (pressure_raw > 255) pressure_raw = 255;

    uint8_t b[5];
    b[0] = (uint8_t)(((mode & 7) << 5) | ((id >> 19) & 0x1f));
    b[1] = (uint8_t)(id >> 11);
    b[2] = (uint8_t)(id >> 3);
    b[3] = (uint8_t)(((id & 7) << 5) | ((pressure_raw >> 3) & 0x1f));

    // the "unknown" field carries the checksum; solve it so sum2N validates
    int solved = -1;
    for (int u = 0; u < 4; ++u) {
        b[4] = (uint8_t)(((pressure_raw & 7) << 5) | ((u & 3) << 3));
        if (tpms_nissan_checksum(b) == 0) {
            solved = u;
            break;
        }
    }
    if (solved < 0)
        return -1;
    if (solved != unknown) {
        char ustr[8];
        snprintf(ustr, sizeof(ustr), "%d", solved);
        capture_set_field(cap, "unknown", ustr);
    }

    // re-encode 37 data bits as Manchester (decoder inverts after decode)
    encoder_manchester_encode(src.bb[0], pstart, b, 37, 1);

    *out_bits = src;

    if (!encoder_verify(decoder, out_bits, cap))
        return -1;
    return 1;
}

/** @sa tpms_nissan_decode() */
static int tpms_nissan_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    // preamble is f5 55 55 55 e
    uint8_t const preamble_pattern[5] = {0xf5, 0x55, 0x55, 0x55, 0xe0}; // 36 bits

    unsigned bitpos = 0;
    int ret         = 0;
    int events      = 0;

    // Find a preamble with enough bits after it that it could be a complete packet
    while ((bitpos = bitbuffer_search(bitbuffer, 0, bitpos, preamble_pattern, 36)) + 77 <=
            bitbuffer->bits_per_row[0]) {
        ret = tpms_nissan_decode(decoder, bitbuffer, 0, bitpos + 36);
        if (ret > 0)
            events += ret;
        bitpos += 1;
    }

    return events > 0 ? events : ret;
}

static char const *const output_fields[] = {
        "model",
        "type",
        "id",
        "mode",
        "pressure_PSI",
        "unknown",
        "mic",
        NULL,
};

r_device const tpms_nissan = {
        .name        = "Nissan TPMS",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 120, // TODO The preamble plus pre-MC data is 113, what should this be?
        .long_width  = 120, // FSK
        .reset_limit = 250, // Maximum gap size before End Of Message [us]. TODO What should this be?
        .decode_fn   = &tpms_nissan_callback,
        .encode_fn   = &tpms_nissan_encode,
        .disabled    = 0,
        .fields      = output_fields,
};
