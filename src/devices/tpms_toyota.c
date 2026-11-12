/** @file
    FSK 9-byte Differential Manchester encoded TPMS data with CRC-8.

    Copyright (C) 2017 Christian W. Zuckschwerdt <zany@triq.net>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/
/**
FSK 9-byte Differential Manchester encoded TPMS data with CRC-8.
Pacific Industries Co.Ltd. PMV-C210
Seen on a Toyota Auris(Corolla). The manufacturers of the Toyota TPMS are
Pacific Industrial Corp and sometimes TRW Automotive and might also be used
in other car brands. Contact me with your observations!

There are 14 bits sync followed by 72 bits manchester encoded data and
3 bits trailer.
E.g. 01010101001111 00110011 [...64 manchester bits] 00101010111

The first 4 bytes are the ID. Followed by 1-bit state,
8-bit values of pressure, temperature, 7-bit state, 8-bit inverted pressure
and then the a CRC-8 with 0x07 truncated poly and init 0x80.
The temperature is offset by 40 deg C.
The pressure seems to be 1/4 PSI offset by -7 PSI (i.e. 28 raw = 0 PSI).
*/

#include "decoder.h"
#include "capture.h"
#include "r_encoder.h"
#include <math.h>
#include <stdlib.h>

static int tpms_toyota_decode(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos)
{
    unsigned int start_pos;
    bitbuffer_t packet_bits = {0};
    uint8_t *b;
    unsigned id;
    unsigned status, pressure1, pressure2, temp;
    int crc;

    // skip the first 1 bit, i.e. raw "01" to get 72 bits
    start_pos = bitbuffer_differential_manchester_decode(bitbuffer, row, bitpos, &packet_bits, 80);
    if (start_pos - bitpos < 144) {
        return 0;
    }
    b = packet_bits.bb[0];

    crc = b[8];
    if (crc8(b, 8, 0x07, 0x80) != crc) {
        return 0;
    }

    id        = (unsigned)b[0] << 24 | b[1] << 16 | b[2] << 8 | b[3];
    status    = (b[4] & 0x80) | (b[6] & 0x7f); // status bit and 0 filler
    pressure1 = (b[4] & 0x7f) << 1 | b[5] >> 7;
    temp      = (b[5] & 0x7f) << 1 | b[6] >> 7;
    pressure2 = b[7] ^ 0xff;

    if (pressure1 != pressure2) {
        decoder_logf(decoder, 1, __func__, "Toyota TPMS pressure check error: %02x vs %02x", pressure1, pressure2);
        return 0;
    }

    char id_str[9];
    snprintf(id_str, sizeof(id_str), "%08x", id);

    /* clang-format off */
    data_t *data = data_make(
            "model",            "",             DATA_STRING,    "Toyota",
            "type",             "",             DATA_STRING,    "TPMS",
            "id",               "",             DATA_STRING,    id_str,
            "status",           "",             DATA_INT,       status,
            "pressure_PSI",     "",             DATA_DOUBLE,    pressure1*0.25-7.0,
            "temperature_C",    "",             DATA_FORMAT,    "%.0f C", DATA_DOUBLE, temp - 40.0,
            "mic",              "Integrity",    DATA_STRING,    "CRC",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

/** @sa tpms_toyota_decode() */
static int tpms_toyota_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    // full preamble is 0101 0101 0011 11 = 55 3c
    // could be shorter   11 0101 0011 11
    uint8_t const preamble_pattern[2] = {0xa9, 0xe0}; // 12 bits (but pass last bit to decode)

    unsigned bitpos = 0;
    int ret         = 0;
    int events      = 0;

    // Find a preamble with enough bits after it that it could be a complete packet
    while ((bitpos = bitbuffer_search(bitbuffer, 0, bitpos, preamble_pattern, 12)) + 156 <=
            bitbuffer->bits_per_row[0]) {
        ret = tpms_toyota_decode(decoder, bitbuffer, 0, bitpos + 11);
        if (ret > 0)
            events += ret;
        bitpos += 2;
    }

    return events > 0 ? events : ret;
}

static inline void bitrow_set_bit(uint8_t *bitrow, unsigned idx, int val)
{
    if (val)
        bitrow[idx / 8] |= (uint8_t)(1 << (7 - (idx % 8)));
    else
        bitrow[idx / 8] &= (uint8_t)~(1 << (7 - (idx % 8)));
}

/** Encoder: inverse of tpms_toyota_decode().

    Rebuilds the raw FSK-PCM bitstream from (edited) decoded fields using the
    captured transmission as a template: the sync preamble and trailer are kept,
    and only the 72-bit differential-Manchester payload is re-encoded in place.
*/
static int tpms_toyota_encode(r_device const *decoder, struct capture *cap, bitbuffer_t *out_bits)
{
    if (cap->bits.num_rows < 1)
        return -1; // need a captured template to encode against

    uint8_t const preamble_pattern[2] = {0xa9, 0xe0};

    bitbuffer_t src = cap->bits; // work on a copy of the raw bits
    unsigned bitpos = bitbuffer_search(&src, 0, 0, preamble_pattern, 12);
    if (bitpos + 156 > src.bits_per_row[0])
        return -1;
    unsigned pstart = bitpos + 11;

    // decode the original payload to obtain the 9 message bytes as a base
    bitbuffer_t packet = {0};
    unsigned endpos = bitbuffer_differential_manchester_decode(&src, 0, pstart, &packet, 80);
    if (endpos - pstart < 144 || packet.bits_per_row[0] < 72)
        return -1;
    uint8_t b[9];
    memcpy(b, packet.bb[0], 9);

    // apply edits from decoded fields (fall back to captured values)
    char const *f;
    if ((f = capture_get_field(cap, "pressure_PSI"))) {
        int pressure1 = (int)lround((atof(f) + 7.0) * 4.0);
        if (pressure1 < 0) pressure1 = 0;
        if (pressure1 > 255) pressure1 = 255;
        b[4] = (uint8_t)((b[4] & 0x80) | ((pressure1 >> 1) & 0x7f));
        b[5] = (uint8_t)(((pressure1 & 1) << 7) | (b[5] & 0x7f));
        b[7] = (uint8_t)(pressure1 ^ 0xff);
    }
    if ((f = capture_get_field(cap, "temperature_C"))) {
        int temp = (int)lround(atof(f) + 40.0);
        if (temp < 0) temp = 0;
        if (temp > 255) temp = 255;
        b[5] = (uint8_t)((b[5] & 0x80) | ((temp >> 1) & 0x7f));
        b[6] = (uint8_t)(((temp & 1) << 7) | (b[6] & 0x7f));
    }
    if ((f = capture_get_field(cap, "status"))) {
        int status = (int)strtol(f, NULL, 0);
        b[4] = (uint8_t)((status & 0x80) | (b[4] & 0x7f));
        b[6] = (uint8_t)((b[6] & 0x80) | (status & 0x7f));
    }
    if ((f = capture_get_field(cap, "id"))) {
        unsigned id = (unsigned)strtoul(f, NULL, 16);
        b[0] = (uint8_t)(id >> 24);
        b[1] = (uint8_t)(id >> 16);
        b[2] = (uint8_t)(id >> 8);
        b[3] = (uint8_t)(id);
    }

    // recompute CRC-8 (poly 0x07, init 0x80) over the 8 message bytes
    b[8] = crc8(b, 8, 0x07, 0x80);

    // re-encode 72 data bits as differential Manchester half-bits in place
    int prev = bitrow_get_bit(src.bb[0], pstart - 1);
    unsigned pos = pstart;
    for (unsigned i = 0; i < 72; ++i) {
        int d  = (b[i / 8] >> (7 - (i % 8))) & 1;
        int h1 = 1 - prev;
        int h2 = d ? h1 : (1 - h1);
        bitrow_set_bit(src.bb[0], pos++, h1);
        bitrow_set_bit(src.bb[0], pos++, h2);
        prev = h2;
    }

    *out_bits = src;

    if (!encoder_verify(decoder, out_bits, cap))
        return -1;

    return 1;
}

static char const *const output_fields[] = {
        "model",
        "type",
        "id",
        "status",
        "pressure_PSI",
        "temperature_C",
        "mic",
        NULL,
};

r_device const tpms_toyota = {
        .name        = "Toyota TPMS",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 52,  // 12-13 samples @250k
        .long_width  = 52,  // FSK
        .reset_limit = 150, // Maximum gap size before End Of Message [us].
        .decode_fn   = &tpms_toyota_callback,
        .encode_fn   = &tpms_toyota_encode,
        .fields      = output_fields,
};
