/** @file
    Jansite FSK 11 byte Manchester encoded checksummed TPMS data.

    Copyright (C) 2021 Benjamin Larsson

    based on code
    2019 Andreas Spiess and Christian W. Zuckschwerdt <zany@triq.net>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

/**
Jansite Solar TPMS Solar Model.

http://www.jansite.cn/P_view.asp?pid=229

- Frequency: 433.92 +/- 20.00 MHz
- Pressure: +/- 0.1 bar from 0 bar to 6.6 bar
- Temperature: +/- 3 C from -40 C to 75 C

Signal is manchester encoded, and a 11 byte large message

Data layout (nibbles):

    SS SS II II II 00 TT PP 00 CC CC

- S: 16 bits sync word, 0xdd33
- I: 24 bits ID
- 0: 8 bits Unknown data 1
- T: 8 bit Temperature (deg. C offset by 55)
- P: 8 bit Pressure
- 0: 8 bits Unknown data 2
- C: 16 bit CRC (CRC-16/BUYPASS)
- The preamble is 0xa6, 0xa6, 0x5a

TODO: identify battery bits
*/

#include "decoder.h"
#include "r_encoder.h"
#include <stdlib.h>
#include <math.h>

static int tpms_jansite_solar_decode(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos)
{
    bitbuffer_t packet_bits = {0};
    uint8_t *b;
    unsigned id;
    int flags;
    int pressure;
    int temperature;

    bitbuffer_manchester_decode(bitbuffer, row, bitpos, &packet_bits, 88);
    bitbuffer_invert(&packet_bits);

    if (packet_bits.bits_per_row[0] < 88) {
        return DECODE_FAIL_SANITY;
    }
    b = packet_bits.bb[0];

    /* Check for sync */
    if ((b[0] << 8 | b[1]) != 0xdd33) {
        return DECODE_FAIL_SANITY;
    }

    /* Check crc */
    uint16_t crc_calc = crc16(&b[2], 7, 0x8005, 0x0000);
    if (((b[9] << 8) | b[10]) != crc_calc) {
        decoder_logf(decoder, 1, __func__, "CRC mismatch %04x vs %02x %02x", crc_calc, b[9], b[10]);
        return DECODE_FAIL_MIC;
    }

    id          = (unsigned)b[2] << 16 | b[3] << 8 | b[4];
    flags       = b[5];
    temperature = b[6];
    pressure    = b[7];

    char id_str[7 + 1];
    snprintf(id_str, sizeof(id_str), "%06x", id);
    char code_str[9 * 2 + 1];
    snprintf(code_str, sizeof(code_str), "%02x%02x%02x%02x%02x%02x%02x%02x%02x", b[2], b[3], b[4], b[5], b[6], b[7], b[8], b[9], b[10]);

    /* clang-format off */
    data_t *data = data_make(
            "model",            "",             DATA_STRING, "Jansite-Solar",
            "type",             "",             DATA_STRING, "TPMS",
            "id",               "",             DATA_STRING, id_str,
            "flags",            "",             DATA_INT, flags,
            "pressure_kPa",     "Pressure",     DATA_FORMAT, "%.0f kPa", DATA_DOUBLE, (float)pressure * 1.6,
            "temperature_C",    "Temperature",  DATA_FORMAT, "%.0f C", DATA_DOUBLE, (float)temperature - 55.0,
            "code",             "",             DATA_STRING, code_str,
            "mic",              "Integrity",    DATA_STRING, "CRC",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

/** @sa tpms_jansite_solar_decode() */
static int tpms_jansite_solar_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    uint8_t const preamble_pattern[3] = {0xa6, 0xa6, 0x5a};

    unsigned bitpos = 0;
    int ret         = 0;
    int events      = 0;

    while ((bitpos = bitbuffer_search(bitbuffer, 0, bitpos, preamble_pattern, 24)) + 80 <=
            bitbuffer->bits_per_row[0]) {

        ret = tpms_jansite_solar_decode(decoder, bitbuffer, 0, bitpos);
        if (ret > 0)
            events += ret;
        bitpos += 2;
    }

    return events > 0 ? events : ret;
}

/** Encoder: inverse of tpms_jansite_solar_decode().

    Patches id, flags, temperature and pressure in every packet of row 0,
    recomputes the CRC-16 and re-encodes the (inverted) Manchester payload.
*/
static int tpms_jansite_solar_encode(r_device const *decoder, struct capture *cap, bitbuffer_t *out_bits)
{
    uint8_t const preamble_pattern[3] = {0xa6, 0xa6, 0x5a};

    if (cap->bits.num_rows < 1)
        return -1;
    bitbuffer_t bits = cap->bits;

    char const *f;
    int v;
    double d;
    int have_id = 0, have_flags = 0, have_t = 0, have_p = 0;
    unsigned id = 0;
    uint8_t flags = 0, temp = 0, press = 0;
    if ((f = capture_get_field(cap, "id"))) {
        id      = (unsigned)strtoul(f, NULL, 16) & 0xffffff;
        have_id = 1;
    }
    if (encoder_field_int(cap, "flags", &v)) {
        flags      = (uint8_t)v;
        have_flags = 1;
    }
    if (encoder_field_dbl(cap, "temperature_C", &d)) {
        long t = lround(d) + 55;
        temp   = (uint8_t)(t < 0 ? 0 : t > 0xff ? 0xff : t);
        have_t = 1;
        encoder_set_dbl(cap, "temperature_C", temp - 55.0);
    }
    if (encoder_field_dbl(cap, "pressure_kPa", &d)) {
        long p = lround(d / 1.6);
        press  = (uint8_t)(p < 0 ? 0 : p > 0xff ? 0xff : p);
        have_p = 1;
        encoder_set_dbl(cap, "pressure_kPa", (float)press * 1.6);
    }

    int patched = 0;
    uint8_t b[11];
    unsigned bitpos = 0;
    while ((bitpos = bitbuffer_search(&bits, 0, bitpos, preamble_pattern, 24)) + 176 <= bits.bits_per_row[0]) {
        bitbuffer_t packet = {0};
        bitbuffer_manchester_decode(&bits, 0, bitpos, &packet, 88);
        bitbuffer_invert(&packet);
        uint8_t *p = packet.bb[0];
        if (packet.bits_per_row[0] >= 88 && (p[0] << 8 | p[1]) == 0xdd33) {
            memcpy(b, p, 11);
            if (have_id) {
                b[2] = (uint8_t)(id >> 16);
                b[3] = (uint8_t)(id >> 8);
                b[4] = (uint8_t)id;
            }
            if (have_flags)
                b[5] = flags;
            if (have_t)
                b[6] = temp;
            if (have_p)
                b[7] = press;
            uint16_t crc = crc16(&b[2], 7, 0x8005, 0x0000);
            b[9]         = (uint8_t)(crc >> 8);
            b[10]        = (uint8_t)crc;
            encoder_manchester_encode(bits.bb[0], bitpos, b, 88, 1);
            patched++;
            bitpos += 176;
        }
        else {
            bitpos += 2;
        }
    }
    if (!patched)
        return -1;

    char code_str[9 * 2 + 1];
    snprintf(code_str, sizeof(code_str), "%02x%02x%02x%02x%02x%02x%02x%02x%02x", b[2], b[3], b[4], b[5], b[6], b[7], b[8], b[9], b[10]);
    capture_set_field(cap, "code", code_str);

    *out_bits = bits;
    return encoder_verify(decoder, out_bits, cap) ? 1 : -1;
}

static char const *const output_fields[] = {
        "model",
        "type",
        "id",
        "flags",
        "pressure_kPa",
        "temperature_C",
        "code",
        "mic",
        NULL,
};

r_device const tpms_jansite_solar = {
        .name        = "Jansite TPMS Model Solar",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 51,
        .long_width  = 51,
        .reset_limit = 5000, // Large enough to merge the 3 duplicate messages
        .decode_fn   = &tpms_jansite_solar_callback,
        .encode_fn   = &tpms_jansite_solar_encode,
        .fields      = output_fields,
};
