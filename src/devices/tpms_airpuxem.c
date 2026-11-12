/** @file
    Airpuxem TYH11_EU6_ZQ FSK 84 bits Manchester encoded TPMS data.

    Copyright (C) 2019 Alexander Grau, Bruno Octau, Christian W. Zuckschwerdt <zany@triq.net>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

/**
Airpuxem TPMS Model TYH11_EU6_ZQ FSK.
- Working Temperature:-40 °C to 125 °C
- Working Frequency: 433.92MHz+-30KHz
- Tire monitoring range value: 100kPa-900kPa+-7kPa
- Based on SENASIC SNP739D TPMS IC ( https://www.senasic.com/Public/Uploads/uploadfile2/files/20240206/DS0069SNP739D0XDatasheet.pdf )
- Probably a 'white-labeled' Jansite TPMS ( http://www.jansite.cn/P_view.asp?pid=232 )

Data layout (nibbles):

    F  II II II II   M N  PP  TT  BB  CC  CC

- F: 4 bit Sync (5)
- I: 32 bit ID
- M: 1 bit Pressure MSB_ONE, 3 bit Flags
- N: 1 bit Pressure MSB TWO, 3 bit Sensor position
- P: 8 bit Pressure LSB (kPa)
- T: 8 bit Temperature (deg. C)
- B: 8 bit Battery level  (a good guess)
- C: 8 bit Checksum
- The preamble is 0xaa..aa9 (or 0x55..556 depending on polarity)
*/

#include "decoder.h"
#include "r_encoder.h"
#include <stdlib.h>
#include <math.h>

static int tpms_airpuxem_decode(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos)
{
    bitbuffer_t dec = {0};
    uint8_t *b;

    // Decode up to ~200 bits of Manchester into a temp buffer
    bitbuffer_manchester_decode(bitbuffer, row, bitpos, &dec, 354);
    if (dec.bits_per_row[0] < 84) {  // need at least 4 ("FIVE") + 64 (CRC'ed data) + 8 (CRC) + 8 (CRC again) = 84 bits
        return DECODE_FAIL_SANITY;
    }

    b = dec.bb[0];

    unsigned nbits  = dec.bits_per_row[0];

    // Basic structural checks
    if ((b[0] >> 4) != 0x5) {
        return DECODE_FAIL_SANITY; // header nibble mismatch
    }

    // Compute CRC over 84 bits starting right after the 4-bit constant header (FIVE)
    uint8_t payload[16] = {0};
    bitbuffer_extract_bytes(&dec, 0, 4, payload, 64);
    uint8_t crc_calc = crc8(payload, 8, 0x2f, 0xaa);

    // Extract two CRC bytes following the 4+64-bit payload
    uint8_t crcs[2] = {0};
    bitbuffer_extract_bytes(&dec, 0, 4 + 64, crcs, 16);
    if (crcs[0] != crc_calc) {
        decoder_logf(decoder, 2, __func__, "CRC mismatch calc=%02x exp0=%02x exp1=%02x len=%u", crc_calc, crcs[0], crcs[1], nbits);
        return DECODE_FAIL_MIC;
    }

    // Extract bitstream starting at bit offset 4
    uint8_t id_bytes[10] = {0};
    bitbuffer_extract_bytes(&dec, 0, 4, id_bytes, 10 * 8);
    unsigned id = ((unsigned)id_bytes[0] << 24) | (id_bytes[1] << 16) | (id_bytes[2] << 8) | id_bytes[3];


    char id_str[8 + 1];
    snprintf(id_str, sizeof(id_str), "%08x", id);

    int flags       = (id_bytes[4] >> 4) & 0x07;
    int position    = id_bytes[4] & 0x07;
    int pressure    = (id_bytes[5] | (((id_bytes[4] >> 7) & 1) << 8) | (((id_bytes[4] >> 3) & 1) << 9)) - 100;
    int temperature  = (char) id_bytes[6];
    int battery      =  id_bytes[7];

    char code_str[11 * 2 + 1];
    bitrow_snprint(b, 11 * 8, code_str, sizeof(code_str));

    /* clang-format off */
    data_t *data = data_make(
            "model",            "",             DATA_STRING, "Airpuxem-TYH11EU6ZQ",
            "type",             "",             DATA_STRING, "TPMS",
            "id",               "",             DATA_STRING, id_str,
            "position",         "",             DATA_INT, position,
            "flags",            "",             DATA_INT, flags,
            "pressure_kPa",     "Pressure",     DATA_FORMAT, "%.0f kPa", DATA_DOUBLE, (double)pressure ,
            "temperature_C",    "Temperature",  DATA_FORMAT, "%.0f C", DATA_DOUBLE, (double)temperature ,
            "battery_V",        "Battery",      DATA_FORMAT, "%.1f V", DATA_DOUBLE, (double)battery * 0.02,
            "code",             "",             DATA_STRING, code_str,
            "mic",              "Integrity",    DATA_STRING, "CRC",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

/** @sa tpms_airpuxem_decode() */
static int tpms_airpuxem_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    // full preamble is (hex)
    // 5555555555555555555555555555555555555555555556

    // invert, search preamble on each row, then decode after it
    uint8_t const preamble_pattern[3] = {0xaa, 0xaa, 0xa9}; // after invert

    int ret    = 0;
    int events = 0;

    bitbuffer_invert(bitbuffer);

    for (int row = 0; row < bitbuffer->num_rows; ++row) {
        unsigned bitpos = 0;
        while ((bitpos = bitbuffer_search(bitbuffer, row, bitpos, preamble_pattern, 24)) + 80 <=
                bitbuffer->bits_per_row[row]) {
                ret = tpms_airpuxem_decode(decoder, bitbuffer, row, bitpos + 24);
            if (ret > 0)
                events += ret;
            bitpos += 2;
        }
    }

    return events > 0 ? events : ret;
}

/** Encoder: inverse of tpms_airpuxem_decode().

    Patches id, position, flags, pressure, temperature and battery in every
    packet, recomputes the CRC-8 (the second, unverified CRC byte is kept)
    and re-encodes the first 84 Manchester bits.
*/
static int tpms_airpuxem_encode(r_device const *decoder, struct capture *cap, bitbuffer_t *out_bits)
{
    uint8_t const preamble_pattern[3] = {0xaa, 0xaa, 0xa9};

    bitbuffer_t bits = cap->bits;
    bitbuffer_invert(&bits);

    char const *f;
    int v;
    double d;
    int have_id = 0, have_pos = 0, have_flags = 0, have_p = 0, have_t = 0, have_b = 0;
    unsigned id = 0, press = 0;
    int pos = 0, flags = 0;
    uint8_t temp = 0, batt = 0;
    if ((f = capture_get_field(cap, "id"))) {
        id      = (unsigned)strtoul(f, NULL, 16);
        have_id = 1;
    }
    if (encoder_field_int(cap, "position", &v)) {
        pos      = v & 0x07;
        have_pos = 1;
    }
    if (encoder_field_int(cap, "flags", &v)) {
        flags      = v & 0x07;
        have_flags = 1;
    }
    if (encoder_field_dbl(cap, "pressure_kPa", &d)) {
        long p = lround(d) + 100;
        press  = (unsigned)(p < 0 ? 0 : p > 0x3ff ? 0x3ff : p);
        have_p = 1;
        encoder_set_dbl(cap, "pressure_kPa", (double)press - 100);
    }
    if (encoder_field_dbl(cap, "temperature_C", &d)) {
        long t = lround(d);
        temp   = (uint8_t)(int8_t)(t < -128 ? -128 : t > 127 ? 127 : t);
        have_t = 1;
        encoder_set_dbl(cap, "temperature_C", (int8_t)temp);
    }
    if (encoder_field_dbl(cap, "battery_V", &d)) {
        long bv = lround(d / 0.02);
        batt    = (uint8_t)(bv < 0 ? 0 : bv > 0xff ? 0xff : bv);
        have_b  = 1;
        encoder_set_dbl(cap, "battery_V", batt * 0.02);
    }

    int patched = 0;
    char code_str[11 * 2 + 1] = {0};
    for (int row = 0; row < bits.num_rows; ++row) {
        unsigned bitpos = 0;
        while ((bitpos = bitbuffer_search(&bits, row, bitpos, preamble_pattern, 24)) + 80 <= bits.bits_per_row[row]) {
            bitbuffer_t dec = {0};
            bitbuffer_manchester_decode(&bits, row, bitpos + 24, &dec, 354);
            uint8_t pl[8], crcs[2];
            if (dec.bits_per_row[0] < 84 || (dec.bb[0][0] >> 4) != 0x5) {
                bitpos += 2;
                continue;
            }
            bitbuffer_extract_bytes(&dec, 0, 4, pl, 64);
            bitbuffer_extract_bytes(&dec, 0, 68, crcs, 16);
            if (crcs[0] != crc8(pl, 8, 0x2f, 0xaa)) {
                bitpos += 2;
                continue;
            }
            if (have_id) {
                pl[0] = (uint8_t)(id >> 24);
                pl[1] = (uint8_t)(id >> 16);
                pl[2] = (uint8_t)(id >> 8);
                pl[3] = (uint8_t)id;
            }
            if (have_flags)
                pl[4] = (uint8_t)((pl[4] & 0x8f) | flags << 4);
            if (have_pos)
                pl[4] = (uint8_t)((pl[4] & 0xf8) | pos);
            if (have_p) {
                pl[4] = (uint8_t)((pl[4] & 0x77) | ((press >> 8) & 1) << 7 | ((press >> 9) & 1) << 3);
                pl[5] = (uint8_t)press;
            }
            if (have_t)
                pl[6] = temp;
            if (have_b)
                pl[7] = batt;
            crcs[0] = crc8(pl, 8, 0x2f, 0xaa);

            // 4-bit header, 64-bit payload, 2 CRC bytes = 84 bits
            uint8_t msg[11];
            msg[0] = (uint8_t)(0x50 | pl[0] >> 4);
            for (int i = 1; i < 8; i++)
                msg[i] = (uint8_t)(pl[i - 1] << 4 | pl[i] >> 4);
            msg[8]  = (uint8_t)(pl[7] << 4 | crcs[0] >> 4);
            msg[9]  = (uint8_t)(crcs[0] << 4 | crcs[1] >> 4);
            msg[10] = (uint8_t)(crcs[1] << 4);
            encoder_manchester_encode(bits.bb[row], bitpos + 24, msg, 84, 0);

            bitbuffer_t chk = {0};
            bitbuffer_manchester_decode(&bits, row, bitpos + 24, &chk, 354);
            bitrow_snprint(chk.bb[0], 11 * 8, code_str, sizeof(code_str));
            patched++;
            bitpos += 24 + 168;
        }
    }
    if (!patched)
        return -1;
    capture_set_field(cap, "code", code_str);

    bitbuffer_invert(&bits);
    *out_bits = bits;
    return encoder_verify(decoder, out_bits, cap) ? 1 : -1;
}

static char const *const output_fields[] = {
        "model",
        "type",
        "id",
        "position",
        "flags",
        "pressure_kPa",
        "temperature_C",
        "battery_V",
        "code",
        "mic",
        NULL,
};

r_device const tpms_airpuxem = {
        .name        = "Airpuxem TPMS TYH11_EU6_ZQ",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 52,
        .long_width  = 52,
        .reset_limit = 150,
        .decode_fn   = &tpms_airpuxem_callback,
        .encode_fn   = &tpms_airpuxem_encode,
        .fields      = output_fields,
};
