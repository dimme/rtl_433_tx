/** @file
    AVE TPMS FSK 11 byte differential Manchester encoded CRC-8 TPMS data.

    Copyright (C) 2021 Pascal Charest

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

/**
AVE TPMS FSK 11 byte differential Manchester encoded CRC-8 TPMS data.


Packet nibbles:

    PRE    IIIIIIII PP TT FF  CC

- PRE = preamble is 0xff 0xfe
- I = sensor Id in hex
- P = Pressure (4 conversion tables available)
- T = Temperature (deg C offset by 50)
- F = Flags
--    mode: 2 bits, mode 0 and 1 are 2.35kPa per pressure bit, mode 2 and 3 are 5.5kPa
--    battery: 3 bits, 7 is low, 6 not full and all other is full
--    unknown: 3 bits, last bit seems to swap from time to time
- C = CRC-8 with poly 0x31 init 0xff (alternatively, 0xd3 and 0x1e)
*/

#include "decoder.h"
#include "r_encoder.h"
#include <stdlib.h>
#include <math.h>

static int tpms_ave_decode(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos)
{
    bitbuffer_t packet_bits = {0};

    bitbuffer_differential_manchester_decode(bitbuffer, row, bitpos, &packet_bits, 160);

    if (packet_bits.bits_per_row[row] < 64) {
        return DECODE_ABORT_LENGTH; // too short to be a whole packet
    }
    decoder_log_bitbuffer(decoder, 1, __func__, &packet_bits, "");

    uint8_t *b = packet_bits.bb[row];

    if (crc8(b, 8, 0x31, 0xff) != 0) {
        return DECODE_FAIL_MIC; // bad checksum
    }

    unsigned id      = (unsigned)b[0] << 24 | b[1] << 16 | b[2] << 8 | b[3];
    int pressure_raw = b[4];
    int temperature  = b[5];
    int mode         = b[6] >> 6 & 0x3;
    int battery_raw  = b[6] >> 3 & 0x7;
    int flags        = b[6] & 0x7;
    // int crc          = b[7];

    int battery_pct = 100; // any other value, i.e. less than 6
    if (battery_raw == 6) {
        battery_pct = 75; // not full
    }
    else if (battery_raw == 7) {
        battery_pct = 25; // low
    }

    double ratio;
    double offset;
    switch (mode) {
    case 0:
        ratio  = 2.352f;
        offset = 47.0f;
        break;
    case 1:
    default:
        ratio  = 2.352f;
        offset = 0.0f;
        break;
    case 2:
        ratio  = 5.491f;
        offset = 18.2f;
        break;
    case 3:
        ratio  = 5.491f;
        offset = 0.0f;
        break;
    }
    double pressure = ((double)pressure_raw - offset) * ratio;

    char id_str[9 + 1];
    snprintf(id_str, sizeof(id_str), "%08x", id);

    /* clang-format off */
    data_t *data = data_make(
            "model",            "Model",         DATA_STRING, "AVE",
            "type",             "Type",          DATA_STRING, "TPMS",
            "id",               "Id",            DATA_STRING, id_str,
            "mode",             "Mode",          DATA_FORMAT, "M%d", DATA_INT, mode,
            "pressure_kPa",     "Pressure",      DATA_FORMAT, "%.1f kPa", DATA_DOUBLE, pressure,
            "temperature_C",    "Temperature",   DATA_FORMAT, "%.0f C", DATA_DOUBLE, (double)temperature - 50.0,
            "battery_ok",       "Battery",       DATA_INT,    battery_raw != 7, // Value 7 means "Low"
    		"battery_pct",      "Battery level", DATA_INT,    battery_pct, // Note: this might change with #3103
            "flags",            "Flags",         DATA_FORMAT, "0x%x", DATA_INT, flags,
            "mic",              "Integrity",     DATA_STRING, "CRC",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

static void tpms_ave_mode(int mode, double *ratio, double *offset)
{
    switch (mode) {
    case 0: *ratio = 2.352f; *offset = 47.0f; break;
    case 1:
    default: *ratio = 2.352f; *offset = 0.0f; break;
    case 2: *ratio = 5.491f; *offset = 18.2f; break;
    case 3: *ratio = 5.491f; *offset = 0.0f; break;
    }
}

static int tpms_ave_battery_pct(int battery_raw)
{
    return battery_raw == 7 ? 25 : battery_raw == 6 ? 75 : 100;
}

/** Encoder: inverse of tpms_ave_decode().

    Patches id, mode, pressure, temperature, battery and flags in every
    packet of row 0, recomputes the CRC-8 and re-encodes the differential
    Manchester payload in place. Battery edits are taken from battery_pct
    (25/75/100) or battery_ok (0 = low) if changed from the original.
*/
static int tpms_ave_encode(r_device const *decoder, struct capture *cap, bitbuffer_t *out_bits)
{
    uint8_t const preamble_pattern[] = {0xcc, 0xcc, 0xcc, 0xcd};

    if (cap->bits.num_rows < 1)
        return -1;
    bitbuffer_t bits = cap->bits;

    int patched = 0;
    unsigned bitpos = 0;
    while ((bitpos = bitbuffer_search(&bits, 0, bitpos, preamble_pattern, 32)) + 132 <= bits.bits_per_row[0]) {
        unsigned start     = bitpos + 32;
        bitbuffer_t packet = {0};
        bitbuffer_differential_manchester_decode(&bits, 0, start, &packet, 160);
        uint8_t *p = packet.bb[0];
        if (packet.bits_per_row[0] < 64 || crc8(p, 8, 0x31, 0xff) != 0) {
            bitpos += 31;
            continue;
        }
        uint8_t b[8];
        memcpy(b, p, 8);
        int orig_batt = b[6] >> 3 & 0x7;

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
        if (encoder_field_int(cap, "mode", &v))
            b[6] = (uint8_t)((b[6] & 0x3f) | (v & 0x3) << 6);
        int mode = b[6] >> 6 & 0x3;
        double ratio, offset;
        tpms_ave_mode(mode, &ratio, &offset);
        if (encoder_field_dbl(cap, "pressure_kPa", &d)) {
            long r = lround(d / ratio + offset);
            b[4]   = (uint8_t)(r < 0 ? 0 : r > 0xff ? 0xff : r);
        }
        if (encoder_field_dbl(cap, "temperature_C", &d)) {
            long t = lround(d) + 50;
            b[5]   = (uint8_t)(t < 0 ? 0 : t > 0xff ? 0xff : t);
        }
        int batt = orig_batt;
        if (encoder_field_int(cap, "battery_pct", &v) && v != tpms_ave_battery_pct(orig_batt))
            batt = v <= 25 ? 7 : v <= 75 ? 6 : 0;
        else if (encoder_field_int(cap, "battery_ok", &v) && v != (orig_batt != 7))
            batt = v ? 0 : 7;
        b[6] = (uint8_t)((b[6] & 0xc7) | batt << 3);
        if (encoder_field_int(cap, "flags", &v))
            b[6] = (uint8_t)((b[6] & 0xf8) | (v & 0x7));
        b[7] = crc8(b, 7, 0x31, 0xff);

        int prev = bitrow_get_bit(bits.bb[0], start - 1);
        encoder_diff_manchester_encode(bits.bb[0], start, &prev, b, 64);

        encoder_set_dbl(cap, "pressure_kPa", ((double)b[4] - offset) * ratio);
        encoder_set_dbl(cap, "temperature_C", (double)b[5] - 50.0);
        char buf[8];
        snprintf(buf, sizeof(buf), "%d", batt != 7);
        capture_set_field(cap, "battery_ok", buf);
        snprintf(buf, sizeof(buf), "%d", tpms_ave_battery_pct(batt));
        capture_set_field(cap, "battery_pct", buf);

        patched++;
        bitpos += 132 + 31;
    }
    if (!patched)
        return -1;

    *out_bits = bits;
    return encoder_verify(decoder, out_bits, cap) ? 1 : -1;
}

/**
Wrapper for the AVE tpms.
@sa tpms_ave_decode()
*/
static int tpms_ave_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    uint8_t const preamble_pattern[] = {0xcc, 0xcc, 0xcc, 0xcd}; // Raw pattern, before differential Manchester coding

    int row;
    unsigned bitpos;
    int ret    = 0;
    int events = 0;

    for (row = 0; row < bitbuffer->num_rows; ++row) {
        bitpos = 0;
        // Find a preamble with enough bits after it that it could be a complete packet
        while ((bitpos = bitbuffer_search(bitbuffer, 0, bitpos, preamble_pattern, 32)) + 132 <=
                bitbuffer->bits_per_row[0]) {
            ret = tpms_ave_decode(decoder, bitbuffer, row, bitpos + 32);
            if (ret > 0) {
                events += ret;
                bitpos += 132;
            }
            bitpos += 31;
        }
    }

    return events > 0 ? events : ret;
}

static char const *const output_fields[] = {
        "model",
        "type",
        "id",
        "battery_ok",
        "battery_pct",
        "mode",
        "pressure_kPa",
        "temperature_C",
        "flags",
        "mic",
        NULL,
};

r_device const tpms_ave = {
        .name        = "AVE TPMS",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 100,
        .long_width  = 100,
        .reset_limit = 400,
        .tolerance   = 15,
        .decode_fn   = &tpms_ave_callback,
        .encode_fn   = &tpms_ave_encode,
        .fields      = output_fields,
};
