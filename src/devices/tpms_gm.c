/** @file
    General Motors Aftermarket TPMS.

    Copyright (C) 2025 Eric Blevins

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

#include "decoder.h"
#include "r_encoder.h"
#include <stdlib.h>
#include <math.h>

/**
General Motors Aftermarket TPMS.

Data was detected and initially captured using:

    rtl_433 -X 'n=name,m=OOK_MC_ZEROBIT,s=120,l=0,r=15600'


Data layout, 130 bits:
    AAAAAAAAAAAAFFFFDDDDIIIIIIPPTTCCX
    0000000000004c90007849176600536d0


- A: preamble 0x000000000000
- F: Flags
- D: Device type or prefix
- I: Device uniquie identifier
- P: Pressure
- T: Temperature
- C: CheckSum, modulo 256

Format string:

    ID:10h FLAGS:4h KPA:2h TEMP:2h CHECKSUM:2h

The only status data detected is learn mode and low battery.
Bit 5 of status indicates low battery when set to 1.
Bits 0,1,8 are set to 0 to indicate learn mode and 1 for operational mode.
The sensors drop to learn mode when detecting a large pressure drop
or when activated with the EL-50448 learning tool.

In learn mode with zero pressure they only transmit when activated by
the learning tool.
Once presurized they will transmit in learn mode and within a couple
minutes switch to sending in operatioinal mode every two minutes.

*/

static int tpms_gm_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    if (bitbuffer->num_rows != 1) {
        return DECODE_ABORT_EARLY;
    }

    if (bitbuffer->bits_per_row[0] != 130) {
        return DECODE_ABORT_LENGTH;
    }

    // Buffer for extracted bytes
    uint8_t b[17] = {0};
    bitbuffer_extract_bytes(bitbuffer, 0, 0, b, 130);

    static uint8_t const preamble_pattern[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (memcmp(b, preamble_pattern, sizeof(preamble_pattern)) != 0) {
        return DECODE_ABORT_EARLY;
    }

    // Checksum skips preamble
    uint8_t computed_checksum = 0;
    int all_zero = 1;
    for (int i = 6; i < 15; i++) {
        computed_checksum += b[i];
        all_zero &= (b[i] == 0);
    }
    if ((computed_checksum & 0xFF) != b[15]) {
        return DECODE_FAIL_MIC;
    }
    // An all-zero payload trivially passes the checksum; reject it rather
    // than reporting a bogus id-0/learn-mode/-60 C reading.
    if (all_zero && b[15] == 0) {
        return DECODE_FAIL_SANITY;
    }

    // Convert ID to an integer
    uint64_t sensor_id = ((uint64_t)b[8] << 32) | ((uint64_t)b[9] << 24) | (b[10] << 16) | (b[11] << 8) | b[12];
    int flags     = (b[6] << 8) | b[7];

    int pressure_raw    = b[13];
    int temperature_raw = b[14];

    // Adding 3.75 made my sensors accurate
    // But I think it might be best to allow the user to
    // to add their own offset when consuming the data
    float pressure_kpa  = (pressure_raw * 2.75);
    float temperature_c = temperature_raw - 60;

    // Extract bits correctly based on little-endian order
    int bit8 = (flags >> 8) & 1;
    int bit1 = (flags >> 1) & 1;
    int bit0 = (flags >> 0) & 1;

    // Flags bits
    int learn_mode = ((bit8 == 0) && (bit1 == 0) && (bit0 == 0));
    int battery_ok = !((flags >> 5) & 1);

    /* clang-format off */
    data_t *data = data_make(
        "model",           "",            DATA_STRING,  "GM-Aftermarket",
        "type",            "",            DATA_STRING,  "TPMS",
        "id",              "",            DATA_INT,     sensor_id,
        "flags",           "",            DATA_INT,     flags,
        "learn_mode",      "",            DATA_INT,     learn_mode,
        "battery_ok",      "",            DATA_INT,     battery_ok,
        "pressure_kPa",    "",            DATA_DOUBLE,  pressure_kpa,
        "temperature_C",   "",            DATA_FORMAT, "%.0f C", DATA_DOUBLE,  temperature_c,
        "mic",             "Integrity",   DATA_STRING,  "CHECKSUM",
        NULL);

    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

/** Encoder: inverse of tpms_gm_decode().

    Patches id, flags, pressure and temperature into the 130-bit row and
    recomputes the checksum. Edits to battery_ok / learn_mode (with flags left
    unchanged) are folded into the flags; both are then re-derived from flags.
*/
static int tpms_gm_encode(r_device const *decoder, struct capture *cap, bitbuffer_t *out_bits)
{
    if (cap->bits.num_rows != 1 || cap->bits.bits_per_row[0] != 130)
        return -1;

    bitbuffer_t src = cap->bits;
    uint8_t b[17] = {0};
    bitbuffer_extract_bytes(&src, 0, 0, b, 130);

    int orig_flags = (b[6] << 8) | b[7];
    int flags      = orig_flags;
    int v;
    char const *f;
    if (encoder_field_int(cap, "flags", &v))
        flags = v & 0xffff;
    if (flags == orig_flags) {
        int orig_batt  = !((orig_flags >> 5) & 1);
        int orig_learn = !(orig_flags & 0x103);
        if (encoder_field_int(cap, "battery_ok", &v) && !!v != orig_batt)
            flags = v ? (flags & ~0x20) : (flags | 0x20);
        if (encoder_field_int(cap, "learn_mode", &v) && !!v != orig_learn)
            flags = v ? (flags & ~0x103) : (flags | 0x103);
    }
    b[6] = (uint8_t)(flags >> 8);
    b[7] = (uint8_t)flags;

    if ((f = capture_get_field(cap, "id"))) {
        uint64_t id = strtoull(f, NULL, 10);
        for (int i = 0; i < 5; ++i)
            b[8 + i] = (uint8_t)(id >> (32 - 8 * i));
    }
    double d;
    if (encoder_field_dbl(cap, "pressure_kPa", &d)) {
        long p = lround(d / 2.75);
        b[13] = (uint8_t)(p < 0 ? 0 : p > 0xff ? 0xff : p);
        encoder_set_dbl(cap, "pressure_kPa", b[13] * 2.75);
    }
    if (encoder_field_dbl(cap, "temperature_C", &d)) {
        long t = lround(d) + 60;
        b[14] = (uint8_t)(t < 0 ? 0 : t > 0xff ? 0xff : t);
        encoder_set_dbl(cap, "temperature_C", b[14] - 60);
    }

    uint8_t sum = 0;
    for (int i = 6; i < 15; ++i)
        sum += b[i];
    b[15] = sum;

    char tmp[16];
    snprintf(tmp, sizeof(tmp), "%d", flags);
    capture_set_field(cap, "flags", tmp);
    snprintf(tmp, sizeof(tmp), "%d", !((flags >> 5) & 1));
    capture_set_field(cap, "battery_ok", tmp);
    snprintf(tmp, sizeof(tmp), "%d", !(flags & 0x103));
    capture_set_field(cap, "learn_mode", tmp);

    for (unsigned k = 0; k < 128; ++k)
        encoder_set_bit(src.bb[0], k, (b[k / 8] >> (7 - (k % 8))) & 1);

    *out_bits = src;

    if (!encoder_verify(decoder, out_bits, cap))
        return -1;
    return 1;
}

/** Output fields for rtl_433 */
static char const *const output_fields[] = {
        "model",
        "type",
        "id",
        "flags",
        "learn_mode",
        "battery_ok",
        "pressure_kPa",
        "temperature_C",
        "mic",
        NULL,
};

r_device const tpms_gm = {
        .name        = "GM-Aftermarket TPMS",
        .modulation  = OOK_PULSE_MANCHESTER_ZEROBIT,
        .short_width = 120,
        .long_width  = 0,
        .reset_limit = 15600,
        .decode_fn   = &tpms_gm_decode,
        .encode_fn   = &tpms_gm_encode,
        .fields      = output_fields,
};
