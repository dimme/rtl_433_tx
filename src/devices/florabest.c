/** @file
    Florabest FB-TH-1 BBQ Thermometer.

    Copyright (C) 2026 Benjamin Larsson

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

#include "decoder.h"
#include "capture.h"
#include "encoder.h"
#include <math.h>

/**
Florabest FB-TH-1 BBQ Thermometer (Lidl), also sold as other Florabest
grill thermometers with a wireless remote display.

Reverse engineered from captures posted in
https://github.com/merbanan/rtl_433/issues/1223.

The sensor sends 30 bits, OOK PPM modulated, repeated about 9 times with
a longer sync gap between repeats.

    .short_width = 2000 us (0 bit)
    .long_width  = 4000 us (1 bit)
    .sync gap    = 9000 us

Layout:

    II II TT TT

- I: 16 bit, observed fixed on the one unit tested (0x4909); unconfirmed
  whether this is a true per-device id or a fixed model/sync code
- T: 16 bit, top 13 bits are the temperature, bit 29 (the very last bit)
  is a parity bit

The 13-bit temperature is a raw value scaled and offset in Fahrenheit,
observed empirically (not from a datasheet), with some inherent
imprecision reported by the original analysis:

    temp_F = raw13 * 0.1 - 90

Integrity check: XOR of all 30 bits (including the parity bit itself) is
always 1 (odd parity) on every capture seen so far.
*/

static int florabest_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    // The device transmits many repeats, accept 3 matching rows.
    int row = bitbuffer_find_repeated_row(bitbuffer, 3, 30);
    if (row < 0) {
        return DECODE_ABORT_EARLY;
    }
    if (bitbuffer->bits_per_row[row] != 30) {
        return DECODE_ABORT_LENGTH;
    }

    uint8_t *b = bitbuffer->bb[row];

    if (b[0] != 0x49) {
        decoder_logf(decoder, 1, __func__, "unexpected first byte: 0x%02x", b[0]);
        return DECODE_FAIL_SANITY;
    }

    if (!b[0] && !b[1] && !b[2] && !b[3]) {
        return DECODE_ABORT_EARLY; // reduce false positives on noise
    }

    // Parity check: XOR of all 30 bits (incl. the parity bit) is always 1.
    int parity = 0;
    for (int i = 0; i < 30; ++i) {
        parity ^= bitrow_get_bit(b, i);
    }
    if (parity != 1) {
        decoder_log(decoder, 1, __func__, "parity check failed");
        return DECODE_FAIL_MIC;
    }

    int id          = (b[0] << 8) | b[1];
    int temp_raw    = (b[2] << 5) | (b[3] >> 3);
    float temp_f    = temp_raw * 0.1f - 90.0f;

    /* clang-format off */
    data_t *data = data_make(
            "model",            "",             DATA_STRING, "Florabest-FBTH1",
            "id",               "Id",           DATA_FORMAT, "%04x", DATA_INT, id,
            "temperature_F",    "Temperature",  DATA_FORMAT, "%.1f F", DATA_DOUBLE, (double)temp_f,
            "mic",              "Integrity",    DATA_STRING, "PARITY",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

/** Encoder: inverse of florabest_decode(). 30-bit, odd parity over all bits. */
static int florabest_encode(r_device const *decoder, struct capture *cap, bitbuffer_t *out_bits)
{
    uint8_t b[4];
    if (encoder_template_rows(cap, 30, b, out_bits) < 0)
        return -1;

    int id;
    double t;
    if (encoder_field_int(cap, "id", &id)) {
        b[0] = (uint8_t)((id >> 8) & 0xff);
        b[1] = (uint8_t)(id & 0xff);
    }
    if (encoder_field_dbl(cap, "temperature_F", &t)) {
        int raw = (int)lround((t + 90.0) * 10.0) & 0x1fff;
        b[2]    = (uint8_t)((raw >> 5) & 0xff);
        b[3]    = (uint8_t)((b[3] & 0x07) | ((raw & 0x1f) << 3));
    }

    // bit 29 (b[3] & 0x04) is parity: total XOR over all 30 bits must be 1
    b[3] &= (uint8_t)~0x04;
    int par = 0;
    for (int i = 0; i < 30; ++i)
        par ^= bitrow_get_bit(b, i);
    if (par != 1)
        b[3] |= 0x04;

    encoder_apply_rows(out_bits, 30, b);
    return encoder_verify(decoder, out_bits, cap) ? 1 : -1;
}

static char const *const output_fields[] = {
        "model",
        "id",
        "temperature_F",
        "mic",
        NULL,
};

r_device const florabest = {
        .name        = "Florabest FB-TH-1 BBQ Thermometer",
        .modulation  = OOK_PULSE_PPM,
        .short_width = 2000,
        .long_width  = 4000,
        .gap_limit   = 6000,
        .reset_limit = 11000,
        .decode_fn   = &florabest_decode,
        .encode_fn   = &florabest_encode,
        .fields      = output_fields,
};
