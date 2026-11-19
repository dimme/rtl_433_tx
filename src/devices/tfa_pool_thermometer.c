/** @file
    TFA pool temperature sensor.

    Copyright (C) 2015 Alexandre Coffignal

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
TFA pool temperature sensor.

Tested with TFA-Pool-thermometer 30.3160.

Sends 10 24 bits frames.

Data layout:

    CCCCIIII IIIITTTT TTTTTTTT DDBF

- C: checksum, sum of nibbles - 1
- I: device id (changing only after reset)
- T: temperature
- D: channel number
- B: battery status
- F: first transmission
*/

static int tfa_pool_thermometer_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    // require 7 of 10 repeats
    int row = bitbuffer_find_repeated_row(bitbuffer, 7, 28);
    if (row < 0) {
        return DECODE_ABORT_EARLY; // no repeated row found
    }
    if (bitbuffer->bits_per_row[row] != 28) {
        return DECODE_ABORT_LENGTH; // prevent false positives
    }

    uint8_t *b = bitbuffer->bb[row];

    int checksum_rx = ((b[0] & 0xF0) >> 4);
    int checksum    = ((b[0] & 0x0F)
            + (b[1] >> 4)
            + (b[1] & 0x0F)
            + (b[2] >> 4)
            + (b[2] & 0x0F)
            + (b[3] >> 4) - 1);

    if (checksum_rx != (checksum & 0x0F)) {
        decoder_logf_bitrow(decoder, 2, __func__, b, bitbuffer->bits_per_row[row], "checksum fail (%02x)", checksum);
        return DECODE_FAIL_MIC;
    }

    int device   = ((b[0] & 0x0F) << 4) | ((b[1] & 0xF0) >> 4);
    int temp_raw = ((b[1] & 0x0F) << 8) | b[2];
    float temp_f = (temp_raw > 2048 ? temp_raw - 4096 : temp_raw) * 0.1f;
    int channel  = ((b[3] & 0xC0) >> 6);
    int battery  = ((b[3] & 0x20) >> 5);

    /* clang-format off */
    data_t *data = data_make(
            "model",            "",                 DATA_STRING,    "TFA-Pool",
            "id",               "Id",               DATA_INT,       device,
            "channel",          "Channel",          DATA_INT,       channel,
            "battery_ok",       "Battery",          DATA_INT,       battery,
            "temperature_C",    "Temperature",      DATA_FORMAT,    "%.1f C",  DATA_DOUBLE,    temp_f,
            "mic",              "Integrity",        DATA_STRING,    "CHECKSUM",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

/** Encoder: inverse of tfa_pool_thermometer_decode(). */
static int tfa_pool_thermometer_encode(r_device const *decoder, struct capture *cap, bitbuffer_t *out_bits)
{
    uint8_t b[4];
    if (encoder_template_rows(cap, 28, b, out_bits) < 0)
        return -1;

    int v;
    double t;
    if (encoder_field_int(cap, "id", &v)) {
        b[0] = (uint8_t)((b[0] & 0xf0) | ((v >> 4) & 0x0f));
        b[1] = (uint8_t)((b[1] & 0x0f) | ((v & 0x0f) << 4));
    }
    if (encoder_field_dbl(cap, "temperature_C", &t)) {
        int temp = (int)lround(t * 10.0) & 0xfff;
        b[1]     = (uint8_t)((b[1] & 0xf0) | ((temp >> 8) & 0x0f));
        b[2]     = (uint8_t)(temp & 0xff);
    }
    if (encoder_field_int(cap, "channel", &v))
        b[3] = (uint8_t)((b[3] & ~0xc0) | ((v & 0x03) << 6));
    if (encoder_field_int(cap, "battery_ok", &v))
        b[3] = (uint8_t)((b[3] & ~0x20) | (v ? 0x20 : 0));

    int checksum = ((b[0] & 0x0F) + (b[1] >> 4) + (b[1] & 0x0F)
            + (b[2] >> 4) + (b[2] & 0x0F) + (b[3] >> 4) - 1) & 0x0F;
    b[0] = (uint8_t)((b[0] & 0x0f) | (checksum << 4));

    encoder_apply_rows(out_bits, 28, b);
    return encoder_verify(decoder, out_bits, cap) ? 1 : -1;
}

static char const *const output_fields[] = {
        "model",
        "id",
        "channel",
        "battery_ok",
        "temperature_C",
        "mic",
        NULL,
};

r_device const tfa_pool_thermometer = {
        .name        = "TFA pool temperature sensor",
        .modulation  = OOK_PULSE_PPM,
        .short_width = 2000,
        .long_width  = 4600,
        .gap_limit   = 7800,
        .reset_limit = 10000,
        .decode_fn   = &tfa_pool_thermometer_decode,
        .encode_fn   = &tfa_pool_thermometer_encode,
        .fields      = output_fields,
};
