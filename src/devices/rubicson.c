/** @file
    Rubicson or InFactory PT-310 temperature sensor.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

*/

#include "decoder.h"
#include "capture.h"
#include "r_encoder.h"
#include <math.h>

/**
Rubicson temperature sensor.

Also older TFA 30.3197 sensors.
Also ZX-7382-675 sensor for inFactory FWS-325.pro, first seen 2025-01 (Pearl).
Also inFactory ZX-7614-675 (replaces ZX-7382-675).

Also InFactory PT-310 pool temperature sensor (AKA ZX-7074/7073). This device
has longer packet lengths of 37 or 38 bits but is otherwise compatible. See more at
https://github.com/merbanan/rtl_433/issues/2119

The sensor sends 12 packets of  36 bits PPM modulated data.

data is grouped into 9 nibbles

    [id0] [id1] [bat|unk1|chan1|chan2] [temp0] [temp1] [temp2] [0xf] [crc1] [crc2]

- The id changes when the battery is changed in the sensor.
- bat bit is 1 if battery is ok, 0 if battery is low
- unk1 is always 0 probably unused
- chan1 and chan2 forms a 2bit value for the used channel
- temp is 12 bit signed scaled by 10
- F is always 0xf
- crc1 and crc2 forms a 8-bit crc, polynomial 0x31, initial value 0x6c, final value 0x0

The sensor can be bought at Kjell&Co. The Infactory pool sensor can be bought at Pearl.
*/
static int rubicson_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    int r = bitbuffer_find_repeated_row(bitbuffer, 3, 36);
    if (r < 0) {
        return DECODE_ABORT_EARLY;
    }

    uint8_t *b = bitbuffer->bb[r];

    // Infactory devices report 38 (or for last repetition) 37 bits
    if (bitbuffer->bits_per_row[r] < 36 || bitbuffer->bits_per_row[r] > 38) {
        return DECODE_ABORT_LENGTH;
    }

    if ((b[3] & 0xf0) != 0xf0) {
        return DECODE_ABORT_EARLY; // const not 1111
    }

    // CRC check
    uint8_t tmp[5];
    tmp[0] = b[0];                // Byte 0 is nibble 0 and 1
    tmp[1] = b[1];                // Byte 1 is nibble 2 and 3
    tmp[2] = b[2];                // Byte 2 is nibble 4 and 5
    tmp[3] = b[3] & 0xf0;         // Byte 3 is nibble 6 and 0-padding
    tmp[4] = (b[3] & 0x0f) << 4 | // CRC is nibble 7 and 8
             (b[4] & 0xf0) >> 4;

    int chk = crc8(tmp, 5, 0x31, 0x6c);
    if (chk) {
        return DECODE_FAIL_MIC;
    }

    int id       = b[0];
    int battery  = (b[1] & 0x80);
    int channel  = ((b[1] & 0x30) >> 4) + 1;
    int temp_raw = (int16_t)((b[1] << 12) | (b[2] << 4)); // sign-extend
    float temp_c = (temp_raw >> 4) * 0.1f;

    /* clang-format off */
    data_t *data = data_make(
            "model",            "",             DATA_STRING, "Rubicson-Temperature",
            "id",               "House Code",   DATA_INT,    id,
            "channel",          "Channel",      DATA_INT,    channel,
            "battery_ok",       "Battery",      DATA_INT,    !!battery,
            "temperature_C",    "Temperature",  DATA_FORMAT, "%.1f C", DATA_DOUBLE, temp_c,
            "mic",              "Integrity",    DATA_STRING, "CRC",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

/** Encoder: inverse of rubicson_callback(), patches every captured repeat. */
static int rubicson_encode(r_device const *decoder, struct capture *cap, bitbuffer_t *out_bits)
{
    uint8_t b[5];
    if (encoder_template_rows(cap, 36, b, out_bits) < 0)
        return -1;

    int v;
    double t;
    if (encoder_field_int(cap, "id", &v))
        b[0] = (uint8_t)v;
    if (encoder_field_int(cap, "battery_ok", &v))
        b[1] = (uint8_t)((b[1] & ~0x80) | (v ? 0x80 : 0));
    if (encoder_field_int(cap, "channel", &v))
        b[1] = (uint8_t)((b[1] & ~0x30) | (((v - 1) & 0x03) << 4));
    if (encoder_field_dbl(cap, "temperature_C", &t)) {
        int temp = (int)lround(t * 10.0) & 0xfff;
        b[1]     = (uint8_t)((b[1] & 0xf0) | ((temp >> 8) & 0x0f));
        b[2]     = (uint8_t)(temp & 0xff);
    }

    // 8-bit CRC (poly 0x31, init 0x6c) over nibbles 0-6 + 0-padding, stored in
    // nibbles 7-8. Solve for the CRC byte that makes crc8(tmp,5) == 0.
    uint8_t tmp[5] = {b[0], b[1], b[2], (uint8_t)(0xf0), 0};
    for (int cand = 0; cand < 256; ++cand) {
        tmp[4] = (uint8_t)cand;
        if (crc8(tmp, 5, 0x31, 0x6c) == 0)
            break;
    }
    b[3] = (uint8_t)(0xf0 | ((tmp[4] >> 4) & 0x0f));
    b[4] = (uint8_t)((tmp[4] & 0x0f) << 4);

    encoder_apply_rows(out_bits, 36, b);
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

// timings based on samp_rate=1024000
r_device const rubicson = {
        .name        = "Rubicson, TFA 30.3197 or InFactory PT-310 Temperature Sensor",
        .modulation  = OOK_PULSE_PPM,
        .short_width = 1000, // Gaps:  Short 976 us, Long 1940 us, Sync 4000 us
        .long_width  = 2000, // Pulse: 500 us (Initial pulse in each package is 388 us)
        .gap_limit   = 3000,
        .reset_limit = 4800, // Two initial pulses and a gap of 9120 us is filtered out
        .decode_fn   = &rubicson_callback,
        .encode_fn   = &rubicson_encode,
        .fields      = output_fields,
};
