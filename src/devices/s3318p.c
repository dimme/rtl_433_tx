/** @file
    Conrad Electronics S3318P outdoor sensor.

    Copyright (C) 2016 Martin Hauke
    Enhanced (C) 2019 Christian W. Zuckschwerdt <zany@triq.net>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/
/**
Largely the same as esperanza_ews, kedsum.
@sa esperanza_ews.c kedsum.c

Also NC-5849-913 from Pearl (for FWS-310 station).

Also ST389 sensor for ORIA WA50 Wireless Digital Freezer Thermometer (no humidity)

Transmit Interval: every ~50s.
Message Format: 40 bits (10 nibbles).

    Byte:      0        1        2        3        4
    Nibble:    1   2    3   4    5   6    7   8    9   10
    Type:   00 IIIIIIII ??CCTTTT TTTTTTTT HHHHHHHH WB??XXXX

- 0: Preamble
- I: sensor ID (changes on battery change)
- C: channel number
- T: temperature
- H: humidity
- W: tx-button pressed
- B: low battery
- ?: unknown meaning
- X: CRC-4 poly 0x3 init 0x0 xor last 4 bits

Example data:

    [01] {42} 04 15 66 e2 a1 00 : 00000100 00010101 01100110 11100010 10100001 00 ---> Temp/Hum/Ch:23.2/46/1

Temperature:
- Sensor sends data in °F, lowest supported value is -90°F
- 12 bit unsigned and scaled by 10 (Nibbles: 6,5,4)
- in this case `011001100101` =  1637/10 - 90 = 73.7 °F (23.17 °C)

Humidity:
- 8 bit unsigned (Nibbles 8,7)
- in this case `00101110` = 46

Channel number: (Bits 10,11) + 1
- in this case `00` --> `00` +1 = Channel1

Battery status: (Bit 33) (0 normal, 1 voltage is below ~2.7 V)
- TX-Button: (Bit 32) (0 indicates regular transmission, 1 indicates requested by pushbutton)

Random Code / Device ID: (Nibble 1)
- changes on every battery change
*/

#include "decoder.h"
#include "capture.h"
#include "encoder.h"
#include <math.h>

static int s3318p_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    uint8_t b[5];
    data_t *data;

    // ignore if two leading sync pulses (Esperanza EWS)
    if (bitbuffer->bits_per_row[0] == 0 && bitbuffer->bits_per_row[1] == 0)
        return DECODE_ABORT_EARLY;

    // the signal should have 6 repeats with a sync pulse between
    // require at least 4 received repeats
    int r = bitbuffer_find_repeated_row(bitbuffer, 4, 42);
    if (r < 0 || bitbuffer->bits_per_row[r] != 42)
        return DECODE_ABORT_LENGTH;

    // remove the two leading 0-bits and align the data
    bitbuffer_extract_bytes(bitbuffer, r, 2, b, 40);

    // No need to decode/extract values for simple test
    // check id channel temperature humidity value not zero
    if (!b[0] && !b[1] && !b[2] && !b[3]) {
        decoder_log(decoder, 2, __func__, "DECODE_FAIL_SANITY data all 0x00");
        return DECODE_FAIL_SANITY;
    }

    // CRC-4 poly 0x3, init 0x0 over 32 bits then XOR the next 4 bits
    int crc = crc4(b, 4, 0x3, 0x0) ^ (b[4] >> 4);
    if (crc != (b[4] & 0xf))
        return DECODE_FAIL_MIC;

    int id          = b[0];
    int channel     = ((b[1] & 0x30) >> 4) + 1;
    int temp_raw    = ((b[2] & 0x0f) << 8) | (b[2] & 0xf0) | (b[1] & 0x0f);
    float temp_f    = (temp_raw - 900) * 0.1f;
    int humidity    = ((b[3] & 0x0f) << 4) | ((b[3] & 0xf0) >> 4);
    int button      = b[4] >> 7;
    int battery_low = (b[4] & 0x40) >> 6;

    /* clang-format off */
    data = data_make(
            "model",            "",             DATA_STRING, "Conrad-S3318P",
            "id",               "ID",           DATA_INT,    id,
            "channel",          "Channel",      DATA_INT,    channel,
            "battery_ok",       "Battery",      DATA_INT,    !battery_low,
            "temperature_F",    "Temperature",  DATA_FORMAT, "%.2f F", DATA_DOUBLE, temp_f,
            "humidity",         "Humidity",     DATA_COND,   humidity != 0, DATA_FORMAT, "%u %%", DATA_INT, humidity,
            "button",           "Button",       DATA_INT,    button,
            "mic",              "Integrity",    DATA_STRING, "CRC",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

/** Encoder: inverse of s3318p_callback(), patches every captured repeat.
    The 40-bit payload starts at bit 2 of each 42-bit row. */
static int s3318p_encode(r_device const *decoder, struct capture *cap, bitbuffer_t *out_bits)
{
    uint8_t raw[6];
    if (encoder_template_rows(cap, 42, raw, out_bits) < 0)
        return -1;

    // extract the 40-bit payload from bit offset 2
    uint8_t b[5] = {0};
    for (int i = 0; i < 40; ++i) {
        int src = i + 2;
        if ((raw[src / 8] >> (7 - src % 8)) & 1)
            b[i / 8] |= (uint8_t)(0x80 >> (i % 8));
    }

    int v;
    double t;
    if (encoder_field_int(cap, "id", &v))
        b[0] = (uint8_t)v;
    if (encoder_field_int(cap, "channel", &v))
        b[1] = (uint8_t)((b[1] & ~0x30) | (((v - 1) & 0x03) << 4));
    if (encoder_field_dbl(cap, "temperature_F", &t)) {
        int temp_raw = ((int)lround(t * 10.0) + 900) & 0xfff;
        b[1] = (uint8_t)((b[1] & 0xf0) | (temp_raw & 0x0f));
        b[2] = (uint8_t)((temp_raw & 0xf0) | ((temp_raw >> 8) & 0x0f));
    }
    if (encoder_field_int(cap, "humidity", &v))
        b[3] = (uint8_t)(((v & 0x0f) << 4) | ((v >> 4) & 0x0f));
    if (encoder_field_int(cap, "button", &v))
        b[4] = (uint8_t)((b[4] & ~0x80) | (v ? 0x80 : 0));
    if (encoder_field_int(cap, "battery_ok", &v))
        b[4] = (uint8_t)((b[4] & ~0x40) | (v ? 0 : 0x40)); // B: 1=low

    // CRC-4 poly 0x3 init 0x0 over the first 32 bits, XORed with the high nibble of b[4]
    int crc = crc4(b, 4, 0x3, 0x0) ^ (b[4] >> 4);
    b[4]    = (uint8_t)((b[4] & 0xf0) | (crc & 0x0f));

    // write the payload back at bit offset 2 of the 42-bit frame
    for (int i = 0; i < 40; ++i) {
        int bit = (b[i / 8] >> (7 - i % 8)) & 1;
        encoder_set_bit(raw, (unsigned)(i + 2), bit);
    }

    // Emit a clean buffer of just the 42-bit payload rows. The captured bit
    // train has leading empty rows that the decoder's Esperanza-EWS guard
    // (rows[0]==0 && rows[1]==0) rejects, so we cannot patch it in place.
    unsigned reps = 0;
    for (unsigned i = 0; i < out_bits->num_rows; ++i)
        if (out_bits->bits_per_row[i] >= 41 && out_bits->bits_per_row[i] <= 44)
            ++reps;
    if (reps < 6)
        reps = 6;
    bitbuffer_clear(out_bits);
    for (unsigned r = 0; r < reps; ++r) {
        if (r > 0)
            bitbuffer_add_row(out_bits); // first row is created by add_bit
        for (int i = 0; i < 42; ++i)
            bitbuffer_add_bit(out_bits, (raw[i / 8] >> (7 - i % 8)) & 1);
    }

    return encoder_verify(decoder, out_bits, cap) ? 1 : -1;
}

static char const *const output_fields[] = {
        "model",
        "id",
        "channel",
        "battery_ok",
        "button",
        "temperature_F",
        "humidity",
        "mic",
        NULL,
};

r_device const s3318p = {
        .name        = "Conrad S3318P, FreeTec NC-5849-913 temperature humidity sensor, ORIA WA50 ST389 temperature sensor",
        .modulation  = OOK_PULSE_PPM,
        .short_width = 1900,
        .long_width  = 3800,
        .gap_limit   = 4400,
        .reset_limit = 9400,
        .decode_fn   = &s3318p_callback,
        .encode_fn   = &s3318p_encode,
        .fields      = output_fields,
};
