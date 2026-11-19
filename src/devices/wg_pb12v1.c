/** @file
    WG-PB12V1 Temperature Sensor.

    Copyright (C) 2015 Tommy Vestermark
    Modifications Copyright (C) 2017 Ciarán Mooney

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/
/**
WG-PB12V1 Temperature Sensor.

Device method to decode a generic wireless temperature probe. Probe marked
with WG-PB12V1-2016/11.

Format of Packets:

The packet format appears to be similar those the Lacrosse format.
(http://fredboboss.free.fr/articles/tx29.php)

    AAAAAAAA MMMMTTTT TTTTTTTT ???IIIII HHHHHHHH CCCCCCCC

- A: Preamble - 11111111
- M: Message type?, fixed 0x3, e.g. Fine Offset WH2 has 0x4 here
- T: Temperature, scale 10, offset 40
- I: ID of probe is set randomly each time the device is powered off-on,
     Note, base station has and unused "123" symbol, but ID values can be
     higher than this.
- H: Humidity - not used, is always 11111111
- C: Checksum - CRC8, polynomial 0x31, initial value 0x0, final value 0x0

Temperature:

Temperature value is "deci-celsius", ie 10 dC = 1C, offset by -40 C.

    0010 01011101 = 605 dC => 60.5 C
    Remove offset => 60.5 C - 40 C = 20.5 C

Unknown:

Possible uses could be weak battery, or new battery.

At the moment it this device cannot distinguish between a Fine Offset
device, see fineoffset.c.
*/

#include "decoder.h"
#include "capture.h"
#include "encoder.h"
#include <math.h>

static int wg_pb12v1_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    // Validate package
    uint8_t *b = bitbuffer->bb[0];
    if (bitbuffer->bits_per_row[0] < 48)
        return DECODE_ABORT_LENGTH;
    if (b[0] != 0xFF) // Preamble
        return DECODE_ABORT_EARLY;
    if ((b[1] & 0xf0) != 0x30) // Message type, TODO: is this always a fixed value?
        return DECODE_ABORT_EARLY;
    if (b[5] != crc8(&b[1], 4, 0x31, 0)) // CRC (excluding preamble)
        return DECODE_FAIL_MIC;
    if (b[4] != 0xFF) // Humidity set to 11111111
        return DECODE_FAIL_OTHER;

    // Nibble 7,8 contains id
    int id = b[3] & 0x1F;

    // Nibble 5,6,7 contains 12 bits of temperature
    // Temperature, scaled by 10, offset by -40 C.
    int temp_raw = ((b[1] & 0x0F) << 8) | b[2];
    float temp_c = (temp_raw - 400) * 0.1f;

    /* clang-format off */
    data_t *data = data_make(
            "model",            "",             DATA_STRING, "WG-PB12V1",
            "id",               "ID",           DATA_INT,    id,
            "temperature_C",    "Temperature",  DATA_FORMAT, "%.1f C", DATA_DOUBLE, temp_c,
            "mic",              "Integrity",    DATA_STRING, "CRC",
            NULL);
    /* clang-format on */
    decoder_output_data(decoder, data);
    return 1;
}

/** Encoder: inverse of wg_pb12v1_decode(). 48-bit PWM, CRC8 over bytes 1-4. */
static int wg_pb12v1_encode(r_device const *decoder, struct capture *cap, bitbuffer_t *out_bits)
{
    uint8_t b[6];
    if (encoder_template_rows(cap, 48, b, out_bits) < 0)
        return -1;

    b[0] = 0xff; // preamble
    int v;
    double t;
    if (encoder_field_dbl(cap, "temperature_C", &t)) {
        int temp = ((int)lround(t * 10.0) + 400) & 0xfff;
        b[1]     = (uint8_t)(0x30 | ((temp >> 8) & 0x0f)); // fixed message type 0x3
        b[2]     = (uint8_t)(temp & 0xff);
    }
    else {
        b[1] = (uint8_t)(0x30 | (b[1] & 0x0f));
    }
    if (encoder_field_int(cap, "id", &v))
        b[3] = (uint8_t)((b[3] & 0xe0) | (v & 0x1f));
    b[4] = 0xff; // humidity fixed
    b[5] = crc8(&b[1], 4, 0x31, 0);

    encoder_apply_rows(out_bits, 48, b);
    return encoder_verify(decoder, out_bits, cap) ? 1 : -1;
}

static char const *const output_fields[] = {
        "model",
        "id",
        "temperature_C",
        "mic",
        NULL,
};

r_device const wg_pb12v1 = {
        .name        = "WG-PB12V1 Temperature Sensor",
        .modulation  = OOK_PULSE_PWM,
        .short_width = 564,  // Short pulse 564 us, long pulse 1476 us, fixed gap 960 us
        .long_width  = 1476, // Maximum pulse period (long pulse + fixed gap)
        .reset_limit = 2500, // We just want 1 package
        .decode_fn   = &wg_pb12v1_decode,
        .encode_fn   = &wg_pb12v1_encode,
        .fields      = output_fields,
};
