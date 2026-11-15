/** @file
    LaCrosse/StarMeteo/Conrad TX35 protocol.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

*/
/**
Generic decoder for LaCrosse "IT+" (instant transmission) protocol.
Param device29or35 must be "29" or "35" depending of the device.

LaCrosse/StarMeteo/Conrad TX35DTH-IT, TFA Dostmann 30.3155     Temperature/Humidity Sensors.
LaCrosse/StarMeteo/Conrad TX29-IT, TFA Dostmann 30.3159.IT     Temperature Sensors.
Found at 868240000Hz.

LaCrosse TX25U Temperature/Temperature Probe at 915 MHz

## Protocol

Example data : https://github.com/merbanan/rtl_433_tests/tree/master/tests/lacrosse/06/gfile-tx29.cu8

       a    a    2    d    d    4    9    2    8    4    4    8    6    a    e    c
    Bits :
    1010 1010 0010 1101 1101 0100 1001 0010 1000 0100 0100 1000 0110 1010 1110 1100
    Bytes num :
    ----1---- ----2---- ----3---- ----4---- ----5---- ----6---- ----7---- ----8----
    ~~~~~~~~~ 1st byte
    preamble, sequence 10B repeated 4 times (see below)
              ~~~~~~~~~~~~~~~~~~~ bytes 2 and 3
    sync word of 0x2dd4
                                  ~~~~ 1st nibble of bytes 4
    sensor model (always 9)
                                       ~~~~ ~~ 2nd nibble of bytes 4 and 1st and 2nd bits of byte 5
    Random device id (6 bits)
                                              ~ 3rd bits of byte 5
    new battery indicator
                                               ~ 4th bits of byte 5
    unknown, unused
                                                 ~~~~ ~~~~ ~~~~ 2nd nibble of byte 5 and byte 6
    temperature, in bcd *10 +40
                                                                ~ 1st bit of byte 7
    weak battery
                                                                 ~~~ ~~~~ 2-8 bits of byte 7
    humidity, in%. If == 0x6a : no humidity sensor
                   If == 0x7d : temperature is actually second probe temperature channel
                                                                          ~~~~ ~~~~ byte 8
    crc8 (poly 0x31 init 0x00) of bytes

## Developer's comments

I have noticed that depending of the device, the message received has different length.
It seems some sensor send a long preamble (33 bits, 0 / 1 alternated), and some send only
six bits as the preamble. I own 3 sensors TX29, and two of them send a long preamble.
So this decoder synchronize on the following sequence:

    1010 1000 1011 0111 0101 0010 01--
       A    8    B    7    5    2    4

-  0 -  5 : short preamble [101010B]
-  6 - 14 : sync word [2DD4h]
- 15 - 19 : sensor model [9]

Short preamble example (sampling rate - 1Mhz):
https://github.com/merbanan/rtl_433_tests/tree/master/tests/lacrosse/06/gfile-tx29-short-preamble.cu8.

TX29 and TX35 share the same protocol, but pulse are different length, thus this decoder
handle the two signal and we use two r_device struct (only differing by the pulse width).

TX25U alternates between a temperature only packet and a packet with temperature and humidity
where a special humidity flag value of 125 indicates the second channel instead of humidity.
0x40 is added to the id to distinguish between channels.

There's no way to distinguish between the TX35 and TX25U models
*/

#include "decoder.h"
#include "capture.h"
#include "r_encoder.h"
#include <math.h>

#define LACROSSE_TX29_NOHUMIDSENSOR  0x6a // Sensor do not support humidity
#define LACROSSE_TX25_PROBE_FLAG     0x7d // Humidity flag to indicate probe temperature channel
#define LACROSSE_TX29_MODEL          29 // Model number
#define LACROSSE_TX35_MODEL          35

static int lacrosse_it(r_device *decoder, bitbuffer_t *bitbuffer, int device29or35)
{
    // 4 bits of preamble, sync word 2dd4, sensor model 9: 24 bit
    uint8_t const preamble[] = {0xa2, 0xdd, 0x49};

    int events = 0;

    for (int row = 0; row < bitbuffer->num_rows; ++row) {
        // Validate message and reject it as fast as possible : check for preamble
        unsigned int start_pos = bitbuffer_search(bitbuffer, row, 0, preamble, 24);
        // no preamble detected, move to the next row
        if (start_pos >= bitbuffer->bits_per_row[row])
            continue; // DECODE_ABORT_EARLY
        decoder_logf(decoder, 1, __func__, "LaCrosse TX29/35 detected, buffer is %d bits length, device is TX%d", bitbuffer->bits_per_row[row], device29or35);
        // remove preamble and keep only five octets
        uint8_t b[5];
        bitbuffer_extract_bytes(bitbuffer, row, start_pos + 20, b, 40);

        // Check message integrity
        int r_crc = b[4];
        int c_crc = crc8(b, 4, 0x31, 0x00);
        if (r_crc != c_crc) {
            decoder_logf(decoder, 1, __func__, "LaCrosse TX29/35 bad CRC: calculated %02x, received %02x", c_crc, r_crc);
            // reject row
            continue; // DECODE_FAIL_MIC
        }

        // message "envelope" has been validated, start parsing data
        int sensor_id   = ((b[0] & 0x0f) << 2) | (b[1] >> 6);
        float temp_c    = 10 * (b[1] & 0x0f) + 1 * ((b[2] >> 4) & 0x0f) + 0.1f * (b[2] & 0x0f) - 40.0f;
        int new_batt    = (b[1] >> 5) & 1;
        int battery_low = b[3] >> 7;
        int humidity    = b[3] & 0x7f;

        data_t *data;
        if ((humidity == LACROSSE_TX29_NOHUMIDSENSOR) || (humidity == LACROSSE_TX25_PROBE_FLAG)) {
            if (humidity == LACROSSE_TX25_PROBE_FLAG)
                sensor_id += 0x40;      // Change ID to distinguish between the main and probe channels
            /* clang-format off */
            data = data_make(
                    "model",            "",             DATA_COND,   device29or35 == 29, DATA_STRING, "LaCrosse-TX29IT",
                    "model",            "",             DATA_COND,   device29or35 != 29, DATA_STRING, "LaCrosse-TX35DTHIT",
                    "id",               "",             DATA_INT,    sensor_id,
                    "battery_ok",       "Battery",      DATA_INT,    !battery_low,
                    "newbattery",       "NewBattery",   DATA_INT,    new_batt,
                    "temperature_C",    "Temperature",  DATA_FORMAT, "%.1f C", DATA_DOUBLE, temp_c,
                    "mic",              "Integrity",    DATA_STRING, "CRC",
                    NULL);
            /* clang-format on */
        }
        else {
            /* clang-format off */
            data = data_make(
                    "model",            "",             DATA_COND,   device29or35 == 29, DATA_STRING, "LaCrosse-TX29IT",
                    "model",            "",             DATA_COND,   device29or35 != 29, DATA_STRING, "LaCrosse-TX35DTHIT",
                    "id",               "",             DATA_INT,    sensor_id,
                    "battery_ok",       "Battery",      DATA_INT,    !battery_low,
                    "newbattery",       "NewBattery",   DATA_INT,    new_batt,
                    "temperature_C",    "Temperature",  DATA_FORMAT, "%.1f C", DATA_DOUBLE, temp_c,
                    "humidity",         "Humidity",     DATA_FORMAT, "%u %%", DATA_INT, humidity,
                    "mic",              "Integrity",    DATA_STRING, "CRC",
                    NULL);
            /* clang-format on */
        }

        decoder_output_data(decoder, data);
        events++;
    }
    return events;
}

/**
Wrapper for the TX29 and TX25U device.
@sa lacrosse_it()
*/
static int lacrossetx29_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    return lacrosse_it(decoder, bitbuffer, LACROSSE_TX29_MODEL);
}

/**
Wrapper for the TX35 device.
@sa lacrosse_it()
*/
static int lacrossetx35_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    return lacrosse_it(decoder, bitbuffer, LACROSSE_TX35_MODEL);
}

/** Encoder: inverse of lacrosse_it() (shared by TX29 and TX35). FSK PCM with a
    0xA2DD49 preamble/sync/model; the 5-byte payload after it is patched in
    place (BCD temperature, +40 offset) and the CRC-8 recomputed. */
static int lacrosse_it_encode(r_device const *decoder, struct capture *cap, bitbuffer_t *out_bits)
{
    *out_bits = cap->bits;
    uint8_t const preamble[] = {0xa2, 0xdd, 0x49};

    int prow       = -1;
    unsigned start = 0;
    for (unsigned r = 0; r < out_bits->num_rows; ++r) {
        unsigned sp = bitbuffer_search(out_bits, r, 0, preamble, 24);
        if (sp < out_bits->bits_per_row[r]) {
            prow  = (int)r;
            start = sp + 20; // payload begins 20 bits into the 24-bit match
            break;
        }
    }
    if (prow < 0)
        return -1;

    uint8_t b[5];
    bitbuffer_extract_bytes(out_bits, (unsigned)prow, start, b, 40);
    int probe = ((b[3] & 0x7f) == LACROSSE_TX25_PROBE_FLAG);

    int v;
    double t;
    if (encoder_field_int(cap, "id", &v)) {
        int raw = probe ? (v - 0x40) : v;
        b[0]    = (uint8_t)(0x90 | ((raw >> 2) & 0x0f)); // model nibble 0x9 + id high
        b[1]    = (uint8_t)((b[1] & 0x3f) | ((raw & 0x03) << 6));
    }
    if (encoder_field_int(cap, "newbattery", &v))
        b[1] = (uint8_t)((b[1] & ~0x20) | (v ? 0x20 : 0));
    if (encoder_field_int(cap, "battery_ok", &v))
        b[3] = (uint8_t)((b[3] & ~0x80) | (v ? 0 : 0x80));
    if (encoder_field_dbl(cap, "temperature_C", &t)) {
        int val = (int)lround((t + 40.0) * 10.0); // total tenths with +40 C offset
        if (val < 0)
            val = 0;
        if (val > 999)
            val = 999;
        b[1] = (uint8_t)((b[1] & 0xf0) | ((val / 100) % 10));        // tens (BCD)
        b[2] = (uint8_t)((((val / 10) % 10) << 4) | (val % 10));     // ones, tenths
    }
    if (encoder_field_int(cap, "humidity", &v))
        b[3] = (uint8_t)((b[3] & 0x80) | (v & 0x7f));
    b[4] = crc8(b, 4, 0x31, 0x00);

    for (unsigned r = 0; r < out_bits->num_rows; ++r) {
        unsigned sp = bitbuffer_search(out_bits, r, 0, preamble, 24);
        if (sp >= out_bits->bits_per_row[r])
            continue;
        unsigned off = sp + 20;
        for (int i = 0; i < 40; ++i)
            encoder_set_bit(out_bits->bb[r], off + (unsigned)i, (b[i / 8] >> (7 - i % 8)) & 1);
    }

    return encoder_verify(decoder, out_bits, cap) ? 1 : -1;
}

static char const *const output_fields[] = {
        "model",
        "id",
        "battery_ok",
        "newbattery",
        "temperature_C",
        "humidity",
        "mic",
        NULL,
};

// Receiver for the TX29 and TX25U device
r_device const lacrosse_tx29 = {
        .name        = "LaCrosse TX29IT, TFA Dostmann 30.3159.IT Temperature sensor",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 55, // 58 us for TX34-IT
        .long_width  = 55, // 58 us for TX34-IT
        .reset_limit = 4000,
        .decode_fn   = &lacrossetx29_callback,
        .encode_fn   = &lacrosse_it_encode,
        .fields      = output_fields,
};

// Receiver for the TX35 device
r_device const lacrosse_tx35 = {
        .name        = "LaCrosse TX35DTH-IT, TFA Dostmann 30.3155 Temperature/Humidity sensor",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 105,
        .long_width  = 105,
        .reset_limit = 4000,
        .decode_fn   = &lacrossetx35_callback,
        .encode_fn   = &lacrosse_it_encode,
        .fields      = output_fields,
};
