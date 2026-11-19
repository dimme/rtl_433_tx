/** @file
    AlectoV1 Weather Sensor protocol.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

*/
/** @fn int alectov1_callback(r_device *decoder, bitbuffer_t *bitbuffer)
AlectoV1 Weather Sensor decoder.
Documentation also at http://www.tfd.hu/tfdhu/files/wsprotocol/auriol_protocol_v20.pdf

Also Unitec W186-F (bought from Migros).

PPM with pulse width 500 us, long gap 4000 us, short gap 2000 us, sync gap 9000 us.

Some sensors transmit 8 long pulses (1-bits) as first row.
Some sensors transmit 3 lone pulses (sync bits) between packets.

Message Format: (9 nibbles, 36 bits):
Please note that bytes need to be reversed before processing!

Format for Temperature Humidity:

    IIIICCII BMMP TTTT TTTT TTTT HHHHHHHH CCCC
    RC       Type Temperature___ Humidity Checksum

- I: 8 bit Random Device ID, includes 2 bit channel (X, 1, 2, 3)
- B: 1 bit Battery status (0 normal, 1 voltage is below ~2.6 V)
- M: 2 bit Message type, Temp/Humidity if not '11' else wind/rain sensor
- P: 1 bit a 0 indicates regular transmission, 1 indicates requested by pushbutton
- T: 12 bit Temperature (two's complement)
- H: 8 bit Humidity BCD format
- C: 4 bit Checksum

Format for Rain:

    IIIIIIII BMMP 1100 RRRR RRRR RRRR RRRR CCCC
    RC       Type      Rain                Checksum

- I: 8 bit Random Device ID, includes 2 bit channel (X, 1, 2, 3)
- B: 1 bit Battery status (0 normal, 1 voltage is below ~2.6 V)
- M: 2 bit Message type, Temp/Humidity if not '11' else wind/rain sensor
- P: 1 bit a 0 indicates regular transmission, 1 indicates requested by pushbutton
- R: 16 bit Rain (bitvalue * 0.25 mm)
- C: 4 bit Checksum

Format for Windspeed:

    IIIIIIII BMMP 1000 0000 0000 WWWWWWWW CCCC
    RC       Type                Windspd  Checksum

- I: 8 bit Random Device ID, includes 2 bit channel (X, 1, 2, 3)
- B: 1 bit Battery status (0 normal, 1 voltage is below ~2.6 V)
- M: 2 bit Message type, Temp/Humidity if not '11' else wind/rain sensor
- P: 1 bit a 0 indicates regular transmission, 1 indicates requested by pushbutton
- W: 8 bit Windspeed  (bitvalue * 0.2 m/s, correction for webapp = 3600/1000 * 0.2 * 100 = 72)
- C: 4 bit Checksum


Format for Winddirection & Windgust:

    IIIIIIII BMMP 111D DDDD DDDD GGGGGGGG CCCC
    RC       Type      Winddir   Windgust Checksum

- I: 8 bit Random Device ID, includes 2 bit channel (X, 1, 2, 3)
- B: 1 bit Battery status (0 normal, 1 voltage is below ~2.6 V)
- M: 2 bit Message type, Temp/Humidity if not '11' else wind/rain sensor
- P: 1 bit a 0 indicates regular transmission, 1 indicates requested by pushbutton
- D: 9 bit Wind direction
- G: 8 bit Windgust (bitvalue * 0.2 m/s, correction for webapp = 3600/1000 * 0.2 * 100 = 72)
- C: 4 bit Checksum
*/

#include "decoder.h"
#include "capture.h"
#include "encoder.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

// return 1 if the checksum passes and 0 if it fails
static int alecto_checksum(uint8_t *b)
{
    int csum = 0;
    for (int i = 0; i < 4; i++) {
        uint8_t tmp = reverse8(b[i]);
        csum += (tmp & 0xf) + ((tmp & 0xf0) >> 4);
    }

    csum = ((b[1] & 0x7f) == 0x6c) ? (csum + 0x7) : (0xf - csum);
    csum = reverse8((csum & 0xf) << 4);

    // Test the checksum
    return (csum == (b[4] >> 4));
}

static uint8_t bcd_decode8(uint8_t x)
{
    return ((x & 0xF0) >> 4) * 10 + (x & 0x0F);
}

static int alectov1_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    bitrow_t *bb = bitbuffer->bb;
    uint8_t *b = bitbuffer->bb[1];
    int temp_raw, humidity;
    float temp_c;
    data_t *data;
    unsigned bits = bitbuffer->bits_per_row[1];

    if (bits != 36)
        return DECODE_ABORT_LENGTH;

    if (bb[1][0] != bb[5][0] || bb[2][0] != bb[6][0]
            || (bb[1][4] & 0xf) != 0 || (bb[5][4] & 0xf) != 0
            || bb[5][0] == 0 || bb[5][1] == 0)
        return DECODE_ABORT_EARLY;

    if (!alecto_checksum(bb[1]) || !alecto_checksum(bb[5])) {
        decoder_log(decoder, 1, __func__, "AlectoV1 Checksum/Parity error");
        return DECODE_FAIL_MIC;
    }

    int battery_low = (b[1] & 0x80) >> 7;
    int msg_type    = (b[1] & 0x60) >> 5;
    //int button      = (b[1] & 0x10) >> 4;
    int msg_rain    = (b[1] & 0x0f) == 0x0c;
    //int msg_wind    = (b[1] & 0x0f) == 0x08 && b[2] == 0;
    //int msg_gust    = (b[1] & 0x0e) == 0x0e;
    int channel     = (b[0] & 0xc) >> 2;
    int sensor_id   = reverse8(b[0]);

    //decoder_logf(decoder, 0, __func__, "AlectoV1 type : %d rain : %d wind : %d gust : %d", msg_type, msg_rain, msg_wind, msg_gust);

    if (msg_type == 0x3 && !msg_rain) {
        // Wind sensor
        int skip = -1;
        // Untested code written according to the specification, may not decode correctly
        if ((b[1] & 0xe) == 0x8 && b[2] == 0) {
            skip = 0;
        }
        else if ((b[1] & 0xe) == 0xe) {
            skip = 4;
        } // According to supplied data!
        if (skip >= 0) {
            double speed  = reverse8(bb[1 + skip][3]);
            double gust   = reverse8(bb[5 + skip][3]);
            int direction = (reverse8(bb[5 + skip][2]) << 1) | (bb[5 + skip][1] & 0x1);

            /* clang-format off */
            data = data_make(
                    "model",            "",                 DATA_STRING, "AlectoV1-Wind",
                    "id",               "House Code",       DATA_INT,    sensor_id,
                    "channel",          "Channel",          DATA_INT,    channel,
                    "battery_ok",       "Battery",          DATA_INT,    !battery_low,
                    "wind_avg_m_s",     "Wind speed",       DATA_FORMAT, "%.2f m/s", DATA_DOUBLE, speed * 0.2F,
                    "wind_max_m_s",     "Wind gust",        DATA_FORMAT, "%.2f m/s", DATA_DOUBLE, gust * 0.2F,
                    "wind_dir_deg",     "Wind Direction",   DATA_INT,    direction,
                    "mic",              "Integrity",        DATA_STRING, "CHECKSUM",
                    NULL);
            /* clang-format on */
            decoder_output_data(decoder, data);
            return 1;
        }
    }

    else if (msg_type == 0x3 && msg_rain) {
        // Rain sensor
        double rain_mm = ((reverse8(b[3]) << 8) | reverse8(b[2])) * 0.25F;

        /* clang-format off */
        data = data_make(
                "model",        "",             DATA_STRING, "AlectoV1-Rain",
                "id",           "House Code",   DATA_INT,    sensor_id,
                "channel",      "Channel",      DATA_INT,    channel,
                "battery_ok",   "Battery",      DATA_INT,    !battery_low,
                "rain_mm",      "Total Rain",   DATA_FORMAT, "%.2f mm", DATA_DOUBLE, rain_mm,
                "mic",          "Integrity",    DATA_STRING, "CHECKSUM",
                NULL);
        /* clang-format on */
        decoder_output_data(decoder, data);
        return 1;
    }

    else if (msg_type != 0x3
            && bb[2][0] == bb[3][0] && bb[3][0] == bb[4][0]
            && bb[4][0] == bb[5][0] && bb[5][0] == bb[6][0]
            && (bb[3][4] & 0xf) == 0 && (bb[5][4] & 0xf) == 0) {
        //static char const *const temp_states[4] = {"stable", "increasing", "decreasing", "invalid"};
        temp_raw = (int16_t)((reverse8(b[1]) & 0xf0) | (reverse8(b[2]) << 8)); // sign-extend
        temp_c   = (temp_raw >> 4) * 0.1f;
        humidity = bcd_decode8(reverse8(b[3]));
        if (humidity > 100)
            return DECODE_FAIL_SANITY; // detect false positive, prologue is also 36bits and sometimes detected as alecto

        /* clang-format off */
        data = data_make(
                "model",         "",            DATA_STRING, "AlectoV1-Temperature",
                "id",            "House Code",  DATA_INT,    sensor_id,
                "channel",       "Channel",     DATA_INT,    channel,
                "battery_ok",    "Battery",     DATA_INT,    !battery_low,
                "temperature_C", "Temperature", DATA_FORMAT, "%.2f C", DATA_DOUBLE, temp_c,
                "humidity",      "Humidity",    DATA_FORMAT, "%u %%",   DATA_INT, humidity,
                "mic",           "Integrity",   DATA_STRING, "CHECKSUM",
                NULL);
        /* clang-format on */
        decoder_output_data(decoder, data);
        return 1;
    }

    return DECODE_FAIL_SANITY;
}

/** Encoder: inverse of alectov1_callback() for the temperature/humidity type.
    Bytes are bit-reversed; emit identical 36-bit rows so the decoder's row
    1/2/5/6 cross-checks hold. The checksum nibble is solved with the existing
    alecto_checksum(). Rain/wind types are left to raw replay. */
static int alectov1_encode(r_device const *decoder, struct capture *cap, bitbuffer_t *out_bits)
{
    char const *model = capture_get_field(cap, "model");
    if (!model || strcmp(model, "AlectoV1-Temperature") != 0)
        return -1; // only the temperature/humidity variant is encoded

    uint8_t air[5];
    if (encoder_template_rows(cap, 36, air, out_bits) < 0)
        return -1;

    int v;
    double t;
    if (encoder_field_int(cap, "id", &v))
        air[0] = reverse8((uint8_t)v);
    if (encoder_field_int(cap, "channel", &v))
        air[0] = (uint8_t)((air[0] & ~0x0c) | ((v & 0x03) << 2));
    if (encoder_field_dbl(cap, "temperature_C", &t)) {
        int temp12  = (int)lround(t * 10.0) & 0xfff;
        uint8_t l1  = reverse8(air[1]);
        l1          = (uint8_t)(((temp12 & 0x0f) << 4) | (l1 & 0x0f)); // keep B/M/M/P
        air[1]      = reverse8(l1);
        air[2]      = reverse8((uint8_t)((temp12 >> 4) & 0xff));
    }
    if (encoder_field_int(cap, "battery_ok", &v)) {
        uint8_t l1 = reverse8(air[1]);
        l1         = (uint8_t)((l1 & ~0x01) | (v ? 0 : 0x01)); // bit0 = battery_low
        air[1]     = reverse8(l1);
    }
    if (encoder_field_int(cap, "humidity", &v))
        air[3] = reverse8((uint8_t)(((v / 10) << 4) | (v % 10)));

    for (int cand = 0; cand < 16; ++cand) { // solve the 4-bit checksum
        air[4] = (uint8_t)(cand << 4);
        if (alecto_checksum(air))
            break;
    }

    // keep the derived id/channel fields consistent with air[0]
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", reverse8(air[0]));
    capture_set_field(cap, "id", buf);
    snprintf(buf, sizeof(buf), "%d", (air[0] & 0x0c) >> 2);
    capture_set_field(cap, "channel", buf);

    encoder_emit_frames(out_bits, 0, air, 36, 8, 0);
    return encoder_verify(decoder, out_bits, cap) ? 1 : -1;
}

static char const *const output_fields[] = {
        "model",
        "id",
        "channel",
        "battery_ok",
        "temperature_C",
        "humidity",
        "rain_mm",
        "wind_avg_m_s",
        "wind_max_m_s",
        "wind_dir_deg",
        "mic",
        NULL,
};

r_device const alectov1 = {
        .name        = "AlectoV1 Weather Sensor (Alecto WS3500 WS4500 Ventus W155/W044 Oregon)",
        .modulation  = OOK_PULSE_PPM,
        .short_width = 2000,
        .long_width  = 4000,
        .gap_limit   = 7000,
        .reset_limit = 10000,
        .decode_fn   = &alectov1_callback,
        .encode_fn   = &alectov1_encode,
        .fields      = output_fields,
};
