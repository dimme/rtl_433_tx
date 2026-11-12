/** @file
    TRW TPMS Sensor.

    Copyright (C) 2025 Bruno OCTAU \@ProfBoc75

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

#include "decoder.h"
#include "r_encoder.h"
#include <stdlib.h>
#include <math.h>

/** @fn int tpms_trw_decode(r_device *decoder, bitbuffer_t *bitbuffer, int type)
TRW TPMS Sensor.

FCC-ID: GQ4-70T

- OEM and Clone OEM models
- Used into Chrysler car from 2014 until 2022 : car models https://www.chryslertpms.com/chrysler-tpms-types-fitment

- OOK then FSK rf signals, mode detail here : https://fcc.report/FCC-ID/GQ4-70T/2201535

S.a issue #3256

 Data layout:

    Byte Position xx xx 0  1  2  3  4  5  6  7  8  9  10
    Sample OOK    00 01 5c 3e 52 85 2e 61 53 4b 0e 52 4
    Sample FSK    7f ff 5c 3e 52 85 2e 62 53 4b 0e 68 4
                   PRE  MM II II II II FN PP TT SS CC X

- PRE : 7FFF (FSK) or 0001 (OOK)

- M:{8}  Mode/Model : 0x5c, 0x5d or 0x5e (Always 0x5c for Clone OEM model)
- I:{32} Sensor ID
- F:{4}  Flag status : 0x6 = pressure drop (OEM) , 0x9 pressure increase (OEM) or drop (Clone OEM), 0xb or 0xc alternatly in Motion
- N:{4}  Seq number : 0,1,2,3,0,1,2,3... for OEM, 1,2,3,4,1,2,3,4... for Clone OEM
- P:{8}  Pressure PSI, scale 2.5
- T:{8}  Temperature C, offset 50
- S:{8}  Motion status, 0x0e parked, other value in motion, not yet guesses
- C:{8}  CRC-8/SMBUS, poly 0x07, init 0x00, final XOR 0x00
- X:{4}  Trailing bit but 0x4 OEM, 0x0 Clone OEM

*/

static unsigned char const preamble_ook[] = { 0x00, 0x01 };
static unsigned char const preamble_fsk[] = { 0x7f, 0xff };

static int tpms_trw_decode(r_device *decoder, bitbuffer_t *bitbuffer, int type)
{
    if (bitbuffer->num_rows != 1) {
        return DECODE_ABORT_EARLY;
    }

    int msg_len = bitbuffer->bits_per_row[0];

    if (msg_len > 98) {
        decoder_logf(decoder, 1, __func__, "Packet too long: %d bits", msg_len);
        return DECODE_ABORT_LENGTH;
    }

    int pos = 0;

    if (type == 0) {
        pos = bitbuffer_search(bitbuffer, 0, pos, preamble_ook, sizeof(preamble_ook) * 8);
    }
    if (type == 1) {
        pos = bitbuffer_search(bitbuffer, 0, pos, preamble_fsk, sizeof(preamble_fsk) * 8);
    }

    if (pos >= bitbuffer->bits_per_row[0]) {
        decoder_log(decoder, 2, __func__, "Preamble not found");
        return DECODE_ABORT_EARLY;
    }

    if (pos + 8 * 11 > bitbuffer->bits_per_row[0]) {
        decoder_log(decoder, 2, __func__, "Length check fail");
        return DECODE_ABORT_LENGTH;
    }

    uint8_t b[11]  = {0};
    pos += 16;

    if ((msg_len - pos) < 81 ) {
        decoder_logf(decoder, 1, __func__, "Packet too short: %d bits", msg_len);
        return DECODE_ABORT_LENGTH;
    }

    bitbuffer_extract_bytes(bitbuffer, 0, pos, b, sizeof(b) * 8);

    if (crc8(b,10,0x07,0x00)) {
        decoder_logf(decoder, 1, __func__, "CRC Error, expected: %02x", crc8(b, 9, 0x07, 0x00));
        return DECODE_FAIL_MIC;
    }

    decoder_log_bitrow(decoder, 2, __func__, b, 88, "MSG");

    int mode               = b[0];
    int id                 = b[1] << 24 | b[2] << 16 | b[3] << 8 | b[4];
    int flags              = (b[5] & 0xF0) >> 4;
    int seq_num            = b[5] & 0x0F;
    int pressure_raw       = b[6];
    float pressure_psi     = pressure_raw * 0.4f;
    int temp_raw           = b[7];
    int temperature_C      = temp_raw - 50;
    int motion_flags       = b[8];
    int oem_model          = (b[10] & 0xf0) >> 4;

    /* clang-format off */
    data_t *data = data_make(
            "model",            "",             DATA_STRING, "TRW",
            "type",             "",             DATA_STRING, "TPMS",
            "mode",             "",             DATA_FORMAT, "%02x",     DATA_INT, mode,
            "id",               "",             DATA_FORMAT, "%08x",     DATA_INT, id,
            //"battery_ok",       "Battery_OK",   DATA_INT,    !low_batt,
            "flags",            "Flags",        DATA_FORMAT, "%01x",     DATA_INT, flags,
            "alert",            "Alert",        DATA_COND, flags == 0x6 || flags == 0x9, DATA_STRING, "Pressure increase/decrease !",
            "seq_num",          "Seq Num",                               DATA_INT, seq_num,
            "pressure_PSI",     "Pressure",     DATA_FORMAT, "%.1f PSI", DATA_DOUBLE, pressure_psi,
            "temperature_C",    "Temperature",  DATA_FORMAT, "%.0f C",   DATA_DOUBLE, (double)temperature_C,
            "motion_flags",     "Motion flags", DATA_FORMAT, "%02x",     DATA_INT, motion_flags,
            "motion_status",    "Motion",       DATA_STRING, motion_flags == 0x0e ? "Parked" : "Moving",
            "oem_model",        "OEM Model",    DATA_COND,   oem_model == 0x4, DATA_STRING, "OEM",
            "oem_model",        "OEM Model",    DATA_COND,   oem_model == 0x0, DATA_STRING, "Clone",
            "mic",              "Integrity",    DATA_STRING, "CRC",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

/**
TRW TPMS Sensor.
@sa tpms_trw_decode()
*/
static int tpms_trw_callback_ook(r_device *decoder, bitbuffer_t *bitbuffer)
{
    return tpms_trw_decode(decoder, bitbuffer, 0);
}

/**
TRW TPMS Sensor.
@sa tpms_trw_decode()
*/
static int tpms_trw_callback_fsk(r_device *decoder, bitbuffer_t *bitbuffer)
{
    return tpms_trw_decode(decoder, bitbuffer, 1);
}

/** Encoder: inverse of tpms_trw_decode() for both the OOK and FSK variant.

    Rows hold NRZ bits (the demodulator Manchester-decodes). Patches mode, id,
    flags, seq_num, pressure, temperature, motion flags and the trailing OEM
    nibble, recomputes the CRC and re-derives the display-only fields.
    Edits to motion_status (with motion_flags unchanged) map to 0x0e (Parked)
    or the captured/0x00 moving value.
*/
static int tpms_trw_encode(r_device const *decoder, struct capture *cap, bitbuffer_t *out_bits)
{
    if (cap->bits.num_rows != 1)
        return -1;

    int is_fsk = decoder->modulation == FSK_PULSE_MANCHESTER_ZEROBIT;
    bitbuffer_t src = cap->bits;
    unsigned len = src.bits_per_row[0];
    unsigned pos = bitbuffer_search(&src, 0, 0, is_fsk ? preamble_fsk : preamble_ook, 16);
    if (pos + 16 + 81 > len)
        return -1;
    pos += 16;

    uint8_t b[11] = {0};
    bitbuffer_extract_bytes(&src, 0, pos, b, 88);

    char const *f;
    int v;
    double d;
    if (encoder_field_int(cap, "mode", &v))
        b[0] = (uint8_t)v;
    if ((f = capture_get_field(cap, "id"))) {
        uint32_t id = (uint32_t)strtoll(f, NULL, 10);
        b[1] = (uint8_t)(id >> 24);
        b[2] = (uint8_t)(id >> 16);
        b[3] = (uint8_t)(id >> 8);
        b[4] = (uint8_t)id;
    }
    if (encoder_field_int(cap, "flags", &v))
        b[5] = (uint8_t)((b[5] & 0x0f) | ((v & 0x0f) << 4));
    if (encoder_field_int(cap, "seq_num", &v))
        b[5] = (uint8_t)((b[5] & 0xf0) | (v & 0x0f));
    if (encoder_field_dbl(cap, "pressure_PSI", &d)) {
        long p = lround(d / 0.4);
        b[6] = (uint8_t)(p < 0 ? 0 : p > 0xff ? 0xff : p);
        encoder_set_dbl(cap, "pressure_PSI", b[6] * 0.4);
    }
    if (encoder_field_dbl(cap, "temperature_C", &d)) {
        long t = lround(d) + 50;
        b[7] = (uint8_t)(t < 0 ? 0 : t > 0xff ? 0xff : t);
        encoder_set_dbl(cap, "temperature_C", b[7] - 50);
    }
    int orig_motion = b[8];
    if (encoder_field_int(cap, "motion_flags", &v))
        b[8] = (uint8_t)v;
    if (b[8] == orig_motion && (f = capture_get_field(cap, "motion_status"))) {
        int parked = !strcmp(f, "Parked");
        if (parked != (orig_motion == 0x0e))
            b[8] = parked ? 0x0e : 0x00;
    }
    if ((f = capture_get_field(cap, "oem_model"))) {
        if (!strcmp(f, "OEM"))
            b[10] = (uint8_t)((b[10] & 0x0f) | 0x40);
        else if (!strcmp(f, "Clone"))
            b[10] = (uint8_t)(b[10] & 0x0f);
    }

    b[9] = crc8(b, 9, 0x07, 0x00);

    for (unsigned k = 0; k < 88 && pos + k < len; ++k)
        encoder_set_bit(src.bb[0], pos + k, (b[k / 8] >> (7 - (k % 8))) & 1);
    // re-read so derived fields reflect the bits that actually fit in the row
    bitbuffer_extract_bytes(&src, 0, pos, b, 88);

    char tmp[16];
    int flags = b[5] >> 4;
    snprintf(tmp, sizeof(tmp), "%d", b[8]);
    capture_set_field(cap, "motion_flags", tmp);
    capture_set_field(cap, "motion_status", b[8] == 0x0e ? "Parked" : "Moving");
    if (flags == 0x6 || flags == 0x9)
        capture_set_field(cap, "alert", "Pressure increase/decrease !");
    else
        capture_del_field(cap, "alert");
    int oem = b[10] >> 4;
    if (oem == 0x4 || oem == 0x0)
        capture_set_field(cap, "oem_model", oem == 0x4 ? "OEM" : "Clone");
    else
        capture_del_field(cap, "oem_model");

    *out_bits = src;

    if (!encoder_verify(decoder, out_bits, cap))
        return -1;
    return 1;
}

static char const *const output_fields[] = {
        "model",
        "type",
        "mode",
        "id",
        //"battery_ok",
        "flags",
        "alert",
        "seq_num",
        "pressure_PSI",
        "temperature_C",
        "motion_flags",
        "motion_status",
        //"fast_leak",
        //"inflate",
        "oem_model",
        "mic",
        NULL,
};

r_device const tpms_trw_ook = {
        .name        = "TRW TPMS OOK OEM and Clone models",
        .modulation  = OOK_PULSE_MANCHESTER_ZEROBIT,
        .short_width = 52,
        .long_width  = 52,
        .reset_limit = 150,
        .decode_fn   = &tpms_trw_callback_ook,
        .encode_fn   = &tpms_trw_encode,
        .fields      = output_fields,
};

r_device const tpms_trw_fsk = {
        .name        = "TRW TPMS FSK OEM and Clone models",
        .modulation  = FSK_PULSE_MANCHESTER_ZEROBIT,
        .short_width = 52,
        .long_width  = 52,
        .reset_limit = 150,
        .decode_fn   = &tpms_trw_callback_fsk,
        .encode_fn   = &tpms_trw_encode,
        .fields      = output_fields,
};
