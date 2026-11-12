/** @file
    EezTire E618 TPMS and Carchet TPMS (same protocol).

    Copyright (C) 2023 Bruno OCTAU (ProfBoc75), Gliebig, and Karen Suhm

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
EezTire E618 TPMS and Carchet TPMS (same protocol).

- Eez RV supported TPMS sensor model E618 : https://eezrvproducts.com/shop/ols/products/tpms-system-e518-anti-theft-replacement-sensor-1-ea
- Carchet TPMS: http://carchet.easyofficial.com/carchet-rv-trailer-car-solar-tpms-tire-pressure-monitoring-system-6-sensor-lcd-display-p6.html
- TST (Truck Systems Technologies) 507 Series TPMS : https://github.com/cterwilliger/tst_tpms

The device uses OOK (ASK) encoding.
The device sends a transmission every 1 second when quick deflation is detected, every 13 - 23 sec when quick inflation is detected, and every 4 min 40 s under steady state pressure.
A transmission starts with a preamble of 0x0000 and the packet is sent twice.

S.a issue #2384, #2657, #2063, #2677, #2819

Data collection parameters on URH software were as follows:
    Sensor frequency: 433.92 MHz
    Sample rate: 2.0 MSps
    Bandwidth: 2.0 Hz
    Gain: 125

    Modulation is ASK (OOK). Packets in URH arrive in the following format:

    aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa [Pause: 897679 samples]
    aaaaaaaa5956a5a5a6555aaa65959999a5aaaaaa [Pause: 6030 samples]
    aaaaaaaa5956a5a5a6555aaa65959999a5aaaaaa [Pause: 11176528 samples]

    Decoding is Manchester I.  After decoding, the packets look like this:

    00000000000000000000000000000000000000
    0000de332fc0b7553000
    0000de332fc0b7553000

 Using rtl_433 software, packets were detected using the following command line entry:
 rtl_433 -X "n=Carchet,m=OOK_MC_ZEROBIT,s=50,l=50,r=1000,invert" -s 1M

 Data layout:

    PRE CC IIIIII PP TT FF FF

- PRE : FFFF
- C : 8 bit CheckSum, modulo 256 with overflow flag
- I: 24 bit little-endian ID
- P: 8 bit pressure  P * 2.5 = Pressure kPa
- T: 8 bit temperature   T - 50 = Temperature C
- F: 16 bit status flags: 0x8000 = low battery, 0x1000 = quick deflation, 0x3000 = quick inflation, 0x0000 = static/steady state

Raw Data example :

    ffff 8b 0d177e 8f 4a 10 00

Format string:

    CHECKSUM:8h ID:24h KPA:8d TEMP:8d FLAG:8b 8b

Decode example:

    CHECKSUM:8b ID:0d177e KPA:8f TEMP:4a FLAG:10 00

*/

static int tpms_eezrv_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    // preamble is ffff
    uint8_t const preamble_pattern[] = {0xff, 0xff}; // full 16-bit preamble

    if (bitbuffer->num_rows != 1) {
        return DECODE_ABORT_EARLY;
    }
    int pos = 0;
    bitbuffer_invert(bitbuffer);
    pos = bitbuffer_search(bitbuffer, 0, pos, preamble_pattern, sizeof(preamble_pattern) * 8);
    if (pos >= bitbuffer->bits_per_row[0]) {
        decoder_log(decoder, 3, __func__, "Preamble not found");
        return DECODE_ABORT_EARLY;
    }
    // The preamble must start at 0, we ca not anchor it to the data
    if (pos > 0) {
        decoder_log(decoder, 1, __func__, "Likely false positive");
        return DECODE_ABORT_EARLY;
    }
    // Check for 2 bytes preamble + 1 byte chk + 7 byte data
    if (pos + 10 * 8 > bitbuffer->bits_per_row[0]) {
        decoder_log(decoder, 2, __func__, "Length check fail");
        return DECODE_ABORT_LENGTH;
    }
    uint8_t b[7]  = {0};
    uint8_t cc[1] = {0};
    bitbuffer_extract_bytes(bitbuffer, 0, pos + 16, cc, sizeof(cc) * 8);
    bitbuffer_extract_bytes(bitbuffer, 0, pos + 24, b, sizeof(b) * 8);

    // Verify checksum
    // If the checksum is greater than 0xFF then the MSB is set.
    // It occurs whether the bit is already set or not and was observed when checksum was in the 0x1FF and the 0x2FF range.
    int computed_checksum = add_bytes(b, sizeof(b));
    if (computed_checksum > 0xff) {
        computed_checksum |= 0x80;
    }

    if ((computed_checksum & 0xff) != cc[0]) {
        decoder_log(decoder, 2, __func__, "Checksum fail");
        return DECODE_FAIL_MIC;
    }

    int temperature_C      = b[4] - 50;
    int flags1             = b[5];
    int flags2             = b[6];
    int fast_leak_detected = (flags1 & 0x10);      // fast leak - reports every second
    int infl_detected      = (flags1 & 0x20) >> 5; // inflating - reports every 15 - 20 sec

    int fast_leak      = fast_leak_detected && !infl_detected;
    float pressure_kPa = (((flags2 & 0x01) << 8) + b[3]) * 2.5f;

    // Low batt = 0x8000;
    int low_batt = flags1 >> 7; // Low batt flag is MSB (activated at V < 3.15 V)(Device fails at V < 3.10 V)
    // Mystery flag at (flags2 & 0x20) showed up during low batt testing

    char id_str[7];
    snprintf(id_str, sizeof(id_str), "%02x%02x%02x", b[0], b[1], b[2]);

    char flags_str[5];
    snprintf(flags_str, sizeof(flags_str), "%02x%02x", flags1, flags2);

    /* clang-format off */
    data_t *data = data_make(
            "model",            "",             DATA_STRING, "EezTire-E618",
            "type",             "",             DATA_STRING, "TPMS",
            "id",               "",             DATA_STRING, id_str,
            "battery_ok",       "Battery_OK",   DATA_INT,    !low_batt,
            "pressure_kPa",     "Pressure",     DATA_FORMAT, "%.0f kPa", DATA_DOUBLE, (double)pressure_kPa,
            "temperature_C",    "Temperature",  DATA_FORMAT, "%.1f C", DATA_DOUBLE, (double)temperature_C,
            "flags",            "Flags",        DATA_STRING, flags_str,
            "fast_leak",        "Fast Leak",    DATA_INT,    fast_leak,
            "inflate",          "Inflate",      DATA_INT,    infl_detected,
            "mic",              "Integrity",    DATA_STRING, "CHECKSUM",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

/** Encoder: inverse of tpms_eezrv_decode().

    The decoder inverts the row, so work on an inverted copy. Patches id,
    pressure, temperature and flags, recomputes the checksum and re-derives the
    flag display fields. Edits to battery_ok / fast_leak / inflate (with flags
    left unchanged) are folded into the flags.
*/
static int tpms_eezrv_encode(r_device const *decoder, struct capture *cap, bitbuffer_t *out_bits)
{
    uint8_t const preamble_pattern[] = {0xff, 0xff};

    if (cap->bits.num_rows != 1)
        return -1;

    bitbuffer_t src = cap->bits;
    bitbuffer_invert(&src);
    if (bitbuffer_search(&src, 0, 0, preamble_pattern, 16) != 0 || src.bits_per_row[0] < 80)
        return -1;

    uint8_t b[7] = {0};
    bitbuffer_extract_bytes(&src, 0, 24, b, 56);

    char const *f;
    int v;
    double d;
    if ((f = capture_get_field(cap, "id"))) {
        uint32_t id = (uint32_t)strtoul(f, NULL, 16);
        b[0] = (uint8_t)(id >> 16);
        b[1] = (uint8_t)(id >> 8);
        b[2] = (uint8_t)id;
    }

    int orig = (b[5] << 8) | b[6];
    int flags = orig;
    if ((f = capture_get_field(cap, "flags")))
        flags = (int)strtoul(f, NULL, 16) & 0xffff;
    if (flags == orig) {
        int o1 = flags >> 8;
        int f1 = o1;
        if (encoder_field_int(cap, "battery_ok", &v) && !!v != !(o1 >> 7))
            f1 = v ? (f1 & ~0x80) : (f1 | 0x80);
        if (encoder_field_int(cap, "inflate", &v) && !!v != ((o1 >> 5) & 1))
            f1 = v ? (f1 | 0x30) : (f1 & ~0x20);
        if (encoder_field_int(cap, "fast_leak", &v) && !!v != ((o1 & 0x10) && !(o1 & 0x20)))
            f1 = v ? ((f1 | 0x10) & ~0x20) : (f1 & ~0x30);
        flags = (f1 << 8) | (flags & 0xff);
    }
    b[5] = (uint8_t)(flags >> 8);
    b[6] = (uint8_t)flags;

    if (encoder_field_dbl(cap, "pressure_kPa", &d)) {
        long p = lround(d / 2.5);
        p = p < 0 ? 0 : p > 0x1ff ? 0x1ff : p;
        b[3] = (uint8_t)p;
        b[6] = (uint8_t)((b[6] & 0xfe) | (p >> 8));
    }
    if (encoder_field_dbl(cap, "temperature_C", &d)) {
        long t = lround(d) + 50;
        b[4] = (uint8_t)(t < 0 ? 0 : t > 0xff ? 0xff : t);
    }

    int chk = add_bytes(b, sizeof(b));
    if (chk > 0xff)
        chk |= 0x80;

    // re-derive the flag display fields
    char tmp[8];
    int f1 = b[5];
    snprintf(tmp, sizeof(tmp), "%02x%02x", b[5], b[6]);
    capture_set_field(cap, "flags", tmp);
    snprintf(tmp, sizeof(tmp), "%d", !(f1 >> 7));
    capture_set_field(cap, "battery_ok", tmp);
    snprintf(tmp, sizeof(tmp), "%d", (f1 & 0x10) && !(f1 & 0x20));
    capture_set_field(cap, "fast_leak", tmp);
    snprintf(tmp, sizeof(tmp), "%d", (f1 >> 5) & 1);
    capture_set_field(cap, "inflate", tmp);

    uint8_t msg[8] = {(uint8_t)chk};
    memcpy(&msg[1], b, 7);
    for (unsigned k = 0; k < 64; ++k)
        encoder_set_bit(src.bb[0], 16 + k, (msg[k / 8] >> (7 - (k % 8))) & 1);

    bitbuffer_invert(&src);
    *out_bits = src;

    if (!encoder_verify(decoder, out_bits, cap))
        return -1;
    return 1;
}

static char const *const output_fields[] = {
        "model",
        "type",
        "id",
        "battery_ok",
        "pressure_kPa",
        "temperature_C",
        "flags",
        "fast_leak",
        "inflate",
        "mic",
        NULL,
};

r_device const tpms_eezrv = {
        .name        = "EezTire E618, Carchet TPMS, TST-507 TPMS",
        .modulation  = OOK_PULSE_MANCHESTER_ZEROBIT,
        .short_width = 50,
        .long_width  = 50,
        .reset_limit = 120,
        .decode_fn   = &tpms_eezrv_decode,
        .encode_fn   = &tpms_eezrv_encode,
        .fields      = output_fields,
};
