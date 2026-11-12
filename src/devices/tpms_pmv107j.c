/** @file
    FSK 8-byte Differential Manchester encoded TPMS data with CRC-8.

    Copyright (C) 2017 Christian W. Zuckschwerdt <zany@triq.net>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/
/**
FSK 8-byte Differential Manchester encoded TPMS data with CRC-8.
Pacific PMV-107J TPMS (315MHz) sensors used by Toyota
based on work by Werner Johansson.

66 bits Differential Manchester encoded TPMS data with CRC-8.

    II II II I F* PP NN TT CC

- I: ID (28 bit)
- F*: Flags, 6 bits (BCCURF, battery_low, repeat_counter, Unknown, rapid_change, failed)
    battery_low 1 is low, 0 is okay, inverted for battery_ok
    repeat_counter 2 bits, normally each message is 1, 2, 3, then repeat
    Unknown "Must be zero for the packet to be recognized by the car" https://github.com/xnk/pacific-tpms
    rapid_change 1 when there is a rapid pressure change, the sensor will
        not wait for the normal 60 second time slot, and will repeat.
    failed (self test?) 1 if failed, 0 if okay
- P: Tire pressure (PSI/0.363 + 40 or kPa/2.48 + 40)
- N: Inverted tire pressure
- T: Tire temperature (Celsius +40, range from -40 to +215 C)
- C: CRC over bits 0 - 57, poly 0x13, init 0
*/

#include "decoder.h"
#include "r_encoder.h"
#include <stdlib.h>
#include <math.h>

static int tpms_pmv107j_decode(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos)
{
    bitbuffer_t packet_bits = {0};
    uint8_t b[9];

    unsigned start_pos = bitbuffer_differential_manchester_decode(bitbuffer, row, bitpos, &packet_bits, 70); // 67 bits expected
    if (start_pos - bitpos < 67 * 2) {
        return 0;
    }
    decoder_log_bitbuffer(decoder, 2, __func__, &packet_bits, "");

    // realign the buffer, prepending 6 bits of 0.
    b[0] = packet_bits.bb[0][0] >> 6;
    bitbuffer_extract_bytes(&packet_bits, 0, 2, b + 1, 64);
    decoder_log_bitrow(decoder, 2, __func__, b, 72, "Realigned");

    int crc = b[8];
    if (crc8(b, 8, 0x13, 0x00) != crc) {
        return 0;
    }

    unsigned id            = b[0] << 26 | b[1] << 18 | b[2] << 10 | b[3] << 2 | b[4] >> 6; // realigned bits 6 - 34
    unsigned status        = b[4] & 0x3f;                                                  // status bits and 0 filler
    unsigned battery_low   = (b[4] & 0x20) >> 5;
    unsigned counter       = (b[4] & 0x18) >> 3;
    // unknown bit         = (b[4] & 0x4) >> 2;
    unsigned rapid_change  = (b[4] & 0x2) >> 1;
    unsigned failed        = b[4] & 0x01;
    unsigned pressure1     = b[5];
    unsigned pressure2     = b[6] ^ 0xff;
    unsigned temp          = b[7];
    float pressure_kpa  = (pressure1 - 40.0f) * 2.48f;
    float temperature_c = temp - 40.0f;

    if (pressure1 != pressure2) {
        decoder_logf(decoder, 1, __func__, "Toyota TPMS pressure check error: %02x vs %02x", pressure1, pressure2);
        return 0;
    }

    char id_str[9];
    snprintf(id_str, sizeof(id_str), "%08x", id);

    /* clang-format off */
    data_t *data = data_make(
            "model",            "",             DATA_STRING,    "PMV-107J",
            "type",             "",             DATA_STRING,    "TPMS",
            "id",               "",             DATA_STRING,    id_str,
            "status",           "",             DATA_INT,       status,
            "battery_ok",       "",             DATA_INT,       !battery_low,
            "counter",          "",             DATA_INT,       counter,
            "rapid_change",     "",             DATA_INT,       rapid_change,
            "failed",           "",             DATA_STRING,    failed ? "FAIL" : "OK",
            "pressure_kPa",     "",             DATA_DOUBLE,    pressure_kpa,
            "temperature_C",    "",             DATA_FORMAT,    "%.1f C", DATA_DOUBLE, temperature_c,
            "mic",              "Integrity",    DATA_STRING,    "CRC",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

/** @sa tpms_pmv107j_decode() */
static int tpms_pmv107j_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    // full preamble is (7 bits) 11111 10
    uint8_t const preamble_pattern[1] = {0xf8}; // 6 bits

    unsigned bitpos = 0;
    int ret         = 0;
    int events      = 0;

    // Find a preamble with enough bits after it that it could be a complete packet
    while ((bitpos = bitbuffer_search(bitbuffer, 0, bitpos, preamble_pattern, 6)) + 67 * 2 <=
            bitbuffer->bits_per_row[0]) {
        ret = tpms_pmv107j_decode(decoder, bitbuffer, 0, bitpos + 6);
        if (ret > 0)
            events += ret;
        bitpos += 2;
    }

    return events > 0 ? events : ret;
}

/** Encoder: inverse of tpms_pmv107j_decode().

    Patches id, status bits, pressure (and its inverted copy) and
    temperature in every packet, recomputes the CRC-8 and re-encodes the 67
    differential Manchester bits in place. The status byte is taken from
    "status" if edited, otherwise from the individual status fields.
*/
static int tpms_pmv107j_encode(r_device const *decoder, struct capture *cap, bitbuffer_t *out_bits)
{
    uint8_t const preamble_pattern[1] = {0xf8};

    if (cap->bits.num_rows < 1)
        return -1;
    bitbuffer_t bits = cap->bits;

    int patched     = 0;
    unsigned bitpos = 0;
    while ((bitpos = bitbuffer_search(&bits, 0, bitpos, preamble_pattern, 6)) + 67 * 2 <= bits.bits_per_row[0]) {
        unsigned start     = bitpos + 6;
        bitbuffer_t packet = {0};
        unsigned end       = bitbuffer_differential_manchester_decode(&bits, 0, start, &packet, 70);
        uint8_t b[9];
        b[0] = packet.bb[0][0] >> 6;
        bitbuffer_extract_bytes(&packet, 0, 2, b + 1, 64);
        if (end - start < 67 * 2 || crc8(b, 8, 0x13, 0x00) != b[8] || b[5] != (b[6] ^ 0xff)) {
            bitpos += 2;
            continue;
        }

        unsigned orig = b[4] & 0x3f;
        unsigned st   = orig;
        char const *f;
        int v;
        double d;
        if (encoder_field_int(cap, "status", &v) && (unsigned)(v & 0x3f) != orig) {
            st = (unsigned)v & 0x3f;
        }
        else {
            if (encoder_field_int(cap, "battery_ok", &v))
                st = (st & ~0x20u) | (v ? 0 : 0x20);
            if (encoder_field_int(cap, "counter", &v))
                st = (st & ~0x18u) | ((unsigned)v & 3) << 3;
            if (encoder_field_int(cap, "rapid_change", &v))
                st = (st & ~0x02u) | (v ? 0x02 : 0);
            if ((f = capture_get_field(cap, "failed")))
                st = (st & ~0x01u) | (!strcmp(f, "FAIL") ? 0x01 : 0);
        }
        unsigned id = (unsigned)b[0] << 26 | b[1] << 18 | b[2] << 10 | b[3] << 2 | b[4] >> 6;
        if ((f = capture_get_field(cap, "id")))
            id = (unsigned)strtoul(f, NULL, 16) & 0x0fffffff; // 28 bit id
        b[0] = (uint8_t)(id >> 26 & 0x03);
        b[1] = (uint8_t)(id >> 18);
        b[2] = (uint8_t)(id >> 10);
        b[3] = (uint8_t)(id >> 2);
        b[4] = (uint8_t)((id & 0x03) << 6 | st);
        if (encoder_field_dbl(cap, "pressure_kPa", &d)) {
            long p = lround(d / 2.48) + 40;
            b[5]   = (uint8_t)(p < 0 ? 0 : p > 0xff ? 0xff : p);
        }
        b[6] = b[5] ^ 0xff;
        if (encoder_field_dbl(cap, "temperature_C", &d)) {
            long t = lround(d) + 40;
            b[7]   = (uint8_t)(t < 0 ? 0 : t > 0xff ? 0xff : t);
        }
        b[8] = crc8(b, 8, 0x13, 0x00);

        // 67 coded bits: the 2 low bits of b[0], b[1..8], then the original last bit
        uint8_t msg[9] = {0};
        for (unsigned i = 0; i < 67; i++) {
            int bit = i < 66 ? (b[(i + 6) / 8] >> (7 - (i + 6) % 8)) & 1 : bitrow_get_bit(packet.bb[0], 66);
            if (bit)
                msg[i / 8] |= (uint8_t)(0x80 >> (i % 8));
        }
        int prev = bitrow_get_bit(bits.bb[0], start - 1);
        encoder_diff_manchester_encode(bits.bb[0], start, &prev, msg, 67);

        char buf[16];
        snprintf(buf, sizeof(buf), "%08x", id);
        capture_set_field(cap, "id", buf);
        snprintf(buf, sizeof(buf), "%u", st);
        capture_set_field(cap, "status", buf);
        snprintf(buf, sizeof(buf), "%d", !(st & 0x20));
        capture_set_field(cap, "battery_ok", buf);
        snprintf(buf, sizeof(buf), "%u", (st & 0x18) >> 3);
        capture_set_field(cap, "counter", buf);
        snprintf(buf, sizeof(buf), "%u", (st & 0x02) >> 1);
        capture_set_field(cap, "rapid_change", buf);
        capture_set_field(cap, "failed", (st & 0x01) ? "FAIL" : "OK");
        encoder_set_dbl(cap, "pressure_kPa", (double)((b[5] - 40.0f) * 2.48f));
        encoder_set_dbl(cap, "temperature_C", (double)(b[7] - 40.0f));

        patched++;
        bitpos = start + 67 * 2;
    }
    if (!patched)
        return -1;

    *out_bits = bits;
    return encoder_verify(decoder, out_bits, cap) ? 1 : -1;
}

static char const *const output_fields[] = {
        "model",
        "type",
        "id",
        "status",
        "battery_ok",
        "counter",
        "rapid_change",
        "failed",
        "pressure_kPa",
        "temperature_C",
        "mic",
        NULL,
};

r_device const tpms_pmv107j = {
        .name        = "PMV-107J (Toyota) TPMS",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 100, // 25 samples @250k
        .long_width  = 100, // FSK
        .reset_limit = 250, // Maximum gap size before End Of Message [us].
        .decode_fn   = &tpms_pmv107j_callback,
        .encode_fn   = &tpms_pmv107j_encode,
        .fields      = output_fields,
};
