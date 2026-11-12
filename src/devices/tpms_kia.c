/** @file
    Kia Rio UB III (UB) 2011-2017 TPMS sensor and some Hyundai models too.

    Copyright (C) 2022 Lasse Mikkel Reinhold, Todor Uzunov aka teou, TTiges, 2019 Andreas
    Spiess, 2017 Christian W. Zuckschwerdt <zany@triq.net>

    This program is free software; you can redistribute it and/or modify it under the terms of
    the GNU General Public License as published by the Free Software Foundation; either version
    2 of the License, or (at your option) any later version.
*/

/**
TPMS sensor for Kio Rio III (UB) 2011-2017 and some Hyundai models. Possibly other brands and
models too.

The sensors have accelerometers that sense the centripetal force in a spinning wheel and begin
to transmit data around 40 km/h. They usually keep transmitting for several minutes after
stopping, but not always; on a few rare occasions they stop instantly. Each sensor sends a
burst of 4-6 packets two times a minutes. The packets in a burst are often, but not always,
identical.

154 bits in a packet. Bit layout (leftmost bit in a field is the most significant):
    zzzzzzzzzzzzzzzz aaaa pppppppp tttttttt iiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii dddddddd ccccc

Legend:
    z: 16-bit preamble = 0xed71. Must be omitted from Manchester-decoding
    a: Unknown, but 0xf in all my own readings
    p: 8-bit pressure given as PSI * 5
    t: 8-bit temperature given as Celsius + 50
    i: 32-bit Sensor ID
    d: Unknown, with different value in each packet
    c: First 5 bits of CRC. We need to append 000 to reach 8 bits. poly=0x07, init=0x76.

NOTE: I often get pressure and temperature values that are outliers (like 200 C or 10 PSI) from
all four sensors, even when CRC is OK. I don't know if all my sensors are defunct or if I have
missed something in the encoding. I have included a "raw" field to make it easier for other
users to investigate it.

NOTE: You may need to use the "-s 1000000" option of rtl_433 in order to get a clear signal.
 */

#include "decoder.h"
#include "r_encoder.h"
#include <stdlib.h>
#include <math.h>

static int tpms_kia_decode(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos)
{
    bitbuffer_t packet_bits = {0};
    uint8_t *b;
    unsigned id;
    uint8_t unknown1;
    uint8_t unknown2;
    uint8_t pressure;

    uint8_t temperature;
    uint8_t crc;
    unsigned int start_pos;
    const unsigned int preamble_length = 16;

    start_pos = bitbuffer_manchester_decode(bitbuffer, row, bitpos, &packet_bits, 154 - preamble_length);
    if (start_pos - bitpos < 154 - preamble_length) {
        return DECODE_ABORT_LENGTH;
    }

    b = packet_bits.bb[0];

    unknown1    = b[0] >> 4;
    pressure    = b[0] << 4 | b[1] >> 4;
    temperature = b[1] << 4 | b[2] >> 4;
    id          = b[2] << 28 | b[3] << 20 | b[4] << 12 | b[5] << 4 | b[6] >> 4;
    unknown2    = b[6] << 8 | b[7];

    // The last 3 bits in b[8] are beyond the packet length of 154 bits. Make them 000.
    crc       = b[8] & (~0x7);
    uint8_t c = crc8(b, 8, 0x07, 0x76);
    if (crc != c) {
        return DECODE_FAIL_MIC;
    }

    char id_str[9 + 1];
    snprintf(id_str, sizeof(id_str), "%08x", id);
    char unknown1_str[2 + 1];
    snprintf(unknown1_str, sizeof(unknown1_str), "%02x", unknown1);
    char unknown2_str[3 + 1];
    snprintf(unknown2_str, sizeof(unknown2_str), "%03x", unknown2);
    char raw[9 * 2 + 1]; // 9 bytes in hex notation
    snprintf(raw, sizeof(raw), "%02x%02x%02x%02x%02x%02x%02x%02x%02x", b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[8]);

    float pressure_float    = pressure / 5.0f;
    float temperature_float = temperature - 50.0f;

    /* clang-format off */
    data_t *data = data_make(
            "model",            "",             DATA_STRING, "Kia",
            "type",             "",             DATA_STRING, "TPMS",
            "id",               "",             DATA_STRING, id_str,
            "unknown1",         "",             DATA_STRING, unknown1_str,
            "unknown2",         "",             DATA_STRING, unknown2_str,
            "pressure_PSI",     "pressure",     DATA_FORMAT, "%.1f PSI", DATA_DOUBLE, pressure_float,
            "temperature_C",    "temperature",  DATA_FORMAT, "%.0f C", DATA_DOUBLE, temperature_float,
            "raw",              "",             DATA_STRING, raw,
            "mic",              "Integrity",    DATA_STRING, "CRC",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

/**
Wrapper for the Kia tpms.
@sa tpms_kia_decode()
*/
static int tpms_kia_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    uint8_t const preamble_pattern[2] = {0xed, 0x71};
    const int preamble_length         = 16;

    unsigned bitpos = 0;
    int ret         = 0;
    int events      = 0;

    // Find a preamble with enough bits after it that it could be a complete packet
    while ((bitpos = bitbuffer_search(bitbuffer, 0, bitpos, preamble_pattern, preamble_length)) + 154 <= bitbuffer->bits_per_row[0]) {
        ret = tpms_kia_decode(decoder, bitbuffer, 0, bitpos + preamble_length);
        if (ret > 0) {
            events += ret;
        }
        bitpos += 2;
    }

    return events > 0 ? events : ret;
}

/** Encoder: inverse of tpms_kia_decode().

    Only the top 5 CRC bits are sent and the decoder requires the low 3 bits
    of the CRC to be zero. If an edit breaks that, the free "d" byte
    (unknown2, varies per packet anyway) is searched for the nearest value
    that yields a valid CRC, unless unknown2 itself was edited.
*/
static int tpms_kia_encode(r_device const *decoder, struct capture *cap, bitbuffer_t *out_bits)
{
    uint8_t const preamble_pattern[2] = {0xed, 0x71};

    if (cap->bits.num_rows < 1)
        return -1;
    bitbuffer_t bits = cap->bits;

    int patched = 0;
    uint8_t b[9];
    unsigned bitpos = 0;
    while ((bitpos = bitbuffer_search(&bits, 0, bitpos, preamble_pattern, 16)) + 154 <= bits.bits_per_row[0]) {
        bitbuffer_t packet = {0};
        bitbuffer_manchester_decode(&bits, 0, bitpos + 16, &packet, 69);
        uint8_t *o = packet.bb[0];
        if (packet.bits_per_row[0] < 69 || (o[8] & 0xf8) != crc8(o, 8, 0x07, 0x76)) {
            bitpos += 2;
            continue;
        }
        memcpy(b, o, 9);

        int v;
        double d;
        char const *f;
        unsigned orig_id = (unsigned)o[2] << 28 | o[3] << 20 | o[4] << 12 | o[5] << 4 | o[6] >> 4;
        if ((f = capture_get_field(cap, "id")) && (unsigned)strtoul(f, NULL, 16) != orig_id) {
            unsigned id = (unsigned)strtoul(f, NULL, 16);
            b[2]        = (uint8_t)((b[2] & 0xf0) | (id >> 28));
            b[3]        = (uint8_t)(id >> 20);
            b[4]        = (uint8_t)(id >> 12);
            b[5]        = (uint8_t)(id >> 4);
            b[6]        = (uint8_t)((id & 0x0f) << 4 | (b[6] & 0x0f));
        }
        if ((f = capture_get_field(cap, "unknown1")) && (strtoul(f, NULL, 16) & 0x0f) != (unsigned)(o[0] >> 4)) {
            b[0] = (uint8_t)((strtoul(f, NULL, 16) & 0x0f) << 4 | (b[0] & 0x0f));
        }
        int u2_edited = 0;
        if ((f = capture_get_field(cap, "unknown2")) && (strtoul(f, NULL, 16) & 0xff) != o[7]) {
            b[7]      = (uint8_t)strtoul(f, NULL, 16);
            u2_edited = 1;
        }
        if (encoder_field_dbl(cap, "pressure_PSI", &d) && fabs(d - ((o[0] << 4 | o[1] >> 4) & 0xff) / 5.0) > 1e-3) {
            long p = lround(d * 5);
            v      = p < 0 ? 0 : p > 0xff ? 0xff : (int)p;
            b[0]   = (uint8_t)((b[0] & 0xf0) | v >> 4);
            b[1]   = (uint8_t)((v & 0x0f) << 4 | (b[1] & 0x0f));
        }
        if (encoder_field_dbl(cap, "temperature_C", &d) && lround(d) != ((o[1] << 4 | o[2] >> 4) & 0xff) - 50) {
            long t = lround(d) + 50;
            v      = t < 0 ? 0 : t > 0xff ? 0xff : (int)t;
            b[1]   = (uint8_t)((b[1] & 0xf0) | v >> 4);
            b[2]   = (uint8_t)((v & 0x0f) << 4 | (b[2] & 0x0f));
        }

        uint8_t c = crc8(b, 8, 0x07, 0x76);
        if (c & 0x07) {
            if (u2_edited)
                return -1;
            uint8_t b7 = b[7];
            for (int i = 1; i < 256 && (c & 0x07); i++) {
                b[7] = (uint8_t)(b7 + ((i & 1) ? (i + 1) / 2 : -(i / 2)));
                c    = crc8(b, 8, 0x07, 0x76);
            }
            if (c & 0x07)
                return -1;
        }
        b[8] = c;

        encoder_manchester_encode(bits.bb[0], bitpos + 16, b, 69, 0);
        patched++;
        bitpos += 154;
    }
    if (!patched)
        return -1;

    char buf[24];
    snprintf(buf, sizeof(buf), "%08x", (unsigned)b[2] << 28 | b[3] << 20 | b[4] << 12 | b[5] << 4 | b[6] >> 4);
    capture_set_field(cap, "id", buf);
    snprintf(buf, sizeof(buf), "%02x", b[0] >> 4);
    capture_set_field(cap, "unknown1", buf);
    snprintf(buf, sizeof(buf), "%03x", b[7]);
    capture_set_field(cap, "unknown2", buf);
    encoder_set_dbl(cap, "pressure_PSI", ((b[0] << 4 | b[1] >> 4) & 0xff) / 5.0);
    encoder_set_dbl(cap, "temperature_C", ((b[1] << 4 | b[2] >> 4) & 0xff) - 50.0);
    snprintf(buf, sizeof(buf), "%02x%02x%02x%02x%02x%02x%02x%02x%02x", b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[8]);
    capture_set_field(cap, "raw", buf);

    *out_bits = bits;
    return encoder_verify(decoder, out_bits, cap) ? 1 : -1;
}

static char const *const output_fields[] = {
        "model",
        "type",
        "id",
        "unknown1",
        "unknown2",
        "pressure_PSI",
        "temperature_C",
        "raw",
        "mic",
        NULL,
};

r_device const tpms_kia = {
        .name        = "Kia TPMS (-s 1000k)",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 50,
        .long_width  = 50,
        .reset_limit = 200,
        .decode_fn   = &tpms_kia_callback,
        .encode_fn   = &tpms_kia_encode,
        .fields      = output_fields,
};
