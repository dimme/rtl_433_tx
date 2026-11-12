/**  @file
    Unbranded SolarTPMS for trucks.

    Copyright (C) 2021 Christian W. Zuckschwerdt <zany@triq.net>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

/**
Unbranded SolarTPMS for trucks, with wheel counter, set of 6.

S.a. #1893

The preamble is 232 bit 0x55..5556.
The data packet is Manchester coded.

Specification:
- Monitoring temperature range: -127 C to 127 C
- Monitoring air pressure range: 0.1 bar to 12.0 bar
- 433 MHz FSK

Data layout (nibbles):

    U II II II II WW F PPP TT CC ?

- U: 4 bit state, decoding unknown, not included in checksum, could be sync
- I: 32 bit ID
- W: 8 bit wheel position
- F: 4 bit status flags: 0x8 = unknown. 0x4 = pressure alert. 0x2|0x1 = battery status (where 00 = low battery).
- P: 12 bit Pressure (kPa)
- T: 8 bit Temperature (deg. C, signed)
- C: 8 bit Checksum (XOR on bytes 0 to 7)
- ?: 4 bit unknown (seems static, probably a decoding artifact)

Example data:

    ID:32h POS:8h FLAGS?4h KPA:12d TEMP:8d CHK:8h TRAILER?8h

    {401} 555555555555555555555555555555555555555555555555555555 5556 99 55 66 95 56 9a 59 95 56 55 59 5a 55 55 55 56 a9 96 9a aa  ff 80
    {401} 555555555555555555555555555555555555555555555555555555 5556 99 a5 66 96 65 99 a9 56 65 55 66 5a 55 55 55 56 a5 a5 59 aa  ff 80

    00000000000000000000000000001 a 0581b281 02 3 000 1e 9b f
    00000000000000000000000000001 a c594ae14 05 3 000 1c c2 f

*/

#include "decoder.h"
#include "capture.h"
#include "r_encoder.h"
#include <stdlib.h>
#include <math.h>

static int tpms_truck_decode(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos)
{
    bitbuffer_t packet_bits = {0};
    bitbuffer_manchester_decode(bitbuffer, row, bitpos, &packet_bits, 76);

    if (packet_bits.bits_per_row[row] < 76) {
        return 0; // DECODE_FAIL_SANITY;
    }

    uint8_t b[9] = {0};
    bitbuffer_extract_bytes(&packet_bits, 0, 4, b, 72);

    // reject all-zero data, the XOR checksum can't catch it
    if (!b[0] && !b[1] && !b[2] && !b[3]) {
        return 0; // DECODE_FAIL_SANITY;
    }

    int chk = xor_bytes(b, 9);
    if (chk != 0) {
        return 0; // DECODE_FAIL_MIC;
    }

    unsigned id        = (unsigned)(b[0] << 24) | (b[1] << 16) | (b[2] << 8) | b[3];
    int wheel          = b[4];
    int flags          = (b[5] >> 4);
    int pressure       = ((b[5] & 0x0f) << 8) | b[6];
    int temperature    = b[7];
    int pressure_alert = (flags & 0x4) == 0x4;
    int battery_ok     = (flags & 0x3) == 0x3; // 0x3 = battery ok, 0x0 = battery low, else unknown.

    char id_str[4 * 2 + 1];
    snprintf(id_str, sizeof(id_str), "%08x", id);

    /* clang-format off */
    data_t *data = data_make(
            "model",          "",               DATA_STRING, "Truck",
            "type",           "",               DATA_STRING, "TPMS",
            "id",             "",               DATA_STRING, id_str,
            "wheel",          "",               DATA_INT,    wheel,
            "pressure_kPa",   "Pressure",       DATA_FORMAT, "%.0f kPa",     DATA_DOUBLE, (float)pressure,
            "temperature_C",  "Temperature",    DATA_FORMAT, "%.0f C",       DATA_DOUBLE, (float)temperature,
            "pressure_alert", "Pressure Alert", DATA_COND,   pressure_alert, DATA_INT,    pressure_alert,
            "battery_ok",     "Battery Ok",     DATA_INT,    battery_ok,
            "flags",          "Flag?",          DATA_FORMAT, "%x",           DATA_INT,    flags,
            "mic",            "Integrity",      DATA_STRING, "CHECKSUM",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

/** Encoder: inverse of tpms_truck_decode().

    Keeps the captured preamble and 4-bit leading state nibble, patches the
    edited fields (id, wheel, flags, pressure, temperature), recomputes the XOR
    checksum and re-derives the battery_ok display field, then re-encodes the
    76-bit Manchester payload in place.
*/
static int tpms_truck_encode(r_device const *decoder, struct capture *cap, bitbuffer_t *out_bits)
{
    if (cap->bits.num_rows < 1)
        return -1;

    uint8_t const preamble_pattern[3] = {0xaa, 0xaa, 0xa9}; // after invert

    bitbuffer_t src = cap->bits;
    bitbuffer_t inv = src;
    bitbuffer_invert(&inv);
    unsigned bitpos = bitbuffer_search(&inv, 0, 0, preamble_pattern, 24);
    if (bitpos + 24 + 152 > src.bits_per_row[0])
        return -1;
    unsigned pstart = bitpos + 24;

    // decode original payload: 4-bit state U followed by 9 message bytes
    bitbuffer_t packet = {0};
    bitbuffer_manchester_decode(&inv, 0, pstart, &packet, 76);
    if (packet.bits_per_row[0] < 76)
        return -1;
    int U = packet.bb[0][0] >> 4;
    uint8_t b[9] = {0};
    bitbuffer_extract_bytes(&packet, 0, 4, b, 72);

    char const *f;
    if ((f = capture_get_field(cap, "id"))) {
        unsigned id = (unsigned)strtoul(f, NULL, 16);
        b[0] = (uint8_t)(id >> 24);
        b[1] = (uint8_t)(id >> 16);
        b[2] = (uint8_t)(id >> 8);
        b[3] = (uint8_t)(id);
    }
    int v;
    if (encoder_field_int(cap, "wheel", &v))
        b[4] = (uint8_t)v;
    if (encoder_field_int(cap, "flags", &v))
        b[5] = (uint8_t)((v << 4) | (b[5] & 0x0f));
    if ((f = capture_get_field(cap, "pressure_kPa"))) {
        int p = (int)lround(atof(f));
        if (p < 0) p = 0;
        if (p > 0xfff) p = 0xfff;
        encoder_set_dbl(cap, "pressure_kPa", p);
        b[5] = (uint8_t)((b[5] & 0xf0) | ((p >> 8) & 0x0f));
        b[6] = (uint8_t)(p & 0xff);
    }
    if ((f = capture_get_field(cap, "temperature_C"))) {
        b[7] = (uint8_t)((int)lround(atof(f)) & 0xff);
        encoder_set_dbl(cap, "temperature_C", b[7]);
    }

    // recompute XOR checksum so xor_bytes(b, 9) == 0
    b[8] = (uint8_t)(b[0] ^ b[1] ^ b[2] ^ b[3] ^ b[4] ^ b[5] ^ b[6] ^ b[7]);

    // re-derive redundant display fields
    int flags = b[5] >> 4;
    char tmp[8];
    snprintf(tmp, sizeof(tmp), "%d", (flags & 0x3) == 0x3 ? 1 : 0);
    capture_set_field(cap, "battery_ok", tmp);

    // assemble the 76-bit payload: U:4 then b[0..8]:72
    uint8_t pay[10] = {0};
    for (int i = 0; i < 4; ++i)
        encoder_set_bit(pay, (unsigned)i, (U >> (3 - i)) & 1);
    for (int k = 0; k < 72; ++k)
        encoder_set_bit(pay, (unsigned)(4 + k), (b[k / 8] >> (7 - (k % 8))) & 1);

    // decode inverts the whole buffer first, so emit (d, 1-d) half-bits
    encoder_manchester_encode(src.bb[0], pstart, pay, 76, 1);

    *out_bits = src;

    if (!encoder_verify(decoder, out_bits, cap))
        return -1;
    return 1;
}

/** @sa tpms_truck_decode() */
static int tpms_truck_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    // preamble
    uint8_t const preamble_pattern[3] = {0xaa, 0xaa, 0xa9}; // after invert

    unsigned bitpos = 0;
    int events      = 0;

    bitbuffer_invert(bitbuffer);
    // Find a preamble with enough bits after it that it could be a complete packet
    while ((bitpos = bitbuffer_search(bitbuffer, 0, bitpos, preamble_pattern, 24)) + 160 <=
            bitbuffer->bits_per_row[0]) {
        events += tpms_truck_decode(decoder, bitbuffer, 0, bitpos + 24);
        bitpos += 2;
    }

    return events;
}

static char const *const output_fields[] = {
        "model",
        "type",
        "id",
        "wheel",
        "pressure_kPa",
        "temperature_C",
        "pressure_alert",
        "battery_ok",
        "flags",
        "mic",
        NULL,
};

r_device const tpms_truck = {
        .name        = "Unbranded SolarTPMS for trucks",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 52,
        .long_width  = 52,
        .reset_limit = 150,
        .decode_fn   = &tpms_truck_callback,
        .encode_fn   = &tpms_truck_encode,
        .fields      = output_fields,
};
