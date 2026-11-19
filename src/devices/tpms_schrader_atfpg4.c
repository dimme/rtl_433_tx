/** @file
    Schrader ATFPG4 heavy-duty truck TPMS.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/
/**
Schrader ATFPG4 heavy-duty truck TPMS.

Valve mounted Schrader/Sensata sensor for heavy trucks, FCC ID MRXATFPG4 / 2ATIMATFPG4.

Signal:
- 433.92 MHz FSK, about +/-42 kHz deviation
- Manchester coded, 52 us half-bit width (9600 bit/s)
- A transmission is 3 identical frames of about 11.4 ms, 100 to 185 ms apart
- Parked sensors transmit about every 122 s

Frame:
- Preamble of alternating chips, ending in the sync {16} 0xaa59 (raw, before MC)
- 11 bytes after MC, inverted (the same framing as the BMW Gen5 decoder, at half the chip rate)

Data layout:

     0  1  2  3  4  5  6  7  8  9 10
    MM II II II II PP TT F1 F2 F3 CC

- MM : Message type, 0x24 for the pressure message
- II : Sensor ID, 32 bit. The first byte was 0x5c on all 8 sensors seen so far
- PP : Pressure, 0.055 bar per count, gauge (0x95 = 8.195 bar, display showed 8.2 bar)
- TT : Temperature, offset 52 C. The offset is not certain, it could be 50
- F1 : Motion, 0x00 on a parked truck, larger values when driving (possibly speed)
- F2 : Sequence number on a parked truck, +1 per transmission. Other use when driving, unknown
- F3 : Status flags, unknown. 0xe4, 0xe8, 0xe0, 0xd8 seen parked (constant per sensor), 0xe5 driving
- CC : CRC-8 of the previous 10 bytes, poly 0x2f, init 0xaa

Flex decoder:

    rtl_433 -R 0 -X 'n=Schrader-ATFPG4,m=FSK_PCM,s=52,l=52,r=800,invert,preamble=55a6,decode_mc,bits>=88'

Example data (after MC, inverted):

    24 5c 3f 7b b1 95 34 00 43 e4 22 : 8.195 bar,  0 C
    24 5c 3f 7b b1 95 34 00 44 e4 e3 : same sensor, next transmission
    24 5c 3f a0 dc 94 34 00 4a e8 c6 : 8.140 bar,  0 C
    24 5c 3f 77 06 78 34 00 4c e4 17 : 6.600 bar,  0 C
    24 5c 3f 7b b1 a3 47 00 ac e4 f9 : 8.965 bar, 19 C
    24 5c 3f 7b b1 9d 3c 49 75 e5 05 : 8.635 bar,  8 C, driving

There is also a 12 byte message type 0x27 with the same CRC, meaning unknown, not decoded here:

    27 5c 3f 78 97 00 00 00 2c 27 9e 01

*/

#include "decoder.h"
#include "capture.h"
#include "encoder.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ATFPG4_MSG_LEN 11

static int tpms_schrader_atfpg4_decode(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos)
{
    bitbuffer_t decoded = {0};
    bitbuffer_manchester_decode(bitbuffer, row, bitpos, &decoded, ATFPG4_MSG_LEN * 8);

    if (decoded.bits_per_row[0] < ATFPG4_MSG_LEN * 8) {
        return DECODE_ABORT_LENGTH;
    }

    bitbuffer_invert(&decoded);
    uint8_t *b = decoded.bb[0];

    // Only the 11 byte pressure message is known
    if (b[0] != 0x24) {
        return DECODE_ABORT_EARLY;
    }

    if (crc8(b, ATFPG4_MSG_LEN, 0x2f, 0xaa)) {
        return DECODE_FAIL_MIC;
    }

    decoder_log_bitrow(decoder, 2, __func__, b, ATFPG4_MSG_LEN * 8, "MSG");

    double pressure_bar = b[5] * 0.055;
    int temperature_c   = b[6] - 52;
    int motion          = b[7];
    int seq_num         = b[8];
    int status          = b[9];

    char id_str[9];
    snprintf(id_str, sizeof(id_str), "%02x%02x%02x%02x", b[1], b[2], b[3], b[4]);

    /* clang-format off */
    data_t *data = data_make(
            "model",            "",             DATA_STRING, "Schrader-ATFPG4",
            "type",             "",             DATA_STRING, "TPMS",
            "id",               "",             DATA_STRING, id_str,
            "pressure_bar",     "Pressure",     DATA_FORMAT, "%.2f bar", DATA_DOUBLE, pressure_bar,
            "temperature_C",    "Temperature",  DATA_FORMAT, "%.0f C",   DATA_DOUBLE, (double)temperature_c,
            "seq_num",          "Sequence",     DATA_INT,    seq_num,
            "status",           "Status",       DATA_FORMAT, "%02x",     DATA_INT,    status,
            "motion",           "Motion",       DATA_FORMAT, "%02x",     DATA_INT,    motion,
            "mic",              "Integrity",    DATA_STRING, "CRC",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

/** @sa tpms_schrader_atfpg4_decode() */
static int tpms_schrader_atfpg4_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    // Raw sync before MC, the last preamble chips and the 4 bit start pattern
    uint8_t const preamble_pattern[] = {0xaa, 0x59};
    unsigned const preamble_bits     = sizeof(preamble_pattern) * 8;

    int ret    = 0;
    int events = 0;

    for (int row = 0; row < bitbuffer->num_rows; ++row) {
        unsigned bitpos = 0;
        // Find a sync with enough bits after it for a full MC coded message
        while ((bitpos = bitbuffer_search(bitbuffer, row, bitpos, preamble_pattern, preamble_bits)) + preamble_bits + ATFPG4_MSG_LEN * 16 <=
                bitbuffer->bits_per_row[row]) {
            ret = tpms_schrader_atfpg4_decode(decoder, bitbuffer, row, bitpos + preamble_bits);
            if (ret > 0) {
                events += ret;
                bitpos += preamble_bits + ATFPG4_MSG_LEN * 16;
            }
            else {
                bitpos += 2;
            }
        }
    }

    return events > 0 ? events : ret;
}

/** Encoder: inverse of tpms_schrader_atfpg4_decode().

    Patches id, pressure, temperature, motion, seq_num and status into the
    0x24 pressure message, recomputes the CRC-8 and re-encodes the inverted
    Manchester payload after every sync found in the captured bits (all frames
    of a transmission). If the capture holds no usable frame, a clean frame
    (preamble, sync, payload) is built from the decoded fields instead.
*/
static int tpms_schrader_atfpg4_encode(r_device const *decoder, struct capture *cap, bitbuffer_t *out_bits)
{
    uint8_t const sync[] = {0xaa, 0x59};
    unsigned const sync_bits = sizeof(sync) * 8;
    unsigned const mc_bits   = ATFPG4_MSG_LEN * 16;

    bitbuffer_t bits = cap->bits;
    uint8_t b[ATFPG4_MSG_LEN];
    int have_template = 0;

    // Locate every frame; the first valid one is the template for unedited bytes
    struct {
        unsigned row, pos;
    } frames[BITBUF_ROWS * 4];
    unsigned num_frames = 0;
    for (unsigned row = 0; row < bits.num_rows; ++row) {
        unsigned pos = 0;
        while ((pos = bitbuffer_search(&bits, row, pos, sync, sync_bits)) + sync_bits + mc_bits <= bits.bits_per_row[row]) {
            bitbuffer_t dec = {0};
            bitbuffer_manchester_decode(&bits, row, pos + sync_bits, &dec, ATFPG4_MSG_LEN * 8);
            if (dec.bits_per_row[0] >= ATFPG4_MSG_LEN * 8) {
                bitbuffer_invert(&dec);
                if (dec.bb[0][0] == 0x24 && num_frames < sizeof(frames) / sizeof(*frames)) {
                    if (!have_template) {
                        memcpy(b, dec.bb[0], ATFPG4_MSG_LEN);
                        have_template = 1;
                    }
                    frames[num_frames].row = row;
                    frames[num_frames].pos = pos + sync_bits;
                    num_frames++;
                    pos += sync_bits + mc_bits;
                    continue;
                }
            }
            pos += 2;
        }
    }

    if (!have_template) {
        // No captured frame: start from defaults seen on a parked truck
        uint8_t const defaults[ATFPG4_MSG_LEN] = {0x24, 0x5c, 0, 0, 0, 0, 52, 0x00, 0x00, 0xe4, 0};
        memcpy(b, defaults, sizeof(b));
        if (!capture_get_field(cap, "id"))
            return -1;
    }

    char const *f;
    int v;
    double d;
    if ((f = capture_get_field(cap, "id")) && *f) {
        uint32_t id = (uint32_t)strtoul(f, NULL, 16);
        b[1]        = (uint8_t)(id >> 24);
        b[2]        = (uint8_t)(id >> 16);
        b[3]        = (uint8_t)(id >> 8);
        b[4]        = (uint8_t)id;
    }
    if (encoder_field_dbl(cap, "pressure_bar", &d)) {
        long p = lround(d / 0.055);
        b[5]   = (uint8_t)(p < 0 ? 0 : p > 0xff ? 0xff : p);
        encoder_set_dbl(cap, "pressure_bar", b[5] * 0.055);
    }
    if (encoder_field_dbl(cap, "temperature_C", &d)) {
        long t = lround(d) + 52;
        b[6]   = (uint8_t)(t < 0 ? 0 : t > 0xff ? 0xff : t);
        encoder_set_dbl(cap, "temperature_C", b[6] - 52.0);
    }
    if (encoder_field_int(cap, "motion", &v))
        b[7] = (uint8_t)v;
    if (encoder_field_int(cap, "seq_num", &v))
        b[8] = (uint8_t)v;
    if (encoder_field_int(cap, "status", &v))
        b[9] = (uint8_t)v;
    b[10] = crc8(b, ATFPG4_MSG_LEN - 1, 0x2f, 0xaa);

    if (have_template) {
        for (unsigned i = 0; i < num_frames; ++i)
            encoder_manchester_encode(bits.bb[frames[i].row], frames[i].pos, b, ATFPG4_MSG_LEN * 8, 1);
    }
    else {
        // 64 preamble chips (0xaa..), the sync, the payload and a trailing chip pair
        memset(&bits, 0, sizeof(bits));
        uint8_t *row = bits.bb[0];
        memset(row, 0xaa, 8);
        memcpy(row + 8, sync, sizeof(sync));
        unsigned end = encoder_manchester_encode(row, 64 + sync_bits, b, ATFPG4_MSG_LEN * 8, 1);
        encoder_set_bit(row, end++, 1);
        encoder_set_bit(row, end++, 0);
        bits.bits_per_row[0] = (uint16_t)end;
        bits.num_rows        = 1;
    }

    *out_bits = bits;
    return encoder_verify(decoder, out_bits, cap) ? 1 : -1;
}

static char const *const output_fields[] = {
        "model",
        "type",
        "id",
        "pressure_bar",
        "temperature_C",
        "seq_num",
        "status",
        "motion",
        "mic",
        NULL,
};

r_device const tpms_schrader_atfpg4 = {
        .name        = "Schrader ATFPG4 heavy-duty truck TPMS",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 52,
        .long_width  = 52,
        .reset_limit = 800,
        .decode_fn   = &tpms_schrader_atfpg4_callback,
        .encode_fn   = &tpms_schrader_atfpg4_encode,
        .fields      = output_fields,
};
