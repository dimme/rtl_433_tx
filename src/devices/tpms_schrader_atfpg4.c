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
- F1 : 0x00 on a parked truck, other values seen when driving, unknown
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
    int flags1          = b[7];
    int flags2          = b[8];
    int flags3          = b[9];

    char id_str[9];
    snprintf(id_str, sizeof(id_str), "%02x%02x%02x%02x", b[1], b[2], b[3], b[4]);

    /* clang-format off */
    data_t *data = data_make(
            "model",            "",             DATA_STRING, "Schrader-ATFPG4",
            "type",             "",             DATA_STRING, "TPMS",
            "id",               "",             DATA_STRING, id_str,
            "pressure_bar",     "Pressure",     DATA_FORMAT, "%.2f bar", DATA_DOUBLE, pressure_bar,
            "temperature_C",    "Temperature",  DATA_FORMAT, "%.0f C",   DATA_DOUBLE, (double)temperature_c,
            "flags1",           "Flags1",       DATA_FORMAT, "%02x",     DATA_INT,    flags1,
            "flags2",           "Flags2",       DATA_FORMAT, "%02x",     DATA_INT,    flags2,
            "flags3",           "Flags3",       DATA_FORMAT, "%02x",     DATA_INT,    flags3,
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

static char const *const output_fields[] = {
        "model",
        "type",
        "id",
        "pressure_bar",
        "temperature_C",
        "flags1",
        "flags2",
        "flags3",
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
        .fields      = output_fields,
};
