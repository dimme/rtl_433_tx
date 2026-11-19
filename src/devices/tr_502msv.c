/** @file
    TR-502MSV remote controller for RC-710DX.

    Copyright (C) 2024 Filip Kosecek <filip.kosecek@gmail.com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

#include "decoder.h"
#include "capture.h"
#include "encoder.h"
#include <string.h>

/**
TR-502MSV remote controller for RC-710DX.

21-bit data packet, repeated up to 4 times:

    PIIIIIII IIIIISSS OCRUU

- P: 1-bit preamble, always 1
- I: 12-bit device id
- S: 3-bit socket id
- O: 1-bit on/off
- C: 1-bit command, brightness/switch
- R: 1-bit reserved, always 0
- U: 2-bit checksum

Socket ids map to sockets as follows:

| Socket | Socket id |
| ------ | --------- |
|   1    |    000    |
|   2    |    100    |
|   3    |    010    |
|   4    |    110    |
|  all   |    111    |

The 2-bit checksum U1U0 is computed from the on/off bit (O), the command
bit (C), and the socket id bits S2S1S0 (msb to lsb) as:

    U1 = C ^ S2 ^ S0
    U0 = O ^ S1
*/
static int tr502msv_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    char const *commands[] = {"OFF", "BRIGHT", "ON", "DIM"};
    char const *sockets[]  = {"1", "3", "2", "4", "ALL"};

    if (bitbuffer->num_rows != 1 || bitbuffer->bits_per_row[0] != 21) {
        return DECODE_ABORT_LENGTH;
    }

    uint8_t *b = bitbuffer->bb[0];

    if ((b[0] & 0x80) == 0) { // preamble bit
        return DECODE_ABORT_EARLY;
    }
    if ((b[2] & 0x20) != 0) { // reserved bit
        return DECODE_FAIL_SANITY;
    }

    int device_id = ((b[0] & 0x7f) << 5) | (b[1] >> 3);
    int socket_id = b[1] & 0x07;
    int on_off    = (b[2] & 0x80) >> 7;
    int command   = (b[2] & 0x40) >> 6;
    int chk1      = (b[2] & 0x10) >> 4;
    int chk0      = (b[2] & 0x08) >> 3;

    int s2 = (socket_id >> 2) & 1;
    int s1 = (socket_id >> 1) & 1;
    int s0 = socket_id & 1;
    if (chk1 != (command ^ s2 ^ s0) || chk0 != (on_off ^ s1)) {
        return DECODE_FAIL_MIC;
    }

    char const *socket_str;
    if (socket_id % 2 == 0) {
        socket_str = sockets[socket_id >> 1];
    } else if (socket_id == 0x7) {
        socket_str = sockets[4];
    } else {
        return DECODE_FAIL_SANITY;
    }

    char const *command_str = commands[(on_off << 1) | command];

    /* clang-format off */
    data_t *data = data_make(
            "model",        "Model",        DATA_STRING, "TR-502MSV",
            "id",           "Device ID",    DATA_FORMAT, "%u", DATA_INT, device_id,
            "socket_id",    "Socket",       DATA_STRING, socket_str,
            "command",      "Command",      DATA_STRING, command_str,
            "mic",          "Integrity",    DATA_STRING, "CHECKSUM",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

/** Encoder: inverse of tr502msv_decode(). 21-bit frame, 2-bit checksum. */
static int tr502msv_encode(r_device const *decoder, struct capture *cap, bitbuffer_t *out_bits)
{
    uint8_t b[3];
    if (encoder_template_rows(cap, 21, b, out_bits) < 0)
        return -1;

    // defaults from the captured template
    int device_id = ((b[0] & 0x7f) << 5) | (b[1] >> 3);
    int socket_id = b[1] & 0x07;
    int on_off    = (b[2] & 0x80) >> 7;
    int command   = (b[2] & 0x40) >> 6;

    int v;
    if (encoder_field_int(cap, "id", &v))
        device_id = v & 0xfff;
    char const *s;
    if ((s = capture_get_field(cap, "socket_id"))) {
        if (!strcmp(s, "1")) socket_id = 0;
        else if (!strcmp(s, "2")) socket_id = 4;
        else if (!strcmp(s, "3")) socket_id = 2;
        else if (!strcmp(s, "4")) socket_id = 6;
        else if (!strcmp(s, "ALL")) socket_id = 7;
    }
    if ((s = capture_get_field(cap, "command"))) {
        if (!strcmp(s, "OFF")) { on_off = 0; command = 0; }
        else if (!strcmp(s, "BRIGHT")) { on_off = 0; command = 1; }
        else if (!strcmp(s, "ON")) { on_off = 1; command = 0; }
        else if (!strcmp(s, "DIM")) { on_off = 1; command = 1; }
    }

    int s2 = (socket_id >> 2) & 1, s1 = (socket_id >> 1) & 1, s0 = socket_id & 1;
    int chk1 = command ^ s2 ^ s0;
    int chk0 = on_off ^ s1;

    b[0] = (uint8_t)(0x80 | ((device_id >> 5) & 0x7f)); // preamble + id high
    b[1] = (uint8_t)(((device_id & 0x1f) << 3) | (socket_id & 0x07));
    b[2] = (uint8_t)((on_off << 7) | (command << 6) | (chk1 << 4) | (chk0 << 3));

    encoder_apply_rows(out_bits, 21, b);
    return encoder_verify(decoder, out_bits, cap) ? 1 : -1;
}

static char const *const output_fields[] = {
        "model",
        "id",
        "socket_id",
        "command",
        "mic",
        NULL,
};

r_device const tr_502msv = {
        .name        = "TR-502MSV remote smart socket controller",
        .modulation  = OOK_PULSE_PWM,
        .short_width = 740,
        .long_width  = 1400,
        .tolerance   = 70,
        .reset_limit = 84000,
        .decode_fn   = &tr502msv_decode,
        .encode_fn   = &tr502msv_encode,
        .fields      = output_fields,
};
