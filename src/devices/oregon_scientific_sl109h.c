/** @file
    Oregon Scientific SL109H decoder.
*/
/**
Oregon Scientific SL109H decoder.

Data layout (bits):

    AAAA CC HHHH HHHH TTTT TTTT TTTT SSSS IIII IIII

- A: 4 bit checksum (add)
- C: 2 bit channel code (1/2 are channels 1/2, 0 is channel 3, 3 is invalid)
- H: 8 bit BCD humidity
- T: 12 bit signed temperature scaled by 10
- S: 4 bit status, unknown
- I: 8 bit a random id that is generated when the sensor starts

S.a. http://www.osengr.org/WxShield/Downloads/OregonScientific-RF-Protocols-II.pdf


The device "Bresser Thermo-/Hygro-Sensor Explore Scientific ST1005H" works
with the same row length, but a completely different interpretation.
As such, if the bits align both decoders can misdetect data from the
other sensor as valid from their sensor with "plausable" but usually
completely wrong values.
*/

#include "decoder.h"
#include "capture.h"
#include "encoder.h"
#include <math.h>

static int oregon_scientific_sl109h_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    // A lone row that happens to pass the weak 4 bit checksum is a likely
    // false positive; require it to repeat identically at least once.
    // bitbuffer_find_repeated_row()'s min_bits is a >= threshold, not exact,
    // so a longer row (e.g. Esperanza EWS at 42 bits) must be excluded here.
    int row_index = bitbuffer_find_repeated_row(bitbuffer, 2, 38);
    if (row_index < 0 || bitbuffer->bits_per_row[row_index] != 38)
        return DECODE_ABORT_LENGTH;

    uint8_t *msg = bitbuffer->bb[row_index];
    uint8_t b[5];

    // check id channel temperature humidity value not zero
    if (!msg[0] && !msg[1] && !msg[2] && !msg[3]) {
        decoder_log(decoder, 2, __func__, "DECODE_FAIL_SANITY data all 0x00");
        return DECODE_FAIL_SANITY;
    }

    uint8_t chk = msg[0] >> 4;

    // align the channel "half nibble"
    bitbuffer_extract_bytes(bitbuffer, row_index, 2, b, 36);
    b[0] &= 0x3f;

    // Prevent false positives from 'allzero'
    // reject if Checksum channelhumidity and temperature are all zero
    if (chk == 0 && b[0] == 0 && b[1] == 0 && b[2] == 0)
        return DECODE_FAIL_SANITY;

    uint8_t sum = add_nibbles(b, 5) & 0xf;
    if (sum != chk) {
        decoder_logf_bitbuffer(decoder, 2, __func__, bitbuffer, "Checksum error. Expected: %01x Calculated: %01x", chk, sum);
        return DECODE_FAIL_MIC;
    }

    int channel_code = b[0] >> 4;
    if (channel_code == 3) {
        decoder_log(decoder, 2, __func__, "invalid channel code");
        return DECODE_FAIL_SANITY;
    }
    int channel = channel_code ? channel_code : 3;

    // BCD humidity: reject invalid digits instead of reporting e.g. 150%.
    int hum_tens = b[0] & 0x0f;
    int hum_ones = b[1] >> 4;
    if (hum_tens > 9 || hum_ones > 9) {
        decoder_logf(decoder, 2, __func__, "invalid BCD humidity nibble: %x %x", hum_tens, hum_ones);
        return DECODE_FAIL_SANITY;
    }
    int humidity = 10 * hum_tens + hum_ones;

    int temp_raw = (int16_t)((b[1] & 0x0f) << 12) | (b[2] << 4); // uses sign-extend
    float temp_c = (temp_raw >> 4) * 0.1f;

    // reduce false positives by checking specified sensor range, this isn't great...
    if (temp_c < -20 || temp_c > 60) {
        decoder_logf(decoder, 2, __func__, "temperature sanity check failed: %.1f C", temp_c);
        return DECODE_FAIL_SANITY;
    }

    // there may be more specific information here; not currently certain what information is encoded here
    int status = (b[3] >> 4);

    // changes when thermometer reset button is pushed/battery is changed
    uint8_t id = ((b[3] & 0x0f) << 4) | (b[4] >> 4);

    /* clang-format off */
    data_t *data = data_make(
            "model",            "Model",                                DATA_STRING, "Oregon-SL109H",
            "id",               "Id",                                   DATA_INT,    id,
            "channel",          "Channel",                              DATA_INT,    channel,
            "temperature_C",    "Celsius",      DATA_FORMAT, "%.1f C",  DATA_DOUBLE, temp_c,
            "humidity",         "Humidity",     DATA_FORMAT, "%u %%",   DATA_INT,    humidity,
            "status",           "Status",                               DATA_INT,    status,
            "mic",              "Integrity",                            DATA_STRING, "CHECKSUM",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

/** Encoder: inverse of oregon_scientific_sl109h_callback(). 38-bit frame; the
    checksum nibble AAAA leads at bits 0-3 and the 36-bit payload is read from
    bit 2, so bits 2-3 are shared (masked off in the sum). */
static int oregon_scientific_sl109h_encode(r_device const *decoder, struct capture *cap, bitbuffer_t *out_bits)
{
    uint8_t raw[5];
    if (encoder_template_rows(cap, 38, raw, out_bits) < 0)
        return -1;

    int v;
    double t;
    // decode defaults derived from the captured template (payload from bit 2)
    uint8_t b[5] = {0};
    for (int i = 0; i < 36; ++i)
        if ((raw[(i + 2) / 8] >> (7 - (i + 2) % 8)) & 1)
            b[i / 8] |= (uint8_t)(0x80 >> (i % 8));
    b[0] &= 0x3f; // clear the shared checksum bits

    int channel = ((b[0] >> 4) == 0) ? 3 : (b[0] >> 4);
    int hum     = (b[0] & 0x0f) * 10 + (b[1] >> 4);
    int status  = b[3] >> 4;
    int id      = ((b[3] & 0x0f) << 4) | (b[4] >> 4);

    if (encoder_field_int(cap, "channel", &v))
        channel = v;
    if (encoder_field_int(cap, "humidity", &v))
        hum = v;
    if (encoder_field_int(cap, "status", &v))
        status = v;
    if (encoder_field_int(cap, "id", &v))
        id = v;
    int temp12;
    if (encoder_field_dbl(cap, "temperature_C", &t))
        temp12 = (int)lround(t * 10.0) & 0xfff;
    else
        temp12 = (((b[1] & 0x0f) << 8) | b[2]) & 0xfff;

    int ch_code = (channel == 3) ? 0 : channel;
    b[0] = (uint8_t)((ch_code << 4) | ((hum / 10) & 0x0f));
    b[1] = (uint8_t)(((hum % 10) << 4) | ((temp12 >> 8) & 0x0f));
    b[2] = (uint8_t)(temp12 & 0xff);
    b[3] = (uint8_t)(((status & 0x0f) << 4) | ((id >> 4) & 0x0f));
    b[4] = (uint8_t)((id & 0x0f) << 4);

    int chk = add_nibbles(b, 5) & 0x0f;

    // write the 36-bit payload at bit offset 2, then overlay the AAAA nibble
    for (int i = 0; i < 36; ++i)
        encoder_set_bit(raw, (unsigned)(i + 2), (b[i / 8] >> (7 - i % 8)) & 1);
    for (int i = 0; i < 4; ++i)
        encoder_set_bit(raw, (unsigned)i, (chk >> (3 - i)) & 1);

    encoder_apply_rows(out_bits, 38, raw);
    return encoder_verify(decoder, out_bits, cap) ? 1 : -1;
}

static char const *const output_fields[] = {
        "model",
        "id",
        "channel",
        "status",
        "temperature_C",
        "humidity",
        "mic",
        NULL,
};

r_device const oregon_scientific_sl109h = {
        .name        = "Oregon Scientific SL109H Remote Thermal Hygro Sensor",
        .modulation  = OOK_PULSE_PPM,
        .short_width = 2000,
        .long_width  = 4000,
        .gap_limit   = 5000,
        .reset_limit = 10000, // packet gap is 8900
        .decode_fn   = &oregon_scientific_sl109h_callback,
        .encode_fn   = &oregon_scientific_sl109h_encode,
        .fields      = output_fields,
};
