/** @file
    Encoder framework implementation.

    Copyright (C) 2024 rtl_433_tx contributors

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

#include "r_encoder.h"
#include "pulse_tx.h"
#include "decoder_util.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static void encoder_sink_fn(r_device *dev, data_t *data)
{
    capture_t *out = dev->output_ctx;
    capture_fields_from_data(out, data);
}

int encoder_decode_once(r_device const *dev, bitbuffer_t *bits, capture_t *out)
{
    // cast away const: we temporarily borrow the decoder to run it
    r_device *d = (r_device *)dev;
    void (*save_out)(r_device *, data_t *) = d->output_fn;
    void *save_ctx = d->output_ctx;

    memset(out, 0, sizeof(*out));
    d->output_fn  = encoder_sink_fn;
    d->output_ctx = out;

    int r = d->decode_fn ? d->decode_fn(d, bits) : 0;

    d->output_fn  = save_out;
    d->output_ctx = save_ctx;
    return r;
}

int encoder_verify(r_device const *dev, bitbuffer_t *bits, capture_t const *want)
{
    bitbuffer_t copy = *bits; // decode may clear the buffer
    capture_t got = {0};
    int events = encoder_decode_once(dev, &copy, &got);
    int ok = (events > 0);
    if (ok) {
        for (unsigned i = 0; i < want->num_fields && ok; ++i) {
            char const *key = want->fields[i].key;
            // only check the meaningful value fields
            if (!strcmp(key, "mic") || !strcmp(key, "type") || !strcmp(key, "model"))
                continue;
            char const *gv = capture_get_field(&got, key);
            if (!gv) {
                ok = 0;
                break;
            }
            char const *wv = want->fields[i].val;
            char *we = NULL, *ge = NULL;
            double wd = strtod(wv, &we);
            double gd = strtod(gv, &ge);
            if (we != wv && ge != gv && *we == '\0' && *ge == '\0') {
                if (fabs(wd - gd) > 0.01)
                    ok = 0;
            }
            else if (strcmp(wv, gv) != 0) {
                ok = 0;
            }
        }
    }
    capture_clear(&got);
    return ok;
}

r_device *capture_find_device(list_t *r_devs, capture_t const *cap)
{
    r_device *by_name = NULL;
    for (void **iter = r_devs->elems; iter && *iter; ++iter) {
        r_device *dev = *iter;
        if (cap->protocol && dev->protocol_num == (unsigned)cap->protocol)
            return dev;
        if (cap->model && dev->name && strcmp(dev->name, cap->model) == 0)
            by_name = dev;
    }
    return by_name;
}

int capture_pulses_from_bits(capture_t *cap, r_device const *dev)
{
    uint32_t rate = cap->sample_rate ? cap->sample_rate : 1000000;
    int r = pulse_tx_from_bits(&cap->bits, dev, &cap->pulses, rate);
    if (r == 0 && cap->is_fsk) {
        cap->pulses.fsk_f1_est = 1;
        cap->pulses.fsk_f2_est = 1;
    }
    return r;
}

int capture_rebuild(capture_t *cap, r_device const *dev)
{
    if (dev && dev->encode_fn) {
        bitbuffer_t bits = {0};
        int r = dev->encode_fn(dev, cap, &bits);
        if (r < 0)
            return r;
        cap->bits = bits;
        cap->modulation_num = (int)dev->modulation;
        cap->is_fsk = (dev->modulation >= FSK_DEMOD_MIN_VAL && dev->modulation < LORA_DEMOD_MIN_VAL);
        if (capture_pulses_from_bits(cap, dev) < 0)
            return -1;
        return 0;
    }

    // generic: regenerate pulses from existing (possibly edited) raw bits
    if (dev)
        capture_pulses_from_bits(cap, dev);
    return 1;
}

int encoder_field_int(capture_t const *cap, char const *key, int *out)
{
    char const *f = capture_get_field(cap, key);
    if (!f || !*f)
        return 0;
    *out = (int)strtol(f, NULL, 10);
    return 1;
}

int encoder_field_dbl(capture_t const *cap, char const *key, double *out)
{
    char const *f = capture_get_field(cap, key);
    if (!f || !*f)
        return 0;
    *out = atof(f);
    return 1;
}

void encoder_set_dbl(capture_t *cap, char const *key, double val)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "%.4f", val);
    capture_set_field(cap, key, buf);
}

static int encoder_is_payload_row(unsigned len, unsigned nbits)
{
    return len + 1 >= nbits && len <= nbits + 2;
}

int encoder_template_rows(capture_t const *cap, unsigned nbits, uint8_t *base, bitbuffer_t *out)
{
    *out = cap->bits;
    // prefer the most repeated full-length row, else any full-length row
    bitbuffer_t tmp = cap->bits;
    int r = bitbuffer_find_repeated_row(&tmp, 1, nbits);
    if (r >= 0 && tmp.bits_per_row[r] > nbits + 2)
        r = -1;
    for (unsigned i = 0; r < 0 && i < cap->bits.num_rows; ++i) {
        if (cap->bits.bits_per_row[i] >= nbits && cap->bits.bits_per_row[i] <= nbits + 2)
            r = (int)i;
    }
    if (r < 0)
        return -1;
    memset(base, 0, (nbits + 7) / 8);
    bitbuffer_extract_bytes(out, (unsigned)r, 0, base, nbits);
    return 0;
}

void encoder_apply_rows(bitbuffer_t *out, unsigned nbits, uint8_t const *payload)
{
    for (unsigned i = 0; i < out->num_rows; ++i) {
        unsigned len = out->bits_per_row[i];
        if (!encoder_is_payload_row(len, nbits))
            continue;
        uint8_t *row = out->bb[i];
        if (len < nbits) {
            out->bits_per_row[i] = (uint16_t)nbits;
        }
        for (unsigned k = 0; k < nbits; ++k) {
            uint8_t mask = (uint8_t)(0x80 >> (k % 8));
            if (payload[k / 8] & mask)
                row[k / 8] |= mask;
            else
                row[k / 8] &= (uint8_t)~mask;
        }
        if (len < nbits) {
            // clear any stale bits after the restored payload
            for (unsigned k = nbits; k < (unsigned)((nbits + 7) / 8) * 8; ++k)
                row[k / 8] &= (uint8_t)~(0x80 >> (k % 8));
        }
    }
}

void encoder_set_bit(uint8_t *row, unsigned idx, int val)
{
    uint8_t mask = (uint8_t)(0x80 >> (idx % 8));
    if (val)
        row[idx / 8] |= mask;
    else
        row[idx / 8] &= (uint8_t)~mask;
}

unsigned encoder_manchester_encode(uint8_t *row, unsigned start, uint8_t const *data, unsigned nbits, int invert)
{
    for (unsigned i = 0; i < nbits; ++i) {
        int d  = (data[i / 8] >> (7 - (i % 8))) & 1;
        int b1 = invert ? d : (1 - d);
        int b2 = invert ? (1 - d) : d;
        encoder_set_bit(row, start++, b1);
        encoder_set_bit(row, start++, b2);
    }
    return start;
}

unsigned encoder_diff_manchester_encode(uint8_t *row, unsigned start, int *prev, uint8_t const *data, unsigned nbits)
{
    int p = *prev;
    for (unsigned i = 0; i < nbits; ++i) {
        int d  = (data[i / 8] >> (7 - (i % 8))) & 1;
        int h1 = 1 - p;
        int h2 = d ? h1 : (1 - h1);
        encoder_set_bit(row, start++, h1);
        encoder_set_bit(row, start++, h2);
        p = h2;
    }
    *prev = p;
    return start;
}
