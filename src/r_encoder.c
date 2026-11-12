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
