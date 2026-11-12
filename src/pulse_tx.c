/** @file
    Pulse transmit: bits<->pulses encoding and pulse_data -> IQ modulation.

    Copyright (C) 2024 rtl_433_tx contributors

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

#include "pulse_tx.h"
#include "fatal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void tx_mod_params_defaults(tx_mod_params_t *p, uint32_t sample_rate, int is_fsk)
{
    memset(p, 0, sizeof(*p));
    p->sample_rate   = sample_rate;
    p->is_fsk        = is_fsk;
    p->fsk_dev_hz    = 25000.0; // +-25 kHz deviation, 50 kHz separation
    p->ook_offset_hz = 0.0;     // transmit OOK pulses at the carrier
    p->amplitude     = 0.85;
    p->repeats       = 3;       // real devices repeat each frame; improves reception of short bursts
    p->gap_us        = 10000.0; // 10 ms idle between repeats
    p->ramp_us       = 2.0;     // small ramp to reduce key-click splatter
}

// Append a pulse/gap pair (widths in samples) to a pulse_data_t.
static void pd_append(pulse_data_t *pulses, int pulse, int gap)
{
    if (pulses->num_pulses >= PD_MAX_PULSES) {
        return;
    }
    pulses->pulse[pulses->num_pulses] = pulse;
    pulses->gap[pulses->num_pulses]   = gap;
    pulses->num_pulses++;
}

// Encode one row of NRZ/RZ PCM: pulse = 1, gap = 0.
static void encode_pcm_row(uint8_t const *row, unsigned bit_len, int s_short, int s_long,
        pulse_data_t *pulses)
{
    int is_nrz   = (s_short == s_long);
    int bit_len_samp = s_long; // nominal bit period
    int cur_pulse = 0;
    int cur_gap   = 0;
    int in_gap    = 0;

    for (unsigned i = 0; i < bit_len; ++i) {
        int bit = (row[i / 8] >> (7 - (i % 8))) & 1;
        if (bit) {
            if (in_gap) {
                // close the previous pulse/gap pair and start a new pulse
                pd_append(pulses, cur_pulse, cur_gap);
                cur_pulse = 0;
                cur_gap   = 0;
                in_gap    = 0;
            }
            cur_pulse += is_nrz ? bit_len_samp : s_short;
            if (!is_nrz) {
                cur_gap = bit_len_samp - s_short; // RZ: pulse then return to zero
                in_gap  = 1;
            }
        }
        else {
            cur_gap += bit_len_samp;
            in_gap = 1;
        }
    }
    pd_append(pulses, cur_pulse, cur_gap);
}

// Encode one row of PPM: fixed pulse, short gap = 0, long gap = 1.
static void encode_ppm_row(uint8_t const *row, unsigned bit_len, int s_short, int s_long,
        int s_pulse, pulse_data_t *pulses)
{
    for (unsigned i = 0; i < bit_len; ++i) {
        int bit = (row[i / 8] >> (7 - (i % 8))) & 1;
        pd_append(pulses, s_pulse, bit ? s_long : s_short);
    }
}

// Encode one row of PWM: short pulse = 1, long pulse = 0, fixed gap.
static void encode_pwm_row(uint8_t const *row, unsigned bit_len, int s_short, int s_long,
        int s_gap, pulse_data_t *pulses)
{
    for (unsigned i = 0; i < bit_len; ++i) {
        int bit = (row[i / 8] >> (7 - (i % 8))) & 1;
        pd_append(pulses, bit ? s_short : s_long, s_gap);
    }
}

int pulse_tx_from_bits(bitbuffer_t const *bits, r_device const *device,
        pulse_data_t *pulses, uint32_t sample_rate)
{
    float samples_per_us = sample_rate / 1.0e6f;
    int s_short = (int)(device->short_width * samples_per_us + 0.5f);
    int s_long  = (int)(device->long_width * samples_per_us + 0.5f);
    int s_sync  = (int)(device->sync_width * samples_per_us + 0.5f);
    int s_reset = (int)(device->reset_limit * samples_per_us + 0.5f);
    int s_gap   = (int)(device->gap_limit * samples_per_us + 0.5f);
    if (s_long <= 0)
        s_long = s_short;
    if (s_short <= 0)
        return -1;

    pulse_data_clear(pulses);
    pulses->sample_rate = sample_rate;

    unsigned mod = device->modulation;
    // End-of-message silence: a gap >= reset_limit tells the slicer the frame is
    // over. Fall back to a wide gap when the device has no reset_limit.
    int reset_gap = s_reset > 0 ? s_reset : s_long * 8;
    // Inter-row separator: slicers start a new row (within the same bitbuffer)
    // for a gap wider than a data symbol but shorter than reset_limit. Decoders
    // routinely require several repeated rows, so rows must stay in one buffer
    // instead of each ending the message. Pick a value above the longest data
    // symbol / gap_limit yet safely below reset_limit.
    int row_floor = s_long > s_gap ? s_long : s_gap;
    int row_sep   = reset_gap;
    if (reset_gap > row_floor + 2)
        row_sep = (row_floor + reset_gap) / 2;
    // PPM encodes each bit in the gap *following* a pulse, so the final bit of a
    // row needs its own terminator pulse; stretching the last data gap would
    // otherwise swallow the final bit at the slicer.
    int is_ppm = (mod == OOK_PULSE_PPM);

    for (unsigned r = 0; r < bits->num_rows; ++r) {
        uint8_t const *row = bits->bb[r];
        unsigned bit_len   = bits->bits_per_row[r];
        if (bit_len == 0)
            continue;

        switch (mod) {
        case OOK_PULSE_PCM:
        case FSK_PULSE_PCM:
            encode_pcm_row(row, bit_len, s_short, s_long, pulses);
            break;
        case OOK_PULSE_PPM:
            encode_ppm_row(row, bit_len, s_short, s_long, s_short, pulses);
            break;
        case OOK_PULSE_PWM:
        case FSK_PULSE_PWM:
            encode_pwm_row(row, bit_len, s_short, s_long, s_sync > 0 ? s_short : s_short, pulses);
            break;
        default:
            return -1; // unsupported coding; caller should replay original pulses
        }

        if (pulses->num_pulses == 0)
            continue;

        int last_row = (r + 1 >= bits->num_rows);
        if (is_ppm) {
            // terminator pulse carries the row separator so the final data gap
            // keeps its short/long meaning. The whole-message reset is added
            // after the loop.
            pd_append(pulses, s_short, last_row ? 0 : row_sep);
        }
        else if (!last_row) {
            // separate rows without ending the message
            pulses->gap[pulses->num_pulses - 1] += row_sep;
        }
    }

    if (pulses->num_pulses == 0)
        return -1;
    // ensure a trailing reset gap so the receiver ends the message
    pulses->gap[pulses->num_pulses - 1] += reset_gap;
    return 0;
}

// Core IQ synthesizer shared by CU8/CS16 outputs. Writes normalized float I/Q
// into a callback-friendly loop. Returns number of samples, allocates fi/fq.
static int synth_iq(pulse_data_t const *pulses, tx_mod_params_t const *params,
        float **out_i, float **out_q)
{
    uint32_t rate = params->sample_rate;
    double src_rate = pulses->sample_rate ? pulses->sample_rate : rate;
    double scale = (double)rate / src_rate; // resample pulse widths to TX rate

    // compute total sample count
    size_t total = 0;
    for (unsigned n = 0; n < pulses->num_pulses; ++n) {
        total += (size_t)(pulses->pulse[n] * scale + 0.5);
        total += (size_t)(pulses->gap[n] * scale + 0.5);
    }
    if (total == 0)
        return -1;

    size_t gap_samps = (size_t)(params->gap_us * rate / 1e6 + 0.5);
    unsigned repeats = params->repeats ? params->repeats : 1;
    size_t guard = (size_t)(0.0003 * rate + 0.5); // ~0.3 ms silence around each frame
    size_t frame = total;
    size_t count = (size_t)(frame + 2 * guard) * repeats + gap_samps * (repeats > 0 ? repeats - 1 : 0);

    float *fi = malloc(count * sizeof(float));
    if (!fi)
        FATAL_MALLOC("synth_iq");
    float *fq = malloc(count * sizeof(float));
    if (!fq) {
        free(fi);
        FATAL_MALLOC("synth_iq");
    }

    double amp = params->amplitude;
    if (amp <= 0.0 || amp > 1.0)
        amp = 0.85;

    double phase = 0.0;
    double two_pi = 2.0 * M_PI;
    double mark_hz  = params->is_fsk ? params->fsk_dev_hz : params->ook_offset_hz;
    double space_hz = params->is_fsk ? -params->fsk_dev_hz : 0.0;
    size_t ramp = (size_t)(params->ramp_us * rate / 1e6 + 0.5);

    size_t idx = 0;
    for (unsigned rep = 0; rep < repeats; ++rep) {
        // lead-in silence so the receiver sees a clean rising edge
        for (size_t k = 0; k < guard && idx < count; ++k, ++idx) {
            fi[idx] = 0.0f;
            fq[idx] = 0.0f;
        }
        for (unsigned n = 0; n < pulses->num_pulses; ++n) {
            size_t plen = (size_t)(pulses->pulse[n] * scale + 0.5);
            size_t glen = (size_t)(pulses->gap[n] * scale + 0.5);
            // The final gap of the frame is the end-of-message reset: render it
            // as silence (carrier off) so the receiver detects packet end. For
            // FSK, interior gaps are the "space" frequency (carrier on).
            int last_gap = (n + 1 == pulses->num_pulses);

            // pulse segment: mark (FSK) or carrier-on (OOK)
            for (size_t k = 0; k < plen && idx < count; ++k, ++idx) {
                double env = amp;
                if (!params->is_fsk) {
                    if (ramp > 0 && k < ramp)
                        env *= (double)k / ramp;
                    if (ramp > 0 && k >= plen - ramp && plen > ramp)
                        env *= (double)(plen - 1 - k) / ramp;
                }
                phase += two_pi * mark_hz / rate;
                fi[idx] = (float)(env * cos(phase));
                fq[idx] = (float)(env * sin(phase));
            }
            // gap segment: space (FSK, carrier on) or carrier-off (OOK);
            // the end-of-message gap is always silence.
            for (size_t k = 0; k < glen && idx < count; ++k, ++idx) {
                if (params->is_fsk && !last_gap) {
                    phase += two_pi * space_hz / rate;
                    fi[idx] = (float)(amp * cos(phase));
                    fq[idx] = (float)(amp * sin(phase));
                }
                else {
                    fi[idx] = 0.0f;
                    fq[idx] = 0.0f;
                }
            }
        }
        // trailing silence
        for (size_t k = 0; k < guard && idx < count; ++k, ++idx) {
            fi[idx] = 0.0f;
            fq[idx] = 0.0f;
        }
        // idle gap between repeats
        if (rep + 1 < repeats) {
            for (size_t k = 0; k < gap_samps && idx < count; ++k, ++idx) {
                fi[idx] = 0.0f;
                fq[idx] = 0.0f;
            }
        }
    }

    *out_i = fi;
    *out_q = fq;
    return (int)idx;
}

int pulse_tx_to_cu8(pulse_data_t const *pulses, tx_mod_params_t const *params,
        uint8_t **out_buf, size_t *out_len)
{
    float *fi = NULL, *fq = NULL;
    int n = synth_iq(pulses, params, &fi, &fq);
    if (n <= 0)
        return -1;

    uint8_t *buf = malloc((size_t)n * 2);
    if (!buf) {
        free(fi);
        free(fq);
        FATAL_MALLOC("pulse_tx_to_cu8");
    }
    for (int i = 0; i < n; ++i) {
        int vi = (int)lround(fi[i] * 127.0) + 128;
        int vq = (int)lround(fq[i] * 127.0) + 128;
        buf[i * 2]     = (uint8_t)(vi < 0 ? 0 : vi > 255 ? 255 : vi);
        buf[i * 2 + 1] = (uint8_t)(vq < 0 ? 0 : vq > 255 ? 255 : vq);
    }
    free(fi);
    free(fq);
    *out_buf = buf;
    *out_len = (size_t)n * 2;
    return 0;
}

int pulse_tx_to_cs16(pulse_data_t const *pulses, tx_mod_params_t const *params,
        int16_t **out_buf, size_t *out_len)
{
    float *fi = NULL, *fq = NULL;
    int n = synth_iq(pulses, params, &fi, &fq);
    if (n <= 0)
        return -1;

    int16_t *buf = malloc((size_t)n * 2 * sizeof(int16_t));
    if (!buf) {
        free(fi);
        free(fq);
        FATAL_MALLOC("pulse_tx_to_cs16");
    }
    for (int i = 0; i < n; ++i) {
        int vi = (int)lround(fi[i] * 32767.0);
        int vq = (int)lround(fq[i] * 32767.0);
        buf[i * 2]     = (int16_t)(vi < -32768 ? -32768 : vi > 32767 ? 32767 : vi);
        buf[i * 2 + 1] = (int16_t)(vq < -32768 ? -32768 : vq > 32767 ? 32767 : vq);
    }
    free(fi);
    free(fq);
    *out_buf = buf;
    *out_len = (size_t)n * 2;
    return 0;
}

int pulse_tx_to_cf32(pulse_data_t const *pulses, tx_mod_params_t const *params,
        float **out_buf, size_t *out_len)
{
    float *fi = NULL, *fq = NULL;
    int n = synth_iq(pulses, params, &fi, &fq);
    if (n <= 0)
        return -1;

    float *buf = malloc((size_t)n * 2 * sizeof(float));
    if (!buf) {
        free(fi);
        free(fq);
        FATAL_MALLOC("pulse_tx_to_cf32");
    }
    for (int i = 0; i < n; ++i) {
        buf[i * 2]     = fi[i];
        buf[i * 2 + 1] = fq[i];
    }
    free(fi);
    free(fq);
    *out_buf = buf;
    *out_len = (size_t)n * 2;
    return 0;
}

int pulse_tx_write_cu8(char const *filename, uint8_t const *buf, size_t len)
{
    FILE *f = fopen(filename, "wb");
    if (!f)
        return -1;
    size_t w = fwrite(buf, 1, len, f);
    fclose(f);
    return w == len ? 0 : -1;
}

int pulse_tx_write_cs16(char const *filename, int16_t const *buf, size_t len)
{
    FILE *f = fopen(filename, "wb");
    if (!f)
        return -1;
    size_t w = fwrite(buf, sizeof(int16_t), len, f);
    fclose(f);
    return w == len ? 0 : -1;
}
