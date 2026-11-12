/** @file
    Regression test for transmit timing fidelity.

    PPM and PWM decoders do not fully specify the on-air symbol timing (a PPM
    decoder only defines the gaps, not the pulse width; a PWM decoder only
    defines the pulses, not the gaps). Real sensors therefore use timing that
    differs a lot from a decoder's nominal widths, and strict hardware receivers
    reject the nominal timing. The transmit path measures the real timing from
    the recorded pulse train (pulse_tx_measure_timing) and feeds it back into
    the synthesizer (pulse_tx_from_bits_timed) so rebuilt frames keep the
    original timing.

    This test builds synthetic recordings whose real timing deliberately differs
    from the decoder nominal, and asserts that (1) the measured timing recovers
    the real widths and (2) the rebuilt pulse train honors them rather than
    falling back to the nominal decoder widths. It fails if a future change
    regresses the shared timing path, protecting every present and future
    PPM/PWM encoder.

    Copyright (C) 2024 rtl_433_tx contributors

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "pulse_tx.h"
#include "pulse_data.h"
#include "bitbuffer.h"
#include "r_device.h"

extern r_device const acurite_606;     // OOK_PULSE_PPM
extern r_device const emos_e6016_rain; // OOK_PULSE_PWM

static int failures = 0;

#define CHECK(cond, ...)                           \
    do {                                           \
        if (!(cond)) {                             \
            printf("FAIL: " __VA_ARGS__);          \
            printf("\n");                          \
            failures++;                            \
        }                                          \
    } while (0)

// 1 sample == 1 us, keeps the synthetic timing readable.
static uint32_t const RATE = 1000000;

static void pd_add(pulse_data_t *pd, int pulse_us, int gap_us)
{
    pd->pulse[pd->num_pulses] = pulse_us;
    pd->gap[pd->num_pulses]   = gap_us;
    pd->num_pulses++;
}

// Dominant pulse/gap width within [lo, hi) us, rounded to the nearest value.
static double dominant(int const *v, unsigned n, double lo, double hi)
{
    double sum = 0;
    unsigned k = 0;
    for (unsigned i = 0; i < n; ++i) {
        if (v[i] >= lo && v[i] < hi) {
            sum += v[i];
            k++;
        }
    }
    return k ? sum / k : 0;
}

static void test_ppm(void)
{
    // Real Acurite 606 pulses are ~500 us wide; the decoder nominal "short"
    // (the short gap) is 2000 us. Pre-fix the synthesizer used 2000 us for the
    // pulse width, which real consoles rejected.
    r_device const *dev = &acurite_606;
    double const real_pulse = 500.0;
    double const gap0       = dev->short_width; // 2000 us -> bit 0
    double const gap1       = dev->long_width;  // 4000 us -> bit 1

    // Bits to encode (and to lay out in the synthetic recording).
    uint8_t data = 0xA6; // 1010 0110
    uint8_t bits[1]      = {data};

    pulse_data_t rec = {0};
    rec.sample_rate = RATE;
    for (int i = 0; i < 8; ++i) {
        int bit = (data >> (7 - i)) & 1;
        pd_add(&rec, (int)real_pulse, bit ? (int)gap1 : (int)gap0);
    }
    // Trailing reset gap so the recording looks like one message.
    rec.gap[rec.num_pulses - 1] = (int)dev->reset_limit;

    pulse_tx_timing_t t;
    pulse_tx_measure_timing(&rec, dev, &t);
    CHECK(fabs(t.pulse_us - real_pulse) < 50.0,
            "PPM measured pulse_us=%.0f, expected ~%.0f", t.pulse_us, real_pulse);
    CHECK(fabs(t.short_us - gap0) < 100.0,
            "PPM measured short gap=%.0f, expected ~%.0f", t.short_us, gap0);
    CHECK(fabs(t.long_us - gap1) < 100.0,
            "PPM measured long gap=%.0f, expected ~%.0f", t.long_us, gap1);

    bitbuffer_t bb = {0};
    bb.num_rows        = 1;
    bb.bits_per_row[0] = 8;
    memcpy(bb.bb[0], bits, sizeof(bits));

    // With measured timing the rebuilt pulses must be the real width.
    pulse_data_t out = {0};
    int r = pulse_tx_from_bits_timed(&bb, dev, &t, &out, RATE);
    CHECK(r == 0, "PPM timed encode returned %d", r);
    double pw = dominant(out.pulse, out.num_pulses, 1, dev->reset_limit);
    CHECK(fabs(pw - real_pulse) < 50.0,
            "PPM rebuilt pulse width=%.0f, expected real ~%.0f (not nominal %.0f)",
            pw, real_pulse, (double)dev->short_width);

    // Without timing the synthesizer falls back to the nominal width; this
    // documents the difference the measured timing makes.
    pulse_data_t nom = {0};
    pulse_tx_from_bits_timed(&bb, dev, NULL, &nom, RATE);
    double pw_nom = dominant(nom.pulse, nom.num_pulses, 1, dev->reset_limit);
    CHECK(fabs(pw_nom - real_pulse) > 50.0,
            "PPM nominal width %.0f unexpectedly matched real %.0f", pw_nom, real_pulse);

    printf("PPM:  real_pulse=%.0f measured=%.0f rebuilt=%.0f nominal=%.0f\n",
            real_pulse, t.pulse_us, pw, pw_nom);
}

static void test_pwm(void)
{
    // PWM: bit 1 -> short pulse, bit 0 -> long pulse. Many PWM transmitters keep
    // a constant bit *period*, so the gap after a short pulse is long and vice
    // versa. The decoder does not define the gaps at all.
    r_device const *dev = &emos_e6016_rain;
    double const sp = dev->short_width; // 300 us short pulse (bit 1)
    double const lp = dev->long_width;  // 800 us long pulse (bit 0)
    double const gap_after_short = 700.0; // constant period ~1000 us
    double const gap_after_long  = 200.0;

    uint8_t data = 0xB4; // 1011 0100
    uint8_t bits[1] = {data};

    pulse_data_t rec = {0};
    rec.sample_rate = RATE;
    for (int i = 0; i < 8; ++i) {
        int bit = (data >> (7 - i)) & 1;
        pd_add(&rec, bit ? (int)sp : (int)lp, bit ? (int)gap_after_short : (int)gap_after_long);
    }
    rec.gap[rec.num_pulses - 1] = (int)dev->reset_limit;

    pulse_tx_timing_t t;
    pulse_tx_measure_timing(&rec, dev, &t);
    CHECK(fabs(t.gap_us - gap_after_short) < 60.0,
            "PWM measured gap after short=%.0f, expected ~%.0f", t.gap_us, gap_after_short);
    CHECK(fabs(t.gap_long_us - gap_after_long) < 60.0,
            "PWM measured gap after long=%.0f, expected ~%.0f", t.gap_long_us, gap_after_long);

    bitbuffer_t bb = {0};
    bb.num_rows        = 1;
    bb.bits_per_row[0] = 8;
    memcpy(bb.bb[0], bits, sizeof(bits));

    pulse_data_t out = {0};
    int r = pulse_tx_from_bits_timed(&bb, dev, &t, &out, RATE);
    CHECK(r == 0, "PWM timed encode returned %d", r);

    // Gap after short pulses vs after long pulses must track the measured,
    // per-symbol values, not a single nominal gap.
    double mid = (sp + lp) / 2;
    double g_after_short = 0, g_after_long = 0;
    unsigned ns = 0, nl = 0;
    for (unsigned i = 0; i < out.num_pulses; ++i) {
        if (out.gap[i] <= 0 || out.gap[i] >= dev->reset_limit)
            continue;
        if (out.pulse[i] < mid) { g_after_short += out.gap[i]; ns++; }
        else                    { g_after_long  += out.gap[i]; nl++; }
    }
    if (ns) g_after_short /= ns;
    if (nl) g_after_long  /= nl;
    CHECK(ns && fabs(g_after_short - gap_after_short) < 60.0,
            "PWM rebuilt gap after short=%.0f, expected ~%.0f", g_after_short, gap_after_short);
    CHECK(nl && fabs(g_after_long - gap_after_long) < 60.0,
            "PWM rebuilt gap after long=%.0f, expected ~%.0f", g_after_long, gap_after_long);
    CHECK(fabs(g_after_short - g_after_long) > 60.0,
            "PWM per-symbol gaps collapsed to a single value (%.0f)", g_after_short);

    printf("PWM:  gap_after_short=%.0f/%.0f gap_after_long=%.0f/%.0f\n",
            gap_after_short, g_after_short, gap_after_long, g_after_long);
}

int main(void)
{
    test_ppm();
    test_pwm();
    if (failures) {
        printf("tx-timing-test: %d failure(s)\n", failures);
        return 1;
    }
    printf("tx-timing-test: all checks passed\n");
    return 0;
}
