/** @file
    SDR transmit via SoapySDR (e.g. HackRF One).

    Copyright (C) 2024 rtl_433_tx contributors

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

#include "sdr_tx.h"
#include "logger.h"
#include <stdio.h>
#include <string.h>

#ifdef SOAPYSDR
#include <SoapySDR/Version.h>
#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>

int sdr_tx_cf32(sdr_tx_params_t const *p, float const *iq, size_t num_elems)
{
    SoapySDRDevice *dev = SoapySDRDevice_makeStrArgs(p->dev_query ? p->dev_query : "");
    if (!dev) {
        print_logf(LOG_ERROR, __func__, "Unable to open TX device: %s", SoapySDRDevice_lastError());
        return -1;
    }

    int r = 0;
    r |= SoapySDRDevice_setSampleRate(dev, SOAPY_SDR_TX, 0, (double)p->sample_rate);
    r |= SoapySDRDevice_setFrequency(dev, SOAPY_SDR_TX, 0, (double)p->center_freq, NULL);
    if (p->antenna)
        SoapySDRDevice_setAntenna(dev, SOAPY_SDR_TX, 0, p->antenna);
    if (SoapySDRDevice_hasGainMode(dev, SOAPY_SDR_TX, 0))
        SoapySDRDevice_setGainMode(dev, SOAPY_SDR_TX, 0, 0); // manual gain
    SoapySDRDevice_setGain(dev, SOAPY_SDR_TX, 0, p->gain);
    if (r != 0) {
        print_logf(LOG_ERROR, __func__, "Failed to configure TX device: %s", SoapySDRDevice_lastError());
        SoapySDRDevice_unmake(dev);
        return -1;
    }

    if (p->verbose) {
        print_logf(LOG_NOTICE, __func__, "TX %u Hz, %u S/s, gain %.0f dB, %zu samples",
                p->center_freq, p->sample_rate, p->gain, num_elems / 2);
    }

    SoapySDRStream *stream = NULL;
#if SOAPY_SDR_API_VERSION >= 0x00080000
    stream = SoapySDRDevice_setupStream(dev, SOAPY_SDR_TX, SOAPY_SDR_CF32, NULL, 0, NULL);
    if (!stream) {
        print_logf(LOG_ERROR, __func__, "Failed to set up TX stream: %s", SoapySDRDevice_lastError());
        SoapySDRDevice_unmake(dev);
        return -1;
    }
#else
    if (SoapySDRDevice_setupStream(dev, &stream, SOAPY_SDR_TX, SOAPY_SDR_CF32, NULL, 0, NULL) != 0) {
        print_logf(LOG_ERROR, __func__, "Failed to set up TX stream: %s", SoapySDRDevice_lastError());
        SoapySDRDevice_unmake(dev);
        return -1;
    }
#endif

    SoapySDRDevice_activateStream(dev, stream, 0, 0, 0);

    size_t mtu = SoapySDRDevice_getStreamMTU(dev, stream);
    if (mtu == 0)
        mtu = 4096;

    size_t total_samples = num_elems / 2;
    size_t sent = 0;
    int ret = 0;
    while (sent < total_samples) {
        size_t chunk = total_samples - sent;
        if (chunk > mtu)
            chunk = mtu;
        void const *buffs[1];
        buffs[0] = iq + sent * 2;
        int flags = 0;
        long long time_ns = 0;
        int w = SoapySDRDevice_writeStream(dev, stream, buffs, chunk, &flags, time_ns, 1000000);
        if (w < 0) {
            print_logf(LOG_ERROR, __func__, "writeStream error: %d", w);
            ret = -1;
            break;
        }
        sent += (size_t)w;
    }

    // allow the hardware to drain the buffer before deactivating
    int flags = SOAPY_SDR_END_BURST;
    long long time_ns = 0;
    SoapySDRDevice_writeStream(dev, stream, (void const *const *)&iq, 0, &flags, time_ns, 100000);

    SoapySDRDevice_deactivateStream(dev, stream, 0, 0);
    SoapySDRDevice_closeStream(dev, stream);
    SoapySDRDevice_unmake(dev);
    return ret;
}

#else // !SOAPYSDR

int sdr_tx_cf32(sdr_tx_params_t const *p, float const *iq, size_t num_elems)
{
    (void)p;
    (void)iq;
    (void)num_elems;
    print_log(LOG_ERROR, __func__, "SoapySDR support not compiled in; cannot transmit. Use an IQ output file instead.");
    return -1;
}

#endif
