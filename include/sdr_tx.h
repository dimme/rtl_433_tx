/** @file
    SDR transmit via SoapySDR (e.g. HackRF One).

    Copyright (C) 2024 rtl_433_tx contributors

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

#ifndef INCLUDE_SDR_TX_H_
#define INCLUDE_SDR_TX_H_

#include <stdint.h>
#include <stddef.h>

/// Transmit parameters for sdr_tx_cf32().
typedef struct sdr_tx_params {
    char const *dev_query;   ///< SoapySDR device selection string (e.g. "driver=hackrf").
    uint32_t sample_rate;    ///< TX sample rate in Hz.
    uint32_t center_freq;    ///< TX center frequency in Hz.
    double gain;             ///< Overall TX gain in dB (driver dependent).
    char const *antenna;     ///< Optional antenna name (NULL for default).
    int verbose;
} sdr_tx_params_t;

/// Transmit an interleaved CF32 (float I/Q) buffer once.
///
/// @param p transmit parameters
/// @param iq interleaved float I/Q samples
/// @param num_elems number of float elements in @p iq (2 per sample)
/// @return 0 on success, negative on error
int sdr_tx_cf32(sdr_tx_params_t const *p, float const *iq, size_t num_elems);

#endif /* INCLUDE_SDR_TX_H_ */
