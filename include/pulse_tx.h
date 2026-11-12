/** @file
    Pulse transmit: bits<->pulses encoding and pulse_data -> IQ modulation.

    Copyright (C) 2024 rtl_433_tx contributors

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

#ifndef INCLUDE_PULSE_TX_H_
#define INCLUDE_PULSE_TX_H_

#include <stdint.h>
#include <stddef.h>
#include "pulse_data.h"
#include "bitbuffer.h"
#include "r_device.h"

/// Parameters controlling IQ synthesis for transmit.
typedef struct tx_mod_params {
    uint32_t sample_rate;   ///< Output IQ sample rate in Hz.
    int is_fsk;             ///< Non-zero for FSK, zero for OOK/ASK.
    double fsk_dev_hz;      ///< FSK frequency deviation in Hz (mark=+dev, space=-dev).
    double ook_offset_hz;   ///< Baseband tone offset for OOK pulses in Hz (0 = at carrier/DC).
    double amplitude;       ///< Peak amplitude 0..1 (relative to full scale).
    unsigned repeats;       ///< Number of times to repeat the frame.
    double gap_us;          ///< Idle gap in microseconds inserted between repeats.
    double ramp_us;         ///< Rise/fall ramp time in microseconds to limit spectral splatter.
} tx_mod_params_t;

/// Fill tx_mod_params_t with sensible defaults for the given modulation/rate.
void tx_mod_params_defaults(tx_mod_params_t *p, uint32_t sample_rate, int is_fsk);

/// Encode a bitbuffer into pulse_data according to a device's modulation/coding.
///
/// This is the inverse of the pulse_slicer_* functions. Supported codings:
/// OOK/FSK PCM (NRZ/RZ), PPM, PWM, Manchester-zerobit. Returns 0 on success,
/// negative on an unsupported modulation.
///
/// @param bits the raw bit rows to encode
/// @param device modulation parameters (short/long/gap/sync widths in us)
/// @param[out] pulses resulting pulse train (widths in samples at sample_rate)
/// @param sample_rate sample rate to express the pulse widths in
/// @return 0 on success, negative on error
int pulse_tx_from_bits(bitbuffer_t const *bits, r_device const *device,
        pulse_data_t *pulses, uint32_t sample_rate);

/// Measured symbol timing (us) overriding a device's nominal widths; 0 = use nominal.
typedef struct pulse_tx_timing {
    float pulse_us;   ///< PPM: fixed pulse width
    float short_us;   ///< PPM: short gap (0), PWM: short pulse (1)
    float long_us;    ///< PPM: long gap (1), PWM: long pulse (0)
    float gap_us;     ///< PWM: gap after a short pulse (1)
    float gap_long_us; ///< PWM: gap after a long pulse (0)
    float row_gap_us; ///< gap between repeated rows
} pulse_tx_timing_t;

/// Measure the actual symbol timing of a recorded pulse train for a device's
/// modulation (PPM and PWM only). Real transmitters often deviate a lot from a
/// decoder's nominal widths (e.g. PPM decoders only define the gaps), and
/// strict hardware receivers reject the nominal timing.
///
/// @param orig recorded pulse train (widths in samples at orig->sample_rate)
/// @param device modulation parameters used to classify the pulses/gaps
/// @param[out] timing measured widths, fields left 0 if not measurable
void pulse_tx_measure_timing(pulse_data_t const *orig, r_device const *device, pulse_tx_timing_t *timing);

/// Like pulse_tx_from_bits() but with optional measured timing overrides.
int pulse_tx_from_bits_timed(bitbuffer_t const *bits, r_device const *device,
        pulse_tx_timing_t const *timing, pulse_data_t *pulses, uint32_t sample_rate);

/// Synthesize interleaved IQ samples (CU8) from a pulse train.
///
/// @param pulses pulse train (widths in samples at pulses->sample_rate)
/// @param params modulation parameters
/// @param[out] out_buf newly allocated CU8 buffer (caller frees)
/// @param[out] out_len length of out_buf in bytes
/// @return 0 on success, negative on error
int pulse_tx_to_cu8(pulse_data_t const *pulses, tx_mod_params_t const *params,
        uint8_t **out_buf, size_t *out_len);

/// Synthesize interleaved IQ samples (CS16) from a pulse train.
int pulse_tx_to_cs16(pulse_data_t const *pulses, tx_mod_params_t const *params,
        int16_t **out_buf, size_t *out_len);

/// Synthesize interleaved IQ samples (CF32, native float) from a pulse train.
/// @param[out] out_len number of float elements (2 per sample)
int pulse_tx_to_cf32(pulse_data_t const *pulses, tx_mod_params_t const *params,
        float **out_buf, size_t *out_len);

/// Write a CU8 buffer to a file.
/// @return 0 on success, negative on error
int pulse_tx_write_cu8(char const *filename, uint8_t const *buf, size_t len);

/// Write a CS16 buffer to a file.
int pulse_tx_write_cs16(char const *filename, int16_t const *buf, size_t len);

#endif /* INCLUDE_PULSE_TX_H_ */
