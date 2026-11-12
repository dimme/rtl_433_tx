/** @file
    Capture file format: store a received transmission (raw pulses, raw bits,
    and decoded values) so it can be inspected, edited, and re-transmitted.

    Copyright (C) 2024 rtl_433_tx contributors

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

#ifndef INCLUDE_CAPTURE_H_
#define INCLUDE_CAPTURE_H_

#include <stdint.h>
#include <stdio.h>
#include "pulse_data.h"
#include "bitbuffer.h"
#include "r_device.h"
#include "data.h"

/// A single decoded field, stored as text for human editing.
typedef struct capture_field {
    char *key;
    char *val;      ///< value rendered as text (numbers verbatim)
    int is_string;  ///< non-zero if the JSON value should be quoted
} capture_field_t;

/// A captured transmission.
typedef struct capture {
    char *time;
    int protocol;          ///< protocol number
    char *model;           ///< decoder/device name
    char *modulation;      ///< modulation name, e.g. "FSK_PULSE_PCM"
    int modulation_num;    ///< numeric modulation (enum modulation_types)
    uint32_t freq_hz;      ///< center frequency in Hz
    uint32_t sample_rate;  ///< capture sample rate in Hz
    double short_us, long_us, reset_us, gap_us, sync_us, tolerance_us;
    int is_fsk;
    double fsk_f1_hz, fsk_f2_hz;

    pulse_data_t pulses;   ///< raw pulse train (widths in samples at sample_rate)
    bitbuffer_t bits;      ///< raw demodulated bits

    capture_field_t *fields;
    unsigned num_fields;
} capture_t;

/// Release all memory owned by a capture (does not free the capture itself).
void capture_clear(capture_t *cap);

/// Populate a capture from a decode event. @p data may be NULL.
void capture_from_decode(capture_t *cap, r_device const *dev,
        pulse_data_t const *pulses, bitbuffer_t const *bits, data_t const *data,
        uint32_t freq_hz);

/// Append decoded fields from a data_t list to a capture.
void capture_fields_from_data(capture_t *cap, data_t const *data);

/// Serialize a capture as a single-line JSON object followed by newline.
int capture_write_json(FILE *f, capture_t const *cap);

/// Parse one capture from a JSON object string.
/// @return 0 on success, negative on error.
int capture_parse_json(capture_t *cap, char const *json, size_t len);

/// Read the next capture (one JSON object per line) from a file.
/// @return 0 on success, 1 on EOF, negative on error.
int capture_read_next(FILE *f, capture_t *cap);

/// Look up a decoded field value by key, or NULL if absent.
char const *capture_get_field(capture_t const *cap, char const *key);

/// Set (or add) a decoded field value by key.
void capture_set_field(capture_t *cap, char const *key, char const *val);

#endif /* INCLUDE_CAPTURE_H_ */
