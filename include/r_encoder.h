/** @file
    Encoder framework: rebuild raw bits/pulses from (edited) capture fields.

    Copyright (C) 2024 rtl_433_tx contributors

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

#ifndef INCLUDE_R_ENCODER_H_
#define INCLUDE_R_ENCODER_H_

#include "capture.h"
#include "r_device.h"
#include "list.h"

/// Find the decoder/device matching a capture (by protocol number, then name).
r_device *capture_find_device(list_t *r_devs, capture_t const *cap);

/// Regenerate the pulse train of a capture from its raw bits using the
/// device modulation parameters. Used for generic bit-level edits/replay.
/// @return 0 on success, negative on unsupported modulation.
int capture_pulses_from_bits(capture_t *cap, r_device const *dev);

/// Rebuild a capture after decoded fields were edited.
///
/// If the device provides an encode_fn it is used to rebuild the raw bits from
/// the edited decoded fields, then the pulse train is regenerated. Otherwise
/// the raw bits are left as-is and only the pulse train is regenerated.
///
/// @return 0 encoded via device encoder, 1 generic (no encoder), negative on error.
int capture_rebuild(capture_t *cap, r_device const *dev);

/// Decode a bitbuffer once with the device decoder, capturing output fields
/// into @p out (fields only). Does not emit to normal outputs.
/// @return number of decode events (>0 success).
int encoder_decode_once(r_device const *dev, bitbuffer_t *bits, capture_t *out);

/// Verify that decoding @p bits reproduces the field values in @p want.
/// Numeric fields are compared with a small tolerance. @return 1 if all match.
int encoder_verify(r_device const *dev, bitbuffer_t *bits, capture_t const *want);

/// Read a decoded field as integer (base 10). @return 1 if present.
int encoder_field_int(capture_t const *cap, char const *key, int *out);

/// Read a decoded field as double. @return 1 if present.
int encoder_field_dbl(capture_t const *cap, char const *key, double *out);

/// Store the value actually encoded (after quantization/clamping) back into a
/// decoded field so encoder_verify() compares against what will be decoded.
void encoder_set_dbl(capture_t *cap, char const *key, double val);

/// Prepare a row-template encode for fixed-length payloads.
///
/// Copies the capture's raw bits to @p out and extracts the first @p nbits
/// of the repeated payload row (rows of @p nbits .. @p nbits + 2 bits) into
/// @p base, so unedited fields keep their captured values.
/// @return 0 on success, negative if no suitable row was captured.
int encoder_template_rows(capture_t const *cap, unsigned nbits, uint8_t *base, bitbuffer_t *out);

/// Overwrite the first @p nbits of every payload row in @p out with @p payload.
/// Rows of @p nbits - 1 (a dropped leading bit) are restored to full length;
/// trailing bits beyond @p nbits are kept.
void encoder_apply_rows(bitbuffer_t *out, unsigned nbits, uint8_t const *payload);

/// Set a single bit (MSB-first) in a raw bitrow.
void encoder_set_bit(uint8_t *row, unsigned idx, int val);

/// Encode @p nbits data bits (MSB-first in @p data) as Manchester half-bits into
/// @p row starting at bit @p start. @p invert selects the coding convention:
/// 0 matches bitbuffer_manchester_decode() (emits 1-d, d); 1 matches a decoder
/// that inverts after Manchester decoding (emits d, 1-d).
/// @return the next bit position after the encoded data.
unsigned encoder_manchester_encode(uint8_t *row, unsigned start, uint8_t const *data, unsigned nbits, int invert);

/// Encode @p nbits data bits (MSB-first in @p data) as differential Manchester
/// half-bits into @p row starting at bit @p start. @p prev holds the previous
/// physical half-bit level (updated on return). Matches
/// bitbuffer_differential_manchester_decode(): a 0 bit toggles, a 1 bit holds.
/// @return the next bit position after the encoded data.
unsigned encoder_diff_manchester_encode(uint8_t *row, unsigned start, int *prev, uint8_t const *data, unsigned nbits);

#endif /* INCLUDE_R_ENCODER_H_ */
