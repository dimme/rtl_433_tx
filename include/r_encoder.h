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

#endif /* INCLUDE_R_ENCODER_H_ */
