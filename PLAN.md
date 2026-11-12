# rtl_433_tx — Record / Edit / Re-transmit Plan

Extend rtl_433 (receive-only ISM decoder) with three integrated capabilities:
1. **Record** received transmissions (raw pulses + raw bits + decoded values) to a file.
2. **Edit** decoded values and reconstruct the raw data (e.g. lower a Toyota TPMS pressure by 1 psi).
3. **Re-transmit** original or edited recordings via a TX-capable SDR (HackRF One) or to an IQ file.

User choices: integrated into the rtl_433 binary; encoder framework for all devices reusing
decoder source; both SoapySDR-HackRF-TX and IQ-file output paths; verify builds along the way.

## Architecture
- Capture JSON format (`capture.[ch]`): metadata + pulses (µs) + raw bits + decoded fields (text).
- Pulse TX (`pulse_tx.[ch]`): bits→pulses inverse slicers + pulse_data→IQ synth (cu8/cs16/cf32).
- Encoder framework (`r_encoder.[ch]`): find device, rebuild via `encode_fn` or generic replay,
  decode-verify harness.
- Per-device `encode_fn` hook on `r_device` (Toyota TPMS implemented as reference).
- SDR TX (`sdr_tx.[ch]`): SoapySDR CF32 transmit path (HackRF).
- CLI: `-F capture[:FILE]` to record; `-j FILE` transmit; `-e key=value` edits; `-o FILE` IQ/json out.

## Status
- [x] Baseline build verified.
- [x] Modulator + inverse slicers + IQ writers (`pulse_tx.c`).
- [x] Capture JSON format read/write (`capture.c`).
- [x] Encoder framework + verify harness (`r_encoder.c`).
- [x] Toyota TPMS encoder (`devices/tpms_toyota.c`).
- [x] SoapySDR HackRF TX (`sdr_tx.c`).
- [x] Config fields (`rtl_433.h`): capture_filename/file, tx_infile/outfile, tx_edits.
- [x] CLI options wired (`-F capture`, `-j`, `-e`, `-o`) + `run_transmit()`.
- [x] Recording hook in `data_acquired_handler` (r_api.c).
- [x] init/free of tx_edits + capture_file (r_api.c).
- [x] Help/usage text updated.
- [x] Full project build green.
- [x] All 28 ctest tests pass (incl. style-check).
- [x] End-to-end verified: record -> replay -> edit (32->31 psi, CRC valid) via `rtl_433_tx -r`.
- [x] HackRF reachable via SoapySDR (SDR TX path ready; no real RF emitted during testing).
- [x] Build/install renamed: executable builds and installs as `rtl_433_tx`, man page as `rtl_433_tx.1` (CMake `OUTPUT_NAME`); tests updated, all 28 pass.

## Usage
- Record:      `rtl_433_tx -F capture:caps.json`            (append each decode: pulses+bits+fields)
- Replay IQ:   `rtl_433_tx -j caps.json -o out.cu8`         (.cu8/.cs16/.cf32 IQ, or .json edited capture)
- Edit+IQ:     `rtl_433_tx -j caps.json -e pressure_PSI=31 -o out.cu8`
- Transmit:    `rtl_433_tx -j caps.json -d driver=hackrf -g 20`   (omit -o => SDR TX; freq from capture)
- Edit keys:   any decoded field, plus freq_hz / sample_rate / repeats.
- Dev helper:  `tests/tx_bootstrap.c` builds a synthetic valid Toyota capture for testing.

## Verification evidence (software loop, 250 kHz cu8)
1. `-F capture` recorded 5 Toyota captures from a replayed IQ file.
2. Replaying a recorded capture re-decodes to the original values (32.0 psi).
3. Editing `-e pressure_PSI=31` rebuilds via the Toyota encoder; re-decode yields 31.0 psi, CRC OK.

## Notes
- Toyota encoder uses template-patching (needs a real captured template in `cap.bits`).
- Un-edited replay regenerates clean pulse timing from the decoded bits when the device is known
  (avoids receiver jitter corrupting the retransmission); falls back to stored pulses otherwise.
- Full hand-written encoders for all ~342 devices are scaffolded (framework + generic replay +
  bit-level edit), with Toyota as the full-fidelity reference encoder.
