# STM32 USB CDC Monitor

> Project-level overview, plan, and glossary live in [../../GLOVE_PROJECT.md](../../GLOVE_PROJECT.md). The canonical glove frame and per-board axis remap reference is in [../../IMU_FRAME_REMAP.md](../../IMU_FRAME_REMAP.md). This file is the host-tool-specific manual.

This tool is a Python desktop UI for the STM32 test firmware's binary USB CDC stream.

It treats the serial port as raw binary data, not text, and accepts either:

- the original 29-byte raw dual-IMU frame
- the newer 18-byte fused-orientation frame emitted by the current `palm_runtime_process()` path

## Features

- Serial port selector with refresh
- Connect and disconnect controls
- Timestamped CSV capture plus plain-text capture summary
- Rolling binary frame resync using `0xD6` and `0xB6` header scans
- Live IMU0 and IMU1 numeric display for raw frames
- Per-node fused summaries for palm-local and external forwarded nodes
- Live fused quaternion display for the selected fused node
- Live yaw, pitch, roll, and drift-rate estimates for the selected fused node
- Phase 1 glove panel with host-side palm-relative `flex/swing/twist` per finger
- Host-side neutral-pose calibration saved to `calibration.json`
- `Zero All` / `Clear Zero` command buttons for the palm-side runtime zero layer
- Packet, checksum error, sequence gap, and discarded-byte counters
- Per-protocol counters for raw vs fused frames
- Fused-frame `node_id` display plus a per-node table and selector
- Last accepted raw frame shown in hex
- Section-delta indicator to show which payload area changed
- Drift summary and tuning hints tied to `imu_config.h`
- Capture sidecar `.glove.jsonl` with host-computed glove snapshots
- Optional live plots when `matplotlib` is installed

## Supported Frame Formats

### Raw Dual-IMU Frame

- Byte `0`: `0xD6`
- Byte `1`: `0x01`
- Byte `2`: sequence
- Bytes `3..14`: IMU0 accel xyz in `mg`, gyro xyz in `0.1 dps`, all `int16` little-endian
- Bytes `15..26`: IMU1 accel xyz in `mg`, gyro xyz in `0.1 dps`, all `int16` little-endian
- Byte `27`: status
- Byte `28`: XOR checksum over bytes `1..27`

### Fused Orientation Frame

- Byte `0`: `0xB6`
- Bytes `1..6`: currently zero-filled force/reserved fields
- Bytes `7..14`: quaternion `w x y z` as `int16` little-endian scaled by `10000`
- Byte `15`: status
- Byte `16`: node ID
- Byte `17`: XOR checksum over bytes `1..16`

For forwarded external nodes, the palm rewrites `node_id` as:

- `UART2`: `20 + local_index`
- `UART3`: `30 + local_index`
- `UART4`: `40 + local_index`
- `UART5`: `50 + local_index`
- `UART6`: `60 + local_index`
- `UART7`: `70 + local_index`

The fingertip sender should place its local chain index in byte `16` before the palm remaps it. Keep local indices in the range `0..9` so each UART keeps a stable decade-based ID block.

Status bits currently observed in the firmware:

- Bit `0`: IMU0 OK
- Bit `1`: IMU1 OK
- Bit `2`: fusion ready
- Bit `3`: single IMU mode
- Bit `4`: USB busy
- Bit `5`: calibrating
- Bit `6`: filter warmup

## Install

From this folder:

```powershell
python -m venv .venv
.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

`pyserial` is required. `matplotlib` enables the embedded live plots.

## Run

From this folder:

```powershell
python monitor.py
```

## Phase 1 Glove Math

The monitor now includes a `Phase 1 Glove Math` panel that treats the palm as the reference and computes per-finger motion on the host:

- `q_rel_i = inv(q_palm) * q_tip_i`
- neutral calibration stores `m_i` per finger
- runtime uses `inv(m_i) * q_rel_i`
- swing-twist around `+X` yields:
  - `flex_deg` for curl toward the palm
  - `swing_deg` for sideways spread
  - `twist_deg` as a diagnostic roll term

Thumb uses the same pipeline as the other fingers. The only thumb-specific behavior is a later renderer-side linkage choice, not the measurement math itself.

### Zero All commands

The panel exposes the palm firmware's existing runtime zero layer:

- `Zero All` -> send `{0xC0, 0x01}`
- `Clear Zero` -> send `{0xC0, 0x00}`

These commands are handled by the palm over USB CDC and do not change fingertip firmware.

### Calibrate Flat

`Calibrate Flat` captures about 2 seconds of the current neutral pose, computes host-side per-finger offsets, and writes `calibration.json` in the current working directory. `Reset Calibration` clears that file and returns the display to raw palm-relative values.

## Drift Workflow

1. Connect to the STM32 USB CDC COM port and wait for fused frames.
2. Click `Start Capture` and choose a CSV file path.
3. For a still test, leave the unit motionless for 30 to 60 seconds.
4. For a motion recovery test, rotate the unit quickly, then hold it still and let it settle.
5. Click `Stop Capture` when done.
6. The app saves:
   - a CSV with timestamped accepted frames
   - a companion `_summary.txt` file with a plain-language drift summary and tuning hint
   - a companion `.glove.jsonl` file with host-computed Phase 1 glove snapshots on palm frames

CSV columns include time, protocol, `node_id`, status, quaternion, yaw/pitch/roll, and raw IMU fields when available.

## Repeatable Drift Tests

### Still Test

Use this to measure drift while the hardware is fully still:

1. Boot the board and wait until warmup clears.
2. Start a capture.
3. Keep the board motionless for 30 to 60 seconds.
4. Note which angle drifts most in the UI summary.
5. Check whether `Validity` says both IMUs stayed valid throughout.

Useful output example:

`still: yaw drifts +1.0 deg/10s, pitch/roll stable, both IMUs valid throughout`

### Motion Recovery Test

Use this to see whether motion causes overshoot or a slow walk afterward:

1. Start a capture.
2. Rotate the board briskly around one axis.
3. Set it down and hold it still again.
4. Watch the yaw/pitch/roll values and drift summary while it settles.
5. Stop the capture and inspect the summary text plus plots.

### IMU Drift Test (Single Duration)

Use this when you want a quick ranking of which finger/node drifts the most during a fixed still window:

1. In `Drift Analysis`, choose `IMU Drift Test` duration: `10`, `30`, `80`, `120`, or `Custom`.
2. If using `Custom`, enter the number of seconds (for example `180`).
3. Click `Start Drift Test` and save the CSV path when prompted.
4. Keep the glove as still as possible for the full duration.
5. The test auto-stops and reports:
   - `Worst: <finger/node> <axis> <rate>`
   - an ordered ranking for all nodes with enough fused samples.

If the report says there were not enough samples, verify that fused frames were streaming for palm/finger nodes during the test.

## Firmware Tuning Guide

The main firmware tuning constants are in `Core/Inc/imu/imu_config.h`:

- `PALM_MAHONY_KP`
  - Increase slightly if pitch/roll correction is too weak while still.
  - Decrease slightly if motion causes noisy correction or overshoot.
- `PALM_MAHONY_KI`
  - Increase slightly if still drift suggests untrimmed gyro bias.
  - Decrease if the estimate becomes sticky or slow to settle after movement.
- `PALM_FUSION_MAX_DISAGREEMENT_DEG`
  - Adjust only if the fused output is frequently entering disagreement or single-IMU mode.
- `PALM_IMU_WARMUP_SAMPLES`
  - Increase only if the early startup phase is clearly unstable and polluting the beginning of a capture.

Important note:

- In the current 6DOF accel+gyro path, yaw drift is harder to eliminate completely than pitch/roll drift because there is no magnetometer heading reference in the transmitted fused frame path.

## Validation Workflow

### Palm board output remap target

After flashing firmware with palm output-frame remap enabled, treat host-side palm remap as identity.

- Axis map: `X_out = -Y_in`, `Y_out = +Z_in`, `Z_out = -X_in`
- Matrix: `C = [[0,-1,0],[0,0,1],[-1,0,0]]`
- Orientation transform: `R' = C * R * C^T`

For the original raw dual-IMU frame:

1. Connect to the STM32 USB CDC COM port.
2. Verify that `IMU0: OK` and `IMU1: OK` appear in the status bits when both sensors are healthy.
3. Move only IMU0.
4. Confirm the IMU0 numeric fields change while IMU1 remains steady.
5. Check `Section Delta` and confirm it reports `IMU0 changed | IMU1 steady` for those motions.
6. Inspect the `Last Raw Frame` panel and confirm the `IMU0` 12-byte section changes while the `IMU1` 12-byte section stays steady.
7. Repeat with only IMU1 moving and confirm the inverse behavior.

For the current fused-orientation frame:

1. Connect to the STM32 USB CDC COM port.
2. Confirm one or more fused nodes appear in the `Fused Nodes` table.
3. Node `0` is the palm-local fused node. Forwarded external nodes appear as `20..29`, `30..39`, `40..49`, `50..59`, `60..69`, or `70..79` based on which UART received them.
4. Select a node from the combo box or table to inspect its quaternion, yaw/pitch/roll, drift summary, and plots independently.
5. Check the `Last Accepted Frame` panel and verify that the quaternion bytes change in the `Quat` section.
6. Use the optional `quat` plot mode to view the currently selected node.

Short palm verification capture (10 to 20 s total):

1. Click `Start Capture`.
2. Hold neutral briefly, then perform one clear sweep each for pitch-only, yaw-only, and roll-only.
3. Click `Stop Capture`.
4. Confirm with host palm remap identity:
   - pitch motion mainly affects pitch
   - yaw motion mainly affects yaw
   - roll motion mainly affects roll
   - no strong cross-axis coupling in small-to-moderate rotations
   - neutral hold stays near-zero drift

## Node Isolation Workflow

Use this before changing `Core/Inc/imu/imu_config.h` again when you need to confirm which host node IDs `30`, `40`, and `50` actually correspond to index, middle, and ring.

1. Connect to the STM32 USB CDC COM port and wait for palm-local fused node `0`.
2. Keep the current firmware and remap trial unchanged.
3. In the `Guided Pose Test` section, choose:
   - `Isolation Finger`: `index`, `middle`, or `ring`
   - `Round`: `1` or `2`
4. Click `Start Isolation Capture` and save a file like `round1_index_only.csv`.
5. Keep the palm as fixed as possible and move only the chosen finger through one clear curl/extend cycle.
6. Click `Stop Capture`.
7. Repeat for `middle` and `ring`, then repeat the whole set for round `2`.
8. Use `Analyze Isolation CSVs` to select the six isolation files together and get a consensus summary.

The isolation analyzer ranks nodes `30`, `40`, and `50` by how much they move relative to the palm using:

- max relative quaternion rotation from the file start
- yaw range
- pitch range
- roll range

If the same node wins both rounds for a finger, the host-side finger-to-node mapping is considered stable enough to tune remaps again.

### Command-line Isolation Analysis

You can also analyze isolation CSVs without the UI:

```powershell
python isolation_analysis.py round1_index_only.csv round2_index_only.csv round1_middle_only.csv round2_middle_only.csv round1_ring_only.csv round2_ring_only.csv
```

The script prints:

- per-file dominant node and palm motion
- a consensus section showing whether `index`, `middle`, and `ring` are stable across the provided files

## Frame Alignment (Phase 0)

Use `frame_alignment.py` to compute the palm-side mounting quaternion that brings a fingertip board into the canonical glove frame (`+X` toward fingers, `+Y` across hand, `+Z` up). See `IMU_FRAME_REMAP.md` for the full Phase 0 procedure and finger-to-UART mapping.

Short version:

1. Lay the palm board and one fingertip board flat on the same surface, same physical orientation.
2. In the monitor, `Start Capture`, wait ~3 seconds with nothing moving, `Stop Capture`.
3. Run against the saved CSV:

   ```powershell
   python frame_alignment.py path\to\capture.csv --node 20
   ```

   `--node` values: `20` thumb, `30` index, `40` middle, `50` ring, `60` pinky. Omit to process all fingertip nodes present.
4. The script prints a `PALM_EXTERNAL_NODE_UARTn_REMAP_W/X/Y/Z` plus `_ORDER` block for each node, along with the raw residual and the simulated residual that would remain after applying the block. Paste the block into `Core/Inc/imu/imu_config.h`, rebuild, flash, recapture; the raw residual should drop below 2 degrees.
5. Long term, prefer fixing the frame inside each fingertip firmware's raw-axis remap and restoring `PALM_EXTERNAL_NODE_UARTn_REMAP_*` to identity.

## Phase 0 Auto-Detect

Use this when you want a single-shot, all-fingers diagnostic that classifies each fingertip's mounting orientation as one of the 24 right-handed axis permutations (e.g. "thumb's `X` is `+Yraw`, `Y` is `-Xraw`, `Z` is `+Zraw`"). The output is a JSON report you can hand to a human reviewer or to an AI agent for analysis. It complements `frame_alignment.py` (which gives you a generic mounting quaternion); the auto-detect tool gives you the cleaner permutation form, which is what a fingertip firmware repo's raw-axis remap should land as.

Two ways to run:

### From the monitor UI

1. Lay the palm board and every active fingertip board flat on the same surface, all in the same physical orientation. Wait a few seconds for Mahony fusion to settle.
2. In the `Phase 0 Frame Alignment Auto-Detect` panel:
   - Click `Phase 0 Auto-Detect (3 s)` for a one-click flow. The monitor records ~3 seconds automatically into `phase0_auto_<timestamp>.csv` in the current working directory, then writes a `<csv>.phase0.json` next to it and pops up the path plus a one-line summary.
   - Click `Analyze Capture (Phase 0)` to re-analyze any existing capture CSV (your own or one shared by someone else). Same JSON report is produced next to the chosen CSV.
3. The popup also surfaces movement warnings if the palm or any tip's orientation spread exceeded `0.5 deg` during the capture; that is the typical sign that you need to redo it.

### From the command line

```powershell
python phase0_autodetect.py path\to\capture.csv [--out report.json]
```

`--out` defaults to `<csv>.phase0.json` next to the input CSV.

### JSON report schema

```text
schema_version: "phase0-autodetect-1.0"
csv_path:       absolute path to the analyzed capture
captured_seconds: span of time_s in the CSV
palm:
  samples:    palm-row sample count
  spread_deg: max angular distance of palm samples from their hemispherical mean
fingers[]:
  node, uart, label
  samples, palm_spread_deg, tip_spread_deg
  raw_residual_deg
  best_permutation:
    name        ("identity" or "perm[+Y,-X,+Z]" style)
    axes        {"X": "+Yraw", "Y": "-Xraw", "Z": "+Zraw"}
    axes_text   "X <- +Yraw, Y <- -Xraw, Z <- +Zraw"
    quaternion  [w,x,y,z] of the chosen permutation
    residual_deg, runner_up_*_deg, confidence_margin_deg
  free_form_mount:
    quaternion       arbitrary best-fit quaternion (same as frame_alignment.py)
    residual_deg     residual after applying it
  verdict:        GOOD | OK | POOR
  verdict_reason: short text explaining the verdict
  fix:
    fingertip_firmware_raw_axis_remap   suggested raw-axis remap line (preferred fix)
    palm_fallback_imu_config_block      ready-to-paste #define block (fallback)
summary_text: one-line per-finger summary
```

Verdict thresholds: `GOOD` requires the best permutation to fit within 2 deg AND beat the runner-up by at least 5 deg. `OK` is a fit within 5 deg. `POOR` is anything else, or any per-board spread above 0.5 deg (interpreted as "you moved during the capture, redo it").

## Notes

- The monitor never assumes packet alignment. It continuously rescans for a valid frame boundary.
- The CDC stream may contain non-IMU bytes or partial frames, so checksum errors and discarded bytes are exposed in the UI.
- Sequence gaps apply to the raw frame format only.
- Interleaved fused frames from multiple `node_id` values are tracked independently; drift metrics and captures are separated per node.
- Raw `0xD6` handling remains a single live debug view because those frames do not currently carry a forwarded-node source identifier.
- For external fused nodes, `node_id` on the host is palm-assigned from `(UART_base + fingertip_local_index)`, not the untouched sender-side byte.
