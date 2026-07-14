# IMU Frame Remap

> See [GLOVE_PROJECT.md](GLOVE_PROJECT.md) for the full project plan, math reference, glossary, and operational FAQ. This file is the focused reference for the canonical glove frame and the per-board axis remap.

This project treats firmware as the source of truth for node orientation.

## Canonical Glove Frame

All nodes publish fused quaternions in the same glove frame:

- `+X`: forward toward the fingertips
- `+Y`: left across the hand
- `+Z`: up away from gravity when the hand is flat

## Handedness (v1)

This glove is a **right-hand** glove. Convention for the canonical frame:

- Place the right hand palm-down, fingers pointing away from the body.
- `+X` points forward along the fingers.
- `+Y` points from the pinky side toward the thumb (to the operator's left when looking at the back of the hand).
- `+Z` points up, out of the back of the hand.

A left-hand variant would invert `+Y`. It is not in scope for v1. The host pipeline and the Qt 3D UI data contract carry a `handedness` field so a left-hand build can be added without a breaking change.

## Board Layout and Finger Mapping

| Finger | Palm UART | Host Node ID Block | Canonical Local ID |
| --- | --- | --- | --- |
| thumb  | UART2 | 20..29 | 20 |
| index  | UART3 | 30..39 | 30 |
| middle | UART4 | 40..49 | 40 |
| ring   | UART5 | 50..59 | 50 |
| pinky  | UART6 | 60..69 | 60 (future) |
| spare  | UART7 | 70..79 | reserved |

The palm board itself sits on the **back of the hand behind the knuckles** and publishes its fused quaternion as node `0`. Its orientation therefore represents overall hand/wrist pose, not the forearm.

## Palm Board Mapping

The active palm remap happens before bias estimation and Mahony fusion in `Core/Src/app/palm_runtime.c`.

Current palm defaults:

- `IMU0`: `X <- +Xraw`, `Y <- +Yraw`, `Z <- +Zraw`
- `IMU1`: `X <- -Xraw`, `Y <- -Yraw`, `Z <- +Zraw`

`IMU1` is mounted as a 180 degree rotation around the board `+Z` axis relative to `IMU0`.

## Fingertip Firmware Contract

Use the same pattern in the fingertip firmware repo:

1. Determine the sensor-native `X/Y/Z` directions from the board drawing.
2. Write one fixed raw-axis remap from sensor frame into the canonical glove frame.
3. Apply that remap to accelerometer and gyro samples before fusion.
4. Transmit the fused quaternion without any manual `w/x/y/z` component remap.

If a fingertip board cannot be described by a clean axis permutation plus sign inversion, apply one fixed mounting quaternion as a true quaternion rotation after fusion. Do not remap `w` as if it were an axis.

## Validation

Validate each node against the palm board in the same physical pose:

1. Place the palm board and one fingertip board flat in the same orientation.
2. Stream both fused quaternions.
3. Adjust the fingertip firmware remap until both orientations match closely.
4. Compare orientation equivalence, not raw sign, because `q` and `-q` represent the same pose.

## Phase 0 Procedure (do this once per fingertip board)

Goal: get each fingertip board's forwarded fused quaternion into the canonical glove frame so every `PALM_EXTERNAL_NODE_UARTn_REMAP_*` in `Core/Inc/imu/imu_config.h` ends up at identity.

1. Confirm the fingertip boards are wired to their intended UARTs per the mapping table above. If unsure, run the isolation workflow in `tools/usb_cdc_monitor/README.md` (section "Node Isolation Workflow").
2. Open the monitor: `python tools/usb_cdc_monitor/monitor.py`. Connect to the palm board.
3. Place the palm board on a flat, level surface, back-of-hand-up, fingers direction pointing forward along `+X`.
4. Lay one fingertip board on the same surface in the same orientation, close enough that both boards share a single pose. Wait a few seconds for the fused output to settle.
5. Click `Start Capture` in the monitor and record at least 3 seconds of samples with nothing moving. Click `Stop Capture`.
6. Run the alignment tool against the saved CSV:

   ```powershell
   python tools/usb_cdc_monitor/frame_alignment.py path\to\capture.csv --node 20
   ```

   Replace `--node 20` with the node id of the board being aligned (20 thumb, 30 index, 40 middle, 50 ring, 60 pinky). Omit `--node` to process all fingertip nodes present in the capture.
7. The tool prints:
   - the raw residual between the tip board's fused quaternion and the palm's (how misaligned the boards currently are)
   - a suggested `PALM_EXTERNAL_NODE_UARTn_REMAP_W/X/Y/Z` plus `_ORDER` block ready to paste into `Core/Inc/imu/imu_config.h`
   - the simulated residual that would remain after applying the suggested mount (target: < 2 deg)
8. Paste the suggested block into `Core/Inc/imu/imu_config.h`, replacing the current values for that UART. Rebuild and flash.
9. Re-run steps 3-6 with the newly-flashed firmware. The raw residual should now be small (< 2 deg). If it is, the tip board is fully canonical.
10. Preferred long-term fix: move the correction from the palm-side mounting quaternion into the fingertip's own firmware raw-axis remap per the "Fingertip Firmware Contract" section above. Once done, the corresponding `PALM_EXTERNAL_NODE_UARTn_REMAP_*` in this repo can be set back to identity (`W=1, X=Y=Z=0`) and the residual test must still pass.

> Quick diagnosis path: if you suspect the mismatch is an axis swap or sign flip rather than a small mounting tilt (e.g. "thumb's `X` looks like index's `Y`"), use the `Phase 0 Auto-Detect (3 s)` button in `tools/usb_cdc_monitor/monitor.py` (or `python tools/usb_cdc_monitor/phase0_autodetect.py <csv>`). It captures one flat pose with all boards on the table at once and writes a `<csv>.phase0.json` report classifying every fingertip against all 24 right-handed permutations. The JSON is small enough to paste into a chat with an agent for review. See "Phase 0 Auto-Detect" in `tools/usb_cdc_monitor/README.md` for the schema.

### Phase 0 Exit Criterion

- Every active fingertip node (20, 30, 40, 50) passes the same-pose residual test at less than 2 degrees using either:
  - identity `PALM_EXTERNAL_NODE_UARTn_REMAP_*` values in `Core/Inc/imu/imu_config.h` (clean case, fingertip firmware is canonical), OR
  - the mounting quaternion generated by `frame_alignment.py` for that UART (fallback case, fingertip firmware still publishes its own board frame).

Record per-board results here once captured:

| Node | UART | Route (firmware canonical / palm fallback) | Post-remap residual | Date |
| ---- | ---- | ------------------------------------------ | ------------------- | ---- |
| 20 (thumb)  |  |  |  |  |
| 30 (index)  |  |  |  |  |
| 40 (middle) |  |  |  |  |
| 50 (ring)   |  |  |  |  |
| 60 (pinky)  |  |  |  |  |

## App Cleanup

After every node publishes the canonical glove frame, the desktop app remaps should become identity:

- Palm: `X <- X`, `Y <- Y`, `Z <- Z`
- Finger tip: `X <- X`, `Y <- Y`, `Z <- Z`

Only make that app-side change after same-pose validation passes for palm and fingertip firmware.

## Palm-Side Forwarded Node Remap

The palm can also normalize forwarded external fused frames before they go to USB.

Per-UART fallback mounting quaternions live in `Core/Inc/imu/imu_config.h` as:

- `PALM_EXTERNAL_NODE_UARTn_REMAP_W/X/Y/Z`
- `PALM_EXTERNAL_NODE_UARTn_REMAP_ORDER`

This path is useful when a fingertip firmware repo has not been updated yet. In that mode:

1. The fingertip still sends its fused quaternion in its own board frame.
2. The palm decodes that quaternion after UART receive.
3. The palm applies one fixed mounting quaternion for that UART stream.
4. The palm re-encodes the quaternion, recomputes CRC, and forwards the corrected frame to USB.

Use identity values once the fingertip firmware itself publishes the canonical glove frame.
