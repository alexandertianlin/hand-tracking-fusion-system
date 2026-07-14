# Glove Data-Collection Project

This document is the single entry point for the data-collection glove built around the STM32H7B0 palm board in this repository. It captures the plan, the math, the operational procedures, the file map, and the design decisions so that anyone (or any agent) picking up the project later can understand it and continue it without re-deriving the rationale.

If you are short on time, read sections [What This Project Is](#what-this-project-is), [Hardware Topology](#hardware-topology), [Phased Plan](#phased-plan), and [How To Operate Phase 0](#how-to-operate-phase-0-frame-alignment).

---

## Table of Contents

- [What This Project Is](#what-this-project-is)
- [Hardware Topology](#hardware-topology)
- [Canonical Glove Frame](#canonical-glove-frame)
- [The Two Different Calibrations](#the-two-different-calibrations)
- [Phased Plan](#phased-plan)
- [How To Operate Phase 0 (Frame Alignment)](#how-to-operate-phase-0-frame-alignment)
- [Palm-Side Frame Correction + Zero All (Runtime)](#palm-side-frame-correction--zero-all-runtime)
- [Next Steps](#next-steps)
- [Phase 1 Design (Per-User Pose Calibration + Joint Math)](#phase-1-design-per-user-pose-calibration--joint-math)
- [Quaternion Math Reference](#quaternion-math-reference)
- [Kinematic Linkage (One Sensor Site per Finger -> Many Joint Angles)](#kinematic-linkage-one-sensor-site-per-finger---many-joint-angles)
- [Host <-> Qt 3D UI Data Contract](#host---qt-3d-ui-data-contract)
- [File Map](#file-map)
- [Glossary](#glossary)
- [FAQ / Gotchas](#faq--gotchas)
- [Decision Log](#decision-log)

---

## What This Project Is

A **right-hand data-collection glove**. The end goals are:

1. Drive a 3D hand model in real time on a PC.
2. Record a clean, reproducible dataset of hand poses for downstream ML / analytics.

Required output, every frame:

- Palm `(yaw, pitch, roll)` in degrees - overall hand/wrist attitude.
- Per finger `(flex_deg, swing_deg)` - cumulative curl and abduction relative to the back of the hand.
- Per finger relative quaternion - the underlying truth, so derived quantities can be recomputed offline.

Out of v1 scope (parked, not lost): magnetometer for palm yaw drift; non-volatile storage of calibrations; multi-IMU per finger; left-hand variant.

---

## Hardware Topology

| Component | Location | Sensor(s) | Role |
| --- | --- | --- | --- |
| Palm board (this repo) | Back of the hand, behind the knuckles | 2x LSM6DSOW (`I2C3`, addr `0x6A`/`0x6B`), fused on-board | Hand/wrist orientation + USB host link + UART hub for fingertips |
| Fingertip board (separate firmware repo) | Distal phalanx of each finger | 2x LSM6DSOW each, fused on-board (same dual-IMU pattern as the palm) | Per-finger orientation, one fused quaternion per board |

Both board types follow the same dual-IMU + Mahony fusion pattern. From the palm firmware's perspective and from the host's perspective, **each fingertip board emits exactly one fused quaternion per cycle** - the dual-IMU redundancy lives entirely inside the fingertip firmware and improves noise/drift behavior without changing the data shape downstream.

Wiring as of v1:

| Finger  | Palm UART | Host node ID block | Canonical local ID |
| ---     | ---       | ---                | ---                |
| thumb   | UART2     | 20..29             | 20                 |
| index   | UART3     | 30..39             | 30                 |
| middle  | UART4     | 40..49             | 40                 |
| ring    | UART5     | 50..59             | 50                 |
| pinky   | UART6     | 60..69             | 60 (future)        |
| spare   | UART7     | 70..79             | reserved           |

The palm board emits its own fused quaternion as `node_id 0` and forwards each fingertip's fused quaternion with the palm-assigned node ID above. **Only `NRST` is free as a discrete on the palm**, so there is no hardware button for calibration; calibration is triggered from the PC and on auto-startup.

Frame formats over USB CDC are documented in [tools/usb_cdc_monitor/README.md](tools/usb_cdc_monitor/README.md) under "Supported Frame Formats". Today there are two:

- `0xD6` raw dual-IMU frame (debug only).
- `0xB6` fused-orientation frame, one per node, 18 bytes.

A new `0xA6` joint-angle frame is added in Phase 3 (see plan below).

---

## Canonical Glove Frame

**Every node** (palm + each fingertip) publishes its fused quaternion in the same body-fixed reference frame:

- `+X` forward, along the fingers
- `+Y` left across the hand (right-hand glove convention: from pinky toward thumb)
- `+Z` up, out of the back of the hand when the hand is flat

This convention is treated as the **source of truth in firmware**, not in the host app. See [IMU_FRAME_REMAP.md](IMU_FRAME_REMAP.md) for per-board details and the validation procedure.

When everything is correct, the host-side parser and the Qt 3D UI need **no axis remap** at all. Identity transforms everywhere downstream.

---

## The Two Different Calibrations

This is the single most-confused topic on the project. Read this twice.

|                  | **Phase 0: board-frame alignment**                                  | **Phase 1: per-user pose calibration**                              |
| ---              | ---                                                                 | ---                                                                 |
| What it removes  | The IMU's mounting offset on its PCB (sensor `+X` not aligned with board `+X`, etc.) | The user's natural finger geometry (thumb splay, glove fit, finger that doesn't lay perfectly straight) |
| Frequency        | Once per board per firmware revision. **Permanent**, baked into a `#define` | Once per session per user. **Temporary**, stored in `calibration.json` |
| Done with        | Bare boards on a flat table, palm + tip in the SAME physical pose   | Glove worn, user holds the chosen neutral pose (e.g. high-five)     |
| Tooling          | [tools/usb_cdc_monitor/frame_alignment.py](tools/usb_cdc_monitor/frame_alignment.py) | `glove_pipeline.py` (Phase 1, planned) |
| Math             | `q_mount = q_palm * conj(q_tip_raw)` (left-multiply, applied by palm firmware via `PALM_EXTERNAL_NODE_UARTn_REMAP_*`) | `m_i = inv(q_palm_neutral) * q_tip_i_neutral` (applied host-side) |
| Side effect if skipped | Per-finger angles drift wildly with hand orientation | Thumb reads ~30 deg of swing in neutral pose; user offsets leak into every recording |

**Both are needed.** Phase 0 is a prerequisite for Phase 1. If you mix them, you'd have to redo the "permanent" board alignment every time a user puts the glove on, which defeats the point of a permanent fix.

---

## Phased Plan

The work is split into five phases, each delivering something testable, none requiring rework of the previous phases. Status as of this document: Phase 0 has landed (tooling + palm-side runtime correction + auto-zero at boot); Phases 1-4 are designed but not yet built.

### Phase 0 - Canonical Frame Lock (DONE)

Make every node publish in one canonical glove frame so per-boot manual axis adjustments end forever.

Three complementary routes now exist, in increasing order of permanence:

1. **Fingertip-firmware raw-axis remap** (preferred long-term) - each fingertip board publishes in the canonical frame directly.
2. **Palm-side common map + per-UART remap** (landed, used today) - the palm board applies a shared sandwich-conjugation correction to every forwarded fingertip quaternion via `PALM_EXTERNAL_NODE_COMMON_MAP_*` in [Core/Inc/imu/imu_config.h](Core/Inc/imu/imu_config.h), plus an optional per-UART `PALM_EXTERNAL_NODE_UARTn_REMAP_*` left/right-multiply fallback. See [Palm-Side Frame Correction + Zero All (Runtime)](#palm-side-frame-correction--zero-all-runtime).
3. **Runtime Zero All** (landed) - palm board latches the current palm and per-port fingertip quaternions as zero references and emits `inv(ref) * q` on the USB output. Fires automatically on boot after a settling window and can be re-triggered with `{0xC0, 0x01}` over USB CDC.

Tooling for route 1 diagnosis:

- [tools/usb_cdc_monitor/frame_alignment.py](tools/usb_cdc_monitor/frame_alignment.py) computes the mounting quaternion from a flat-pose CSV.
- [tools/usb_cdc_monitor/phase0_autodetect.py](tools/usb_cdc_monitor/phase0_autodetect.py) classifies every fingertip's mounting as one of the 24 right-handed axis permutations.

**Exit criterion (met):** after flashing the current firmware, palm and every fingertip read the same axes with the same signs in the world-Euler view once the post-boot Zero All fires. Record per-board findings in the table at the bottom of [IMU_FRAME_REMAP.md](IMU_FRAME_REMAP.md). Folding the common-map sandwich back into fingertip firmware (route 1) stays on the Phase 0 backlog.

### Phase 1 - Host-Side Glove Math + Glove UI Panel (NEXT)

All math lives in Python first so we can iterate cheaply. Nothing firmware-side.

- New module `tools/usb_cdc_monitor/glove_pipeline.py`:
  - `palm_ypr(q)` -> ZYX intrinsic Euler.
  - `finger_rel(q_palm, q_tip)` -> per-finger relative quaternion.
  - `apply_calib(q_rel, m_i)` -> remove per-user neutral offset.
  - `swing_twist_decompose(q, axis=X)` -> `(flex_deg, swing_deg, twist_deg)`.
  - `compute_snapshot()` -> the JSON record below.
- Calibration in Python:
  - "Calibrate Flat" button captures ~2 s of palm + per-node quats, stores `q_palm_ref` and per-finger `m_i = conj(q_palm_ref) * q_tip_ref`, writes `calibration.json`.
  - "Reset Calibration" clears state.
- Extend [tools/usb_cdc_monitor/monitor.py](tools/usb_cdc_monitor/monitor.py) with a **Glove** panel: palm YPR bars, per-finger flex/swing bars + numeric readout + validity dot, Calibrate / Reset buttons, session recorder writing CSV + JSONL with both raw quats and computed angles.

### Phase 2 - Data Contract for the External Qt 3D UI

The Qt 3D UI lives in a separate repo. Give it a stable, USB-agnostic feed.

- Add `tools/usb_cdc_monitor/glove_stream_server.py`: line-delimited JSON over local TCP on `127.0.0.1:8765`, schema in [Host <-> Qt 3D UI Data Contract](#host---qt-3d-ui-data-contract).
- UI toggle in monitor.py: "Enable external stream".
- Document the contract in `tools/usb_cdc_monitor/GLOVE_STREAM.md`.
- Include a tiny reference consumer script that prints the last record.

### Phase 3 - MCU Calibration Command + On-Board Joint Angles

Port the host math into firmware for low-latency / standalone use. Host keeps recomputing from raw quats so the dataset stays reproducible.

- Protocol additions:
  - New USB-in command opcode: `0xC1, 0x01` start flat calibration; `0xC1, 0x00` reset calibration. (The `0xC0, *` group is already taken by the runtime Zero All; see [Palm-Side Frame Correction + Zero All (Runtime)](#palm-side-frame-correction--zero-all-runtime).) Extend the chain in `Process_USB_Receive_Buffer` in [Core/Src/usb_process.c](Core/Src/usb_process.c).
  - New USB-out frame `0xA6`: palm quat + palm YPR + 5x (flex, swing) as `int16` deg*100 + status + `node_id=0xA0` + XOR. Target ~30 bytes.
  - Host parser in [tools/usb_cdc_monitor/palm_parser.py](tools/usb_cdc_monitor/palm_parser.py) learns `0xA6`.
- Firmware:
  - `Core/Src/app/glove_calibration.c` storing `q_palm_ref` and per-node `m_i` in RAM (NVM later).
  - `Core/Src/app/glove_joint_math.c` - straight port of the Python math.
  - Hook into `palm_runtime_process()` to build and transmit `0xA6` alongside the existing `0xB6` frames.
- Triggers: PC command (primary), auto at boot after gyro-bias finalization (gated by `GLOVE_AUTO_POSE_CALIB`, default on). Hardware button **deferred** because only `NRST` is free.
- Cross-check: Python math (Phase 1) and firmware math (Phase 3) must agree within tolerance on captured recordings. **Python is the reference**; if firmware diverges, firmware is the bug.

### Phase 4 - Dataset Workflow + Pinky + Polish

- Labeled gesture sessions, schema versioning in JSONL header, per-session hash of the active calibration.
- End-to-end pinky on UART6 / node 60 (frame layout already reserves the slot).
- Optional second-pose calibration ("full fist") per finger for per-user flex range normalization, lets the Qt UI show 0..100 % curl instead of raw degrees.
- Parked items kept visible: magnetometer for palm yaw, NVM-stored calibration, multi-session statistics.

---

## How To Operate Phase 0 (Frame Alignment)

Repeat for each fingertip board. Numbers below assume index/UART3/node 30; substitute thumb (UART2/20), middle (UART4/40), ring (UART5/50) as needed.

1. Power up the glove and connect over USB. `python tools/usb_cdc_monitor/monitor.py`, select port, **Connect**.
2. **Physically co-align the two boards.** This is the most important step.
   - Bare-board mode (preferred): place the palm board and the index fingertip board side by side on a flat table, both oriented so the silkscreen `+X` arrow (or your chosen reference edge) points the same direction. Both face up, both flat. Wait ~3-5 s for Mahony to settle.
   - Glove-already-assembled mode: take the glove off the hand, lay the palm board flat on the table, then physically lay the fingertip board on the same table next to it in the same orientation. You may have to flex the glove to do this; that is fine.
3. In monitor, **Start Capture** -> save as `align_index_v1.csv`. Wait at least 3 s with absolutely nothing moving. **Stop Capture**.
4. Run the alignment tool:

   ```powershell
   python tools/usb_cdc_monitor/frame_alignment.py align_index_v1.csv --node 30
   ```
5. Inspect the output:
   - `palm spread` and `tip spread` should both be < 0.5 deg. Higher means something moved during the capture; redo it.
   - `raw tip-vs-palm residual` is the current misalignment in deg.
   - `simulated residual after remap` is what the math says you would have left after applying the suggested mount; should be near zero.
   - The verdict line says GOOD / OK / POOR.
6. Copy the printed `#define PALM_EXTERNAL_NODE_UART3_REMAP_*` block into [Core/Inc/imu/imu_config.h](Core/Inc/imu/imu_config.h), replacing the current values for that UART. Build and flash.
7. Repeat steps 2-4 with the new firmware. The **raw** residual on this second run should be < 2 deg. Record the result in the table at the bottom of [IMU_FRAME_REMAP.md](IMU_FRAME_REMAP.md).
8. Long-term: move the correction into the fingertip firmware repo's own raw-axis remap, then restore `PALM_EXTERNAL_NODE_UARTn_REMAP_*` to identity in this repo and re-validate.

You can also include all four boards in **one** capture if they can be laid co-aligned at once, then run `python frame_alignment.py align_all.csv` (no `--node`) and the tool prints blocks for every fingertip node it finds.

### Quick alternative: Phase 0 Auto-Detect

If the issue you suspect is "thumb's `X` is parallel to index's `Y`" or "ring's `Z` is flipped" - i.e. an axis-permutation mismatch rather than a small mounting tilt - use the auto-detect flow instead. It captures a single flat pose, classifies every fingertip's mounting against all 24 right-handed axis permutations at once, and writes a JSON report ready to send to a reviewer.

1. Lay the palm board and every fingertip board flat on the same surface, all in the same physical orientation.
2. In monitor.py, click **Phase 0 Auto-Detect (3 s)** in the `Phase 0 Frame Alignment Auto-Detect` panel. A `phase0_auto_<timestamp>.csv` and a `<csv>.phase0.json` are written to the current working directory; the popup names them.
3. (Optional, no hardware needed) Re-run on any saved CSV via **Analyze Capture (Phase 0)** in the same panel, or from the CLI:

   ```powershell
   python tools/usb_cdc_monitor/phase0_autodetect.py path\to\capture.csv
   ```

The JSON includes, per finger: the best-fitting permutation (e.g. `X <- +Yraw, Y <- -Xraw, Z <- +Zraw`), the residual, a confidence margin, and both fix forms (clean fingertip-firmware raw-axis remap line and a palm-side fallback `#define` block). Schema details: [tools/usb_cdc_monitor/README.md](tools/usb_cdc_monitor/README.md) section "Phase 0 Auto-Detect".

Use the auto-detect for diagnosis and as a sanity check; keep using `frame_alignment.py` when you want the exact mounting quaternion for the palm-side fallback `#define` block.

---

## Palm-Side Frame Correction + Zero All (Runtime)

This is the corrective layer that actually runs on the palm board today. It is what you interact with when you flash the current firmware, not a future plan.

### What landed

1. **Shared compile-time fingertip mapping** - one sandwich-conjugation quaternion applied to every forwarded fingertip fused quaternion, configured via `PALM_EXTERNAL_NODE_COMMON_MAP_*` in [Core/Inc/imu/imu_config.h](Core/Inc/imu/imu_config.h). Semantics:

   ```text
   q_forwarded = q_common * (q_uart_multiply * q_raw) * conj(q_common)
   ```

   Sandwich (not left/right multiply) is the mathematically correct operation for rotating the *frame* in which a quaternion is expressed. The existing `PALM_EXTERNAL_NODE_UARTn_REMAP_*` per-UART left/right-multiply constants are kept as a per-port fallback and remain at identity by default.

2. **Palm-side runtime Zero All** - a debug/session reference layer applied to the USB output stream only (sensor fusion itself is untouched).

   ```text
   q_palm_out = inv(q_palm_zero_ref) * q_palm_mapped
   q_tip_out  = inv(q_tip_zero_ref[port]) * q_tip_mapped
   ```

   Zero references are captured lazily: the palm latches on its first valid Mahony sample after the request, and each fingertip port latches on its next forwarded frame.

3. **Auto-zero at boot** - `main()` arms a one-shot deadline `PALM_GLOVE_ZERO_STARTUP_DELAY_MS` milliseconds after power-up (default 5000 ms). When the deadline passes, Zero All fires exactly once, giving the palm Mahony filter time to finish gyro-bias calibration and the fingertip boards time to stream stable quaternions before their references are latched.

4. **USB command group `0xC0`** - two-byte commands on USB CDC:

   | Bytes | Meaning |
   | --- | --- |
   | `0xC0 0x01` | Zero All: arm palm + all ports, capture next valid sample per source |
   | `0xC0 0x00` | Clear zero references (resume raw mapped output) |

   Commands are decoded in [Core/Src/usb_process.c](Core/Src/usb_process.c) and serviced from main-loop context via a shared `g_glove_zero_request` flag in [Core/Src/main.c](Core/Src/main.c).

### Tuning the common map

The currently shipped default is:

```c
#define PALM_EXTERNAL_NODE_COMMON_MAP_W  0.0f
#define PALM_EXTERNAL_NODE_COMMON_MAP_X -0.70710678f
#define PALM_EXTERNAL_NODE_COMMON_MAP_Y  0.70710678f
#define PALM_EXTERNAL_NODE_COMMON_MAP_Z  0.0f
```

which is `q_180Y * q_-90Z` as a single rotation and absorbs the observed "X/Y swap plus X/Z inversion" between this generation of fingertip firmware and the palm canonical frame. The header comment in [Core/Inc/imu/imu_config.h](Core/Inc/imu/imu_config.h) lists the other common presets (identity, `+/-90 deg Z`, `180 deg Y`, `180 deg Y * +90 deg Z`).

To verify after any change:

1. Clean build and flash the palm board.
2. Power-up, wait ~5 s for the deferred Zero All to fire.
3. In the 3D UI or monitor.py world-Euler view, rotate the whole glove together and confirm every node's X/Y/Z move in the same direction with the same sign as the palm.
4. Send `{0xC0, 0x00}` to compare against the raw mapped output if you want to see what the common map is doing.

### Relationship to Phase 0 tooling

`frame_alignment.py` and `phase0_autodetect.py` still produce **per-UART** fallback blocks (the `PALM_EXTERNAL_NODE_UARTn_REMAP_*` multiply constants) because that route pre-dates the common-map sandwich layer and is still the right fix when **one** fingertip board has a different physical mounting from the rest. Today's default assumes every fingertip board shares the same mismatch and the common map handles all of them in one line.

---

## Next Steps

Phase 0 now behaves correctly end-to-end: boot the palm, wait ~5 s, every node reads the canonical glove frame and zeros to identity. What comes next, in order:

1. **Cross-finger validation** - wear the glove and confirm every fingertip tracks the palm with correct axis signs under full ROM. Record any per-finger deviations; if a single board disagrees with the rest, use `phase0_autodetect.py` or `frame_alignment.py` to produce a per-UART fix for that one board and keep the common map as-is.
2. **Push the correction upstream** - fold the common-map sandwich back into the fingertip firmware's raw-axis remap so the glove is self-describing. Once that lands, zero `PALM_EXTERNAL_NODE_COMMON_MAP_*` to identity and re-validate; this is the long-term "source of truth for axis convention is firmware" target.
3. **Start Phase 1 (host-side glove math)** - build `tools/usb_cdc_monitor/glove_pipeline.py` with palm YPR, per-finger `q_rel`, swing-twist decomposition, and a per-user flat-pose calibration JSON. See [Phase 1 Design](#phase-1-design-per-user-pose-calibration--joint-math).
4. **Add a Glove panel to monitor.py** - palm YPR bars, per-finger flex/swing bars + numeric readout + validity dot, Calibrate Flat / Reset Calibration buttons, session CSV+JSONL recorder. Reuses the existing capture plumbing.

Deliberately **not** next: firmware-side joint math (Phase 3). Host math must prove out on real recordings first; firmware is a port, not the reference.

---

## Phase 1 Design (Per-User Pose Calibration + Joint Math)

Phase 0 normalizes the boards. Phase 1 normalizes the user.

### Procedure (when implemented)

1. User wears the glove and adopts the **neutral reference pose**: hand flat, fingers extended naturally, thumb in its natural relaxed position alongside the index. Palm-down, hand level.
2. User clicks **Calibrate Flat** in the Glove panel of monitor.py.
3. The monitor captures ~2 s of palm + per-finger quaternions, hemispherically averages each, and stores:
   - `q_palm_ref` (the palm at neutral)
   - per finger `m_i = conj(q_palm_ref) * q_tip_i_ref` (the natural offset of finger i relative to the palm at neutral)
4. From now on, every per-finger reading is corrected via:

   ```text
   q_rel_i             = inv(q_palm_current) * q_tip_i_current
   q_rel_calibrated_i  = inv(m_i) * q_rel_i
   ```

   `q_rel_calibrated_i` is **identity** the moment the user returns to the neutral pose, and grows as the finger curls or splays.

5. Decompose with [swing-twist](#kinematic-linkage-one-sensor-site-per-finger---many-joint-angles) around `+X`:
   - twist around `+X` -> finger roll (logged as diagnostic, ignored for visualization)
   - swing projected onto `+Y` -> **flex** (positive = curl toward palm)
   - swing projected onto `+Z` -> **swing** (positive = abduction away from middle finger)

6. Both raw `(flex_deg, swing_deg)` and the relative quaternion are logged to CSV/JSONL per frame.

### What this design intentionally does NOT do

- It does **not** treat the thumb specially. Same per-finger pipeline handles all five fingers via per-finger `m_i` offsets.
- It does **not** try to compute per-joint MCP/PIP/DIP angles from a single fingertip-board sensor site - that is mathematically underdetermined regardless of how many IMU chips are co-located there. Per-joint angles are produced downstream by a [linkage model](#kinematic-linkage-one-sensor-site-per-finger---many-joint-angles).
- It does **not** apply the calibration on the firmware until it has been validated in Python on real captures (Phase 3 only).

---

## Quaternion Math Reference

Conventions used everywhere in this project:

- Quaternion order: `(w, x, y, z)`, scalar first.
- Right-handed multiplication: `q1 * q2` means rotate by `q2` then by `q1` in the same fixed frame, equivalently rotate by `q1` first then `q2` in body frame.
- All quaternions are normalized; ambiguity `q == -q` is resolved by forcing `w >= 0` after every operation that could flip it.
- Hemispherical averaging: when averaging multiple samples, flip any sample whose dot product with the first sample is negative, then take the normalized linear mean. Adequate for small spreads (< ~30 deg).

### Core formulas

```text
# Frame alignment (Phase 0)
# Goal: q_out = q_mount * q_tip_raw  ==  q_palm_canonical when boards are co-aligned
q_mount = q_palm_canonical * conj(q_tip_raw)

# Per-finger relative orientation
q_rel = inv(q_palm) * q_tip                        # before user calibration
q_rel_calibrated = inv(m_i) * q_rel                # after user calibration

# Per-user neutral offset (captured once at neutral pose)
m_i = inv(q_palm_neutral) * q_tip_neutral

# Palm Euler (ZYX intrinsic)
yaw, pitch, roll = quaternion_to_euler(q_palm)

# Swing-twist decomposition around finger-length axis +X
# returns q_swing, q_twist such that q_rel_calibrated = q_swing * q_twist
# - q_twist is the rotation about +X
# - q_swing is the residual; its rotation axis lies in the YZ plane
# Then:
flex_deg  = component of q_swing's axis-angle along +Y
swing_deg = component of q_swing's axis-angle along +Z
twist_deg = q_twist axis-angle (logged as diagnostic only)
```

### Why swing-twist over Euler

Euler angles (e.g. extracting "pitch" from `q_rel` and calling it bend) work for small bends but suffer from gimbal lock near 90 deg on the middle axis and arbitrarily mix abduction into flex. Swing-twist decomposition cleanly separates twist (roll about the finger length) from the perpendicular rotation that actually represents flex + swing, and remains stable up to about +/-170 deg.

---

## Kinematic Linkage (One Sensor Site per Finger -> Many Joint Angles)

A real finger has up to 4 rotational DoF (MCP flex + abduction, PIP flex, DIP flex). Each fingertip board sits at a **single sensor site** (the distal phalanx) and gives 3 rotational DoF of orientation, of which only 2 are useful (flex + abduction; twist is mostly noise). The two co-located LSM6DSOW chips on the fingertip board reduce noise and let the board recover from a sensor failure, but they do not add a second sensor site - the fused output is still one orientation at one location. The system is therefore **underdetermined**: infinitely many MCP/PIP/DIP combinations can produce the same fingertip orientation. We resolve it with a fixed coupling, the way Leap Motion and most sparse-sensor data gloves do.

### The linkage model

Define one curl scalar per finger, `theta_curl in [0, 1]`. For each joint, define a fixed share `c_*` and a per-user max angle `curl_max_*`:

```text
theta_MCP_flex = c_MCP * curl_max_MCP * theta_curl
theta_PIP_flex = c_PIP * curl_max_PIP * theta_curl
theta_DIP_flex = c_DIP * curl_max_DIP * theta_curl
theta_MCP_abd  = swing_measured              # abduction stays its own DoF
```

Default ratios (tunable per finger):

| Finger             | MCP share | PIP share | DIP share | Typical max angles      |
| ---                | ---       | ---       | ---       | ---                     |
| index/middle/ring  | ~30 %     | ~50 %     | ~20 %     | MCP 90, PIP 110, DIP 70 |
| thumb              | 50 % MCP, 50 % IP | -  | -         | varies per user         |
| pinky              | similar to ring | -   | -         | similar to ring         |

### Solving for theta_curl

The IMU's measured flex is the cumulative chain rotation:

```text
flex_measured = (c_MCP * curl_max_MCP + c_PIP * curl_max_PIP + c_DIP * curl_max_DIP) * theta_curl
theta_curl    = clip(flex_measured / total_chain_max, 0, 1)
```

Then propagate `theta_curl` back to the per-joint angles using the ratios. This is exactly the "low-DoF robotic finger driven by one input" model.

### Where the linkage lives

**The linkage is a renderer-side concern, not a measurement-side concern.** Rationale:

- The linkage ratios may be tuned later. The raw measurement does not change.
- Multiple consumers may want different linkage models. Recording per-joint angles bakes one choice into every recording forever.
- A future 2-IMU-per-finger upgrade adds proximal measurements that constrain the linkage; old recordings still work because they kept the raw cumulative measurement.

So `glove_pipeline.py` records and outputs raw `(flex_deg, swing_deg, q_rel)` per finger. A separate `finger_linkage.py` (planned) provides `expand_to_joints(flex, swing, finger_name) -> {mcp_flex, mcp_abd, pip_flex, dip_flex}` that the Qt 3D UI calls at render time, with a `linkage_id` string for versioning.

---

## Host <-> Qt 3D UI Data Contract

The Qt 3D UI lives in a separate repo and consumes a local TCP stream. One JSON record per line, default port `127.0.0.1:8765`. This is the schema (Phase 2 + Phase 3 fields):

```json
{
  "schema_version": "1.0",
  "ts_ms": 123456,
  "handedness": "right",
  "calibrated": true,
  "calibration_id": "<sha8 of calibration.json>",
  "linkage_id": "default_v1",
  "status": 0,

  "palm": {
    "quat": [w, x, y, z],
    "ypr_deg": [yaw, pitch, roll]
  },

  "fingers": [
    {
      "name": "thumb",
      "node": 20,
      "valid": true,
      "quat_rel":  [w, x, y, z],
      "flex_deg":  12.3,
      "swing_deg": 25.0,
      "twist_deg": 0.4,
      "joints": {
        "mcp_flex_deg": 6.1,
        "mcp_abd_deg":  25.0,
        "pip_flex_deg": 0.0,
        "dip_flex_deg": 0.0
      }
    }
    // ... index (30), middle (40), ring (50), pinky (60)
  ]
}
```

Rules:

- The **raw** fields (`quat_rel`, `flex_deg`, `swing_deg`, `twist_deg`) are always present and authoritative.
- The `joints` block is **derived** via the linkage and may differ between `linkage_id` versions; consumers that want to apply their own linkage should ignore it.
- Pinky is included with `valid: false` until the UART6 board is online.
- Schema version is bumped only on breaking changes; additive fields do not bump it.

---

## File Map

This section lists what currently exists and what is planned, with a one-line purpose for each file. Use it as a navigation aid.

### Firmware (palm board)

| File | Purpose | Status |
| --- | --- | --- |
| [Core/Src/main.c](Core/Src/main.c) | Top-level `main()`, peripheral init, super-loop; arms the deferred startup Zero All and services `g_glove_zero_request` from USB | exists |
| [Core/Src/app/palm_runtime.c](Core/Src/app/palm_runtime.c) | Reads palm IMUs, applies axis remap, gyro bias, Mahony fusion, builds `0xB6` frame; owns the palm-side zero reference (`palm_runtime_request_zero` / `palm_runtime_clear_zero`) | exists |
| [Core/Src/app/external_node_rx.c](Core/Src/app/external_node_rx.c) | UART2-7 receive of fingertip fused frames (each fingertip board emits one fused quaternion derived from its own dual-IMU fusion), applies `PALM_EXTERNAL_NODE_UARTn_REMAP_*` (per-UART multiply) then the shared `PALM_EXTERNAL_NODE_COMMON_MAP_*` sandwich, per-port zero rebasing, retags `node_id`, forwards to USB | exists |
| [Core/Src/usb_process.c](Core/Src/usb_process.c) | USB CDC RX command parser. Handles legacy 2-byte commands plus `{0xC0, 0x01}` Zero All / `{0xC0, 0x00}` clear-zero | exists; needs `0xC1` per-user flat-calibration opcode in Phase 3 |
| [Core/Src/imu/lsm6dsow.c](Core/Src/imu/lsm6dsow.c) | LSM6DSOW driver | exists |
| [Core/Src/imu/mahony_filter.c](Core/Src/imu/mahony_filter.c) | 6DOF Mahony fusion | exists |
| [Core/Src/protocol/palm_protocol.c](Core/Src/protocol/palm_protocol.c) | `0xB6` and `0xD6` frame builders / parsers | exists |
| [Core/Inc/imu/imu_config.h](Core/Inc/imu/imu_config.h) | All tuning constants, axis remaps, per-UART fallback mounting quaternions, shared `PALM_EXTERNAL_NODE_COMMON_MAP_*`, `PALM_GLOVE_ZERO_AT_STARTUP`, `PALM_GLOVE_ZERO_STARTUP_DELAY_MS` | exists - edit this for Phase 0 results |
| [Core/Inc/global.h](Core/Inc/global.h) | Cross-module constants incl. `GLOVE_ZERO_CMD`/`GLOVE_ZERO_ALL_SUBCMD`/`GLOVE_ZERO_CLEAR_SUBCMD` and the `GLOVE_ZERO_REQUEST_*` main-loop service flag values | exists |
| `Core/Src/app/glove_calibration.c` | Stores `q_palm_ref` + per-node `m_i` | planned (Phase 3) |
| `Core/Src/app/glove_joint_math.c` | Port of Python flex/swing math; builds `0xA6` frame | planned (Phase 3) |

### Host tooling (Python)

| File | Purpose | Status |
| --- | --- | --- |
| [tools/usb_cdc_monitor/monitor.py](tools/usb_cdc_monitor/monitor.py) | Tkinter UI: serial monitor, per-node fused view, drift metrics, capture, isolation/pose tests | exists; gets a Glove panel in Phase 1 |
| [tools/usb_cdc_monitor/palm_parser.py](tools/usb_cdc_monitor/palm_parser.py) | Stateless `0xD6` and `0xB6` parser + streaming `FrameParser` | exists; gets `0xA6` in Phase 3 |
| [tools/usb_cdc_monitor/isolation_analysis.py](tools/usb_cdc_monitor/isolation_analysis.py) | CLI to confirm finger-to-UART mapping from per-finger isolation captures | exists |
| [tools/usb_cdc_monitor/frame_alignment.py](tools/usb_cdc_monitor/frame_alignment.py) | **Phase 0 tool**: from a flat-pose CSV, computes mounting quat + ready-to-paste `#define` block | exists |
| [tools/usb_cdc_monitor/phase0_autodetect.py](tools/usb_cdc_monitor/phase0_autodetect.py) | **Phase 0 auto-detect**: classifies every fingertip's mounting as one of the 24 right-handed axis permutations, emits a JSON report for human/agent review | exists |
| [tools/usb_cdc_monitor/README.md](tools/usb_cdc_monitor/README.md) | Tool-specific docs | exists |
| `tools/usb_cdc_monitor/glove_pipeline.py` | Palm YPR + per-finger flex/swing + per-user calibration apply | planned (Phase 1) |
| `tools/usb_cdc_monitor/glove_stream_server.py` | TCP JSON-line stream for the Qt 3D UI | planned (Phase 2) |
| `tools/usb_cdc_monitor/finger_linkage.py` | Linkage model, `expand_to_joints()` | planned (Phase 1.5) |
| `tools/usb_cdc_monitor/GLOVE_STREAM.md` | External data contract spec | planned (Phase 2) |

### Documentation

| File | Purpose |
| --- | --- |
| [GLOVE_PROJECT.md](GLOVE_PROJECT.md) | This file. Project plan + math + ops + glossary |
| [IMU_FRAME_REMAP.md](IMU_FRAME_REMAP.md) | Canonical glove frame, per-board mapping, Phase 0 procedure, validation table |

---

## Glossary

| Term | Meaning |
| --- | --- |
| **Canonical glove frame** | `+X` along fingers, `+Y` left across hand (toward thumb on right hand), `+Z` up |
| **PON (Point Of Navigation)** | The hand reference IMU. In our v1 = the palm board (back of hand) |
| **POM (Point Of Measurement)** | A per-finger sensor *site* on the finger. In v1 we only have the **fingertip** POM (one site per finger, even though the fingertip board carries 2 fused LSM6DSOW chips). Future upgrades may add a knuckle POM (proximal phalanx) and middle POM, which would constrain MCP/PIP/DIP individually |
| **Sensor site** | A physical location on the hand where a board is mounted. Each site emits one fused quaternion regardless of how many IMU chips are co-located there. v1 has 5 sites: 1 palm + 4 fingertips (5th fingertip planned) |
| **MCP** | Metacarpophalangeal joint = the knuckle. Flexion + abduction (2 DoF) |
| **PIP** | Proximal interphalangeal joint = middle finger joint. Flexion only (1 DoF) |
| **DIP** | Distal interphalangeal joint = the joint nearest the fingertip. Flexion only (1 DoF) |
| **IP** (thumb) | The thumb's single interphalangeal joint, equivalent of PIP+DIP combined |
| **Flex** | Curl of the finger toward the palm. Always >= 0 in normal use |
| **Swing / abduction** | Sideways spread at the MCP. Positive = away from middle finger |
| **Twist** | Rotation around the finger-length axis. Mostly sensor noise; logged as diagnostic |
| **Phase 0** | Permanent board-frame alignment (PCB mounting offset). One-time per board |
| **Phase 1** | Per-session per-user neutral-pose calibration |
| **Linkage / kinematic synergy** | Fixed-ratio coupling that turns one measured curl angle into MCP/PIP/DIP angles |
| **Node ID** | 8-bit ID the palm assigns to each forwarded fused frame. `0` = palm, `20-29` = UART2 chain, `30-39` = UART3, etc. |
| **Same-pose residual** | Angular distance between the palm's fused quat and a fingertip's fused quat when both boards are physically co-aligned. Target < 2 deg after Phase 0 |

---

## FAQ / Gotchas

**Q: I ran Phase 0 with the glove on my hand, with all five boards on the table of my hand. Why are the angles still wrong?**
A: Phase 0 must be done with the boards in the *same physical orientation*, not "all on the hand". A hand has each fingertip pointing in a slightly different direction, so the boards are not co-aligned. Take the boards off the glove (or take the glove off the hand), lay them on a table all pointing the same direction, then capture.

**Q: My thumb reads about 30 degrees of swing in neutral pose. Is Phase 0 broken?**
A: No - that is the natural splay of the thumb relative to the palm. Phase 0 only removes the board's mounting offset, not your hand's geometry. Phase 1 ("Calibrate Flat" in the Glove panel) absorbs that natural splay into `m_thumb` so neutral reads ~0 deg.

**Q: I want per-joint angles (MCP, PIP, DIP) directly out of the pipeline. Why are we only outputting flex+swing?**
A: With only one *sensor site* per finger (the fingertip board, even though it carries two co-located IMU chips for redundancy), per-joint angles are mathematically underdetermined - infinitely many MCP/PIP/DIP combinations produce the same fingertip orientation. Adding a second IMU chip at the same location does not help; you would need a sensor at a second *site* on the finger (e.g. the proximal phalanx) to constrain the joints individually. The linkage model (renderer side) is the standard fix: it imposes a fixed ratio that distributes the measured flex across joints. Recording the raw measurement keeps the dataset honest and lets you swap linkage models without re-collecting data.

**Q: Where do I add a hardware calibration button?**
A: Not in v1 - only `NRST` is free on the palm board, and we cannot repurpose it. Calibration triggers in v1 are the PC command (runtime Zero All on `0xC0, 0x01`, per-user flat calibration on `0xC1, 0x01` when Phase 3 lands) and auto-startup. If a future board revision frees a GPIO, wire it as EXTI to the same trigger path the PC command uses.

**Q: Yaw drifts on the palm. Will that ruin per-finger angles?**
A: No - per-finger angles use `inv(q_palm) * q_tip`, which cancels any global rotation including yaw drift. The drift only shows up in the **palm YPR** output, which is a separate field. The Qt 3D UI should always render fingers from the relative quaternions, never from world-frame fingertip quaternions.

**Q: I added a fifth board (pinky) on UART6. Where do I configure it?**
A: The firmware already plumbs UART6 / node block 60-69. On the host side, the Phase 1 pipeline is finger-list-driven (planned), so you just append a `("pinky", 60, "UART6")` entry to the finger list. The JSON schema already reserves the slot.

**Q: I changed the linkage ratios. Are my old recordings invalid?**
A: No, because the recordings store the raw `(flex_deg, swing_deg, q_rel)`. The linkage is applied at render time. Bump the `linkage_id` string so consumers that care about the version can branch on it.

**Q: Phase 0 verdict says POOR for one board even after several captures. What now?**
A: That usually means the IMU's axis on the PCB cannot be expressed as a single rotation - there is an axis swap (e.g., sensor `+X` is the board's `+Y` AND its `+Z` is inverted). A single mounting quaternion can express any rotation, so it should still work; the most common cause of POOR is movement during capture (check the `palm spread` / `tip spread` numbers in the tool output). Failing that, the firmware-side raw-axis remap is the proper fix.

---

## Decision Log

Architecturally significant decisions recorded so they are not re-litigated by accident.

- **Right-hand glove only in v1.** Left-hand variant is supported by inverting `+Y` and is wired into the data contract via a `handedness` field; it is just not built.
- **One sensor *site* per finger in v1** (the fingertip board, which itself carries 2x LSM6DSOW fused on-board for noise/redundancy). Adding a second sensor site on the finger (e.g. a proximal-phalanx board) is a future upgrade; the data contract and pipeline are designed so adding it does not break recordings.
- **Glove math runs on both PC and MCU.** PC first (Phase 1) for fast iteration; MCU later (Phase 3) for low-latency / standalone use. Python is the reference implementation; firmware must match.
- **Calibration triggers: PC command + auto-startup. No hardware button.** Only `NRST` is free.
- **Linkage model is renderer-side, not measurement-side.** Keeps the dataset reproducible across linkage tuning.
- **Palm board is the universal hand reference for all fingers.** Not "index relative to thumb" or any other peer-finger scheme.
- **Frame alignment tool is offline / CSV-driven.** Reuses the existing `monitor.py` capture path instead of duplicating serial I/O.
- **Source of truth for axis convention is firmware**, not the host app. Host downstream is identity-only.
- **Palm-side common-map uses sandwich conjugation**, not a plain left/right multiply. Sandwich (`q_common * q * conj(q_common)`) is the correct operation for rotating the *frame* a quaternion is expressed in; left/right multiply would compose an extra rotation on top of the fingertip pose instead.
- **Runtime Zero All is USB-output-only**, applied after Mahony + remap + common-map and before the `0xB6` frame is built. Sensor fusion stays untouched so clearing the zero reference returns clean raw mapped output. It is a debug/session convenience, not a replacement for Phase 0 (board alignment) or Phase 1 (per-user neutral pose).
- **Auto-zero at boot is deferred**, not immediate. Palm Mahony needs ~4 s for gyro-bias calibration and fingertip boards need a moment to stream stable quaternions; the default 5 s window in `PALM_GLOVE_ZERO_STARTUP_DELAY_MS` gives both. Capture stays lazy per-source so a quiet port simply waits for its first real frame.
- **Python is the source of truth for joint math.** Firmware port in Phase 3 must agree on captured recordings; if it does not, firmware is the bug.

---

*Last updated after palm-side common map + runtime Zero All + auto-zero-at-boot landed; Phase 0 exits, Phase 1 is next. Update this file whenever a phase advances or a decision changes.*
