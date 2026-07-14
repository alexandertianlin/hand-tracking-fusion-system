# Qt UI Agent Prompt

Project: `c:\Users\ooiky\Documents\Github\Gesture glove UI\Gesture_glove`

Goal: prepare the Qt UI to consume Phase 1 host-computed glove values, without changing firmware assumptions.

Context:

- The firmware side already streams fused quaternions per node via `0xB6`.
- Node IDs:
  - palm = `0`
  - thumb = `20`
  - index = `30`
  - middle = `40`
  - ring = `50`
  - pinky = `60` later
- Phase 1 math is host-first, not firmware-first.
- The measurement model is:
  - `q_rel_i = inv(q_palm) * q_tip_i`
  - `m_i = inv(q_palm_ref) * q_tip_i_ref` during neutral calibration
  - `q_rel_calibrated_i = inv(m_i) * q_rel_i`
  - derive `flex/swing/twist` from swing-twist decomposition around `+X`
- Do not use raw quaternion components or plain relative Euler as the final bend metric.
- Thumb uses the same relative quaternion + calibration pipeline as other fingers. Only later linkage/joint distribution is thumb-specific.

Existing host-side control commands:

- `Zero All`: send bytes `{0xC0, 0x01}`
- `Clear Zero`: send bytes `{0xC0, 0x00}`
- These are palm-side USB CDC commands only.
- `Zero All` tells the palm to latch the next valid palm sample and the next valid forwarded sample per port as zero references.
- `Clear Zero` removes those zero references and returns to raw mapped output.

What to inspect first:

- `src/app/applicationcontext.cpp`
- `src/domain/sensorstatestore.cpp`
- `src/kinematics/relativeposesolver.cpp`
- `src/viewmodels/handposeviewmodel.cpp`
- `src/viewmodels/imudebugviewmodel.cpp`
- `qml/scene/HandScene.qml`
- `qml/components/DiagnosticsPanel.qml`

What to plan/build:

1. Add a clear per-finger data model for `thumb/index/middle/ring/(pinky placeholder)` carrying at least:
   - `active/valid`
   - `relative quaternion`
   - `flex_deg`
   - `swing_deg`
   - `twist_deg`
2. Extend the existing multi-node path so the UI can display all available fingers, not just palm + index.
3. Add a Glove panel or diagnostics section showing:
   - palm YPR
   - per-finger flex/swing numeric values
   - calibration state / validity indicator
   - `Zero All` and `Clear Zero` actions using the exact commands above
4. Update the 3D scene so it can preview a simple hand pose driven by per-finger `flex/swing` values, even if the first version uses a coarse linkage.
5. Keep the current IMU debug views available for validation.

Constraints:

- Treat host-derived `flex/swing/twist` as the authoritative Phase 1 UI input.
- Keep architecture ready for a later external stream or shared host contract.
- Keep thumb in the common finger pipeline; do not branch special quaternion math for thumb.
- Do not move the measurement math into firmware yet. Firmware is a later port only after host outputs are validated on real captures.
- Do not claim to derive exact MCP/PIP/DIP anatomical angles from one fingertip sensor site. Any joint breakdown should be renderer-side linkage only.

Expected output from you:

- A concrete file-by-file implementation plan for the Qt repo
- The data structures / viewmodels you would add or change
- Any UI assumptions or blockers you need clarified before coding
