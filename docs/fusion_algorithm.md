# Multi-Modal Fusion Algorithm: IMU × Vision × Tactile

## 1. Problem Statement

Wearable IMU sensors provide **high-frequency (100 Hz)** finger orientation data but suffer from **long-term drift**. Camera-based hand pose estimation (HaMeR) provides **accurate absolute orientation** but at **low frequency (~7.4 FPS)** and is **occlusion-sensitive**. The goal is to fuse these modalities to achieve: drift-free, occlusion-robust, high-frequency hand pose estimation.

## 2. Coordinate Systems

Three coordinate systems must be aligned:

| System | Axes | Notes |
|---|---|---|
| **Camera space** (HaMeR output) | Z-forward, Y-down, X-right (right-handed) | MANO `global_orient` output, 3×3 rotation matrix → quaternion |
| **IMU sensor space** | Body-relative, via MPU6050 quaternion output | Each finger has its own IMU quaternion stream |
| **Unity space** (target) | Z-forward, Y-up, X-right (left-handed) | Unity uses left-handed coordinate system |

### Coordinate Alignment

The alignment is achieved through a **calibration offset quaternion** $Q_{\text{offset}}$:

$$Q_{\text{offset}} = Q_{\text{imu}}^{-1} \cdot Q_{\text{visual}}$$

where $Q_{\text{imu}}$ and $Q_{\text{visual}}$ are sampled simultaneously during calibration (open palm facing camera, static).

The calibrated IMU quaternion is:

$$Q_{\text{imu,aligned}} = Q_{\text{imu}} \cdot Q_{\text{offset}}$$

### Calibration Protocol

1. User places hand flat, palm facing depth camera camera
2. System collects **15 frames** of simultaneous IMU + HaMeR data
3. Compute $Q_{\text{offset}}$ per finger from averaged quaternions
4. Store offset in calibration database
5. Rate-limit: auto-calibration triggers once per 3 seconds (via "fully open hand" gesture detection: all 5 finger curl values < 0.3)

## 3. Fusion Method A: Per-Finger Confidence-Gated Modal Switching

### 3.1 Mechanism

Each finger independently selects its orientation source based on the **vision confidence score** $c_i$ for finger $i$:

$$Q_i^{\text{fused}} = \begin{cases} Q_i^{\text{visual}} & \text{if } c_i \geq \theta_{\text{high}} \\ Q_i^{\text{imu,aligned}} & \text{if } c_i < \theta_{\text{low}} \\ \text{Slerp}(Q_i^{\text{imu}}, Q_i^{\text{visual}}, \alpha_i) & \text{if } \theta_{\text{low}} \leq c_i < \theta_{\text{high}} \end{cases}$$

where:
- $\theta_{\text{low}} = 0.3$ (below this, vision is considered unreliable)
- $\theta_{\text{high}} = 0.7$ (above this, vision is considered reliable)
- Intermediate confidence → smooth blend via Slerp

### 3.2 Confidence Source

HaMeR outputs per-keypoint confidence scores (0–1) for 21 MANO keypoints. Per-finger confidence $c_i$ is computed as the **minimum confidence** across the 4 keypoints belonging to finger $i$:

$$c_i = \min(c_{i,\text{MCP}}, c_{i,\text{PIP}}, c_{i,\text{DIP}}, c_{i,\text{TIP}})$$

This ensures that even one unreliable keypoint on a finger triggers the IMU fallback.

### 3.3 Vision Hold Watchdog

A short "IDLE" state (confidence temporarily dropping) should not cancel an ongoing correction. The watchdog holds the vision takeover for **N frames** after confidence drops below threshold:

$$\text{takeover}_i(t) = \text{takeover}_i(t-1) \lor (c_i \geq \theta) \quad \text{hold for } N = 5 \text{ frames}$$

## 4. Fusion Method B: Slerp Complementary Filter

### 4.1 Core Equation

The Slerp (Spherical Linear Interpolation) complementary filter blends IMU and visual quaternions:

$$Q_i^{\text{fused}} = \text{Slerp}(Q_i^{\text{imu,aligned}}, Q_i^{\text{visual}}, \alpha_i)$$

where $\alpha_i$ is the **dynamic blending weight** ranging from 0.85 (vision-reliable) to 1.0 (pure IMU).

**Convention**: $\alpha = 0$ → pure visual, $\alpha = 1$ → pure IMU.

### 4.2 Dynamic α Scheduling

$$\alpha_i = \begin{cases} \alpha_{\text{low}} = 0.85 & \text{if } c_i \geq c_{\text{high}} = 0.7 \\ \alpha_{\text{high}} = 1.0 & \text{if } c_i < c_{\text{low}} = 0.5 \\ \alpha_{\text{low}} + \frac{(\alpha_{\text{high}} - \alpha_{\text{low}})(c_{\text{high}} - c_i)}{c_{\text{high}} - c_{\text{low}}} & \text{if } 0.5 \leq c_i < 0.7 \end{cases}$$

Linear interpolation in the intermediate range ensures smooth transitions between modalities.

### 4.3 Slerp Implementation

$$\text{Slerp}(Q_1, Q_2, \alpha) = \frac{\sin((1-\alpha)\Omega)}{\sin(\Omega)} Q_1 + \frac{\sin(\alpha \Omega)}{\sin(\Omega)} Q_2$$

where $\Omega = \arccos(Q_1 \cdot Q_2)$ is the angle between the two quaternions.

**Edge case**: When $\Omega$ is very small (< 10⁻⁶), fall back to linear interpolation to avoid division by zero:

$$\text{Lerp}(Q_1, Q_2, \alpha) = (1-\alpha) Q_1 + \alpha Q_2 \quad \text{(then normalize)}$$

## 5. Orientation-Gated Vision Correction

### 5.1 Three-State Orientation Gate

Vision corrections are only accepted when the hand orientation relative to the camera is favorable:

| Orientation | Gate | Correction Policy |
|---|---|---|
| **Palm facing camera** (θ < 30°) | OPEN | Full correction: all fingers use visual anchor |
| **Back of hand facing camera** (θ > 150°) | PARTIAL | Only accept "open/fist" classification for drift refresh |
| **Side facing** (30° < θ < 150°) | CLOSED | No vision intervention — pure IMU mode |

The orientation θ is estimated from the palm normal vector computed from HaMeR's MANO output.

### 5.2 Open-Palm Drift Refresh

When the system detects a fully open palm (all 5 finger curl values < 0.3) with palm facing camera:
1. Trigger auto-calibration (recalculate $Q_{\text{offset}}$)
2. Reset IMU drift baseline
3. Rate-limit: once per 3 seconds

This provides periodic drift correction without continuous vision dependence.

## 6. Occlusion Handling

### 6.1 Occlusion Detection

The occlusion detector monitors vision confidence per finger:

$$\text{occluded}_i = (c_i < \theta_{\text{low}}) \land (\text{duration} > T_{\text{min}} = 0.5\text{s})$$

A finger is considered occluded only if low confidence persists for > 0.5 seconds (to avoid triggering on brief detection drops).

### 6.2 IMU Propagation During Occlusion

When a finger is occluded, the system switches to **pure IMU mode**:

$$Q_i^{\text{fused}} = Q_i^{\text{imu,aligned}} \quad \text{while } \text{occluded}_i$$

Upon vision recovery ($c_i \geq \theta_{\text{high}}$ for 3 consecutive frames), the system smoothly transitions back via Slerp with exponentially decaying α:

$$\alpha_i(t) = \alpha_{\text{low}} + (1 - \alpha_{\text{low}}) \cdot e^{-\lambda (t - t_{\text{recovery})}}, \quad \lambda = 0.5$$

This prevents sudden orientation jumps when vision reappears.

## 7. Data Flow Summary

```
depth camera (640×480 @15fps) ──→ ViTPose (every 6 frames) ──→ bbox tracking
                                                              │
                                                              ▼
                                                         HaMeR (MANO)
                                                              │
                                              global_orient (3×3 → Q_visual)
                                              21 keypoints + confidence
                                                              │
IMU Glove (6×MPU6050 @100Hz) ──→ Q_imu + force ──→ Q_offset alignment
                                                              │
                                                              ▼
                                                    FusionEngine
                                                    ┌─────────────────────┐
                                                    │ Method A: Gated     │
                                                    │   c_i ≥ 0.3 → vis  │
                                                    │   c_i < 0.3 → IMU  │
                                                    │                     │
                                                    │ Method B: Slerp     │
                                                    │   α = 0.85–1.0      │
                                                    │   dynamic by c_i    │
                                                    │                     │
                                                    │ Gate: orientation   │
                                                    │   palm/back/side    │
                                                    │                     │
                                                    │ Occlusion: IMU-only │
                                                    │   recovery: e-decay │
                                                    └─────────────────────┘
                                                              │
                                                              ▼
                                                    UDP JSON (8080/5055)
                                                              │
                                                              ▼
                                                    Unity FusionHandController
                                                    per-finger visualization
```

## 8. Tactile/Force Channel

The tactile/force data from the glove is transmitted alongside IMU quaternions in the serial packet:

- **Format**: Each finger's tactile + force values included in the STM32 serial stream
- **Rate**: Same as IMU (100Hz)
- **Current use**: Force-grid visualization in Unity; tactile data available for future world-action model integration
- **Future**: Per-finger tactile readings will be used as contact-event labels (on/off/slip) for the tactile world-action model training

## 9. Evaluation Metrics (To Be Implemented)

| Metric | Definition | Ground Truth Source |
|---|---|---|
| **MPJPE** | Mean Per-Joint Position Error (mm) | MANO mesh vertices vs annotated 3D positions |
| **PA-MPJPE** | Procrustes-Aligned MPJPE (mm) | Same, after rigid alignment (removes global pose) |
| **Drift error** | Angular deviation over time (degrees) | Long-duration static hold → measured drift over 60s |
| **Fusion improvement** | Drift-only vs fused drift reduction (%) | IMU-only 60s vs fused 60s |
| **Per-finger F1** | Open/fist classification accuracy per finger | Human annotation of finger states |
| **Occlusion recovery** | Time to stable pose after occlusion ends (ms) | Simulated occlusion → recovery latency |

---

*This document consolidates the fusion algorithms from 6 separate GitHub repos into a single reference. Source repos: vision-tactile-fusion, vision-aided-imu-gesture-glove, vitpose-imu-glove-unity, agiletact, vision-imu-gesture-glove, hamer.*
