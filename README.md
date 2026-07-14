# Hand Tracking Fusion System

> **Real-time multi-modal 3D hand pose estimation** fusing Intel depth camera camera (HaMeR/ViTPose) with wearable IMU+tactile glove, visualized in Unity.

[![System Demo](docs/images/system-demo-placeholder.png)](docs/architecture.md)

---

## What This System Does

This system solves a fundamental problem in hand tracking: **IMU sensors drift over time, but camera-based tracking is slow and occlusion-sensitive**. We fuse both modalities — and tactile/force data from the glove — to achieve **drift-free, occlusion-robust, high-frequency 3D hand pose estimation** with per-finger granularity.

### Key Results

| Metric | Value | Notes |
|---|---|---|
| Full pipeline latency | ~135ms | → **7.4 FPS** effective |
| IMU streaming rate | 100 Hz | 6× MPU6050, serial/WiFi |
| Vision (HaMeR) rate | ~7.4 FPS | ViTPose every 6 frames + HaMeR per frame |
| Fusion method | Slerp complementary | Dynamic α = 0.85–1.0 by vision confidence |
| Confidence threshold | 0.3 (per-finger) | Below → IMU fallback; above → vision anchor |
| Calibration | Auto via open-palm | 15-frame offset, once per 3s |

> ⚠️ **Evaluation metrics (MPJPE, PA-MPJPE, drift reduction) are pending** — see [results/README.md](results/) for the evaluation plan.

---

## System Architecture

```
depth camera Camera ──→ ViTPose (hand detection) ──→ HaMeR (MANO 3D mesh)
                                                     │
                                                     │ global_orient quaternion + 21 keypoints + confidence
                                                     ▼
IMU+Tactile Glove ──→ STM32 (serial/WiFi) ──→ Quaternion + Force per finger @100Hz
                                                     │
                                                     ▼
                                              FusionEngine
                                              ┌──────────────────────────────────┐
                                              │ Slerp Complementary Filter        │
                                              │   Q_fused = Slerp(Q_imu, Q_vis, α)│
                                              │   α = 0.85 (vision reliable)      │
                                              │   α → 1.0 (vision occluded)       │
                                              │                                    │
                                              │ Per-Finger Confidence Gating       │
                                              │   c ≥ 0.3 → vision anchor         │
                                              │   c < 0.3 → IMU fallback          │
                                              │                                    │
                                              │ Orientation Gate                   │
                                              │   Palm → full correction          │
                                              │   Back → open/fist only           │
                                              │   Side → no intervention          │
                                              │                                    │
                                              │ Occlusion → IMU propagation       │
                                              │   Recovery → exponential decay α  │
                                              └──────────────────────────────────┘
                                                     │
                                                     ▼
                                              UDP JSON (port 8080 + 5055)
                                                     │
                                                     ▼
                                              Unity FusionHandController
                                              per-finger 3D visualization
```

See [docs/fusion_algorithm.md](docs/fusion_algorithm.md) for full mathematical formulation.

---

## Repository Structure

```
hand-tracking-fusion-system/
├── README.md                    ← This file
├── LICENSE                      ← MIT License
├── CITATION.cff                 ← Citation metadata
├── python/                      ← Python inference & fusion scripts
│   ├── run_vitpose_v3.py        ← Main pipeline (ViTPose + HaMeR → UDP)
│   ├── fusion_pipeline.py       ← IMU+Visual Slerp fusion (agiletact)
│   ├── depth_camera_hamer_fusion.py    ← HaMeR + IMU local visualization
│   ├── mediapipe_udp_sender.py  ← MediaPipe backup pipeline
│   ├── realtime_vitpose_cuda.py ← ViTPose CUDA ONNX inference
│   ├── run_hamer_camera.py      ← Standalone HaMeR inference
│   ├── test_accuracy.py         ← Accuracy evaluation (pending)
│   ├── measure_latency.py       ← Latency benchmarking
│   └── requirements.txt         ← Python dependencies
├── unity/                       ← Unity project (to be merged)
│   ├── Assets/Scripts/          ← C# scripts (VisionBridge, FusionHandController, etc.)
│   ├── Assets/Scenes/           ← Unity scenes
│   ├── Packages/
│   └── ProjectSettings/
├── firmware/                    ← STM32 glove firmware (to be merged)
│   ├── STM32H523_I2C/           ← Serial variant
│   ├── STM32H70B_WIFI/          ← WiFi variant
│   └── README.md                ← Firmware documentation
├── data/                        ← Released data samples (to be collected)
│   ├── samples/                 ← 5–10 recorded IMU+vision sequences
│   ├── calibration/             ← Calibration data samples
│   └── annotations/             ← Ground truth annotations
├── configs/                     ← Configuration files
│   ├── camera.yaml              ← Camera settings
│   ├── fusion_config.yaml       ← Fusion parameters
│   └── calibration.yaml         ← Calibration config
├── docs/                        ← Documentation
│   ├── fusion_algorithm.md      ← ✅ Full math formulation (complete)
│   ├── PROJECT_REPRODUCTION.md  ← Reproduction guide (to be merged)
│   ├── DEPLOYMENT_GUIDE.md      ← Deployment instructions (to be merged)
│   ├── architecture.md          ← System architecture overview (to be merged)
│   └── firmware_guide.md        ← Firmware setup guide (to be written)
├── results/                     ← Evaluation results (pending)
│   ├── latency_analysis.md      ← Latency benchmarks
│   ├── accuracy_evaluation.md   ← Accuracy results (pending)
│   └── fusion_comparison.md     ← Fusion comparison results (pending)
│   └── README.md                ← Evaluation plan
├── hamer_source/                ← HaMeR source code + ViTPose configs
├── models/                      ← Model weight references & download links
├── benchmark/                   ← Benchmark scripts
└── .gitignore
```

> **Note**: Files marked "to be merged" and "to be collected" need to be migrated from the 6 source repos and collected from hardware sessions. See the consolidation plan in `../GitHub仓库整合与优化计划.md`.

---

## Quick Start

### Prerequisites

| Component | Specification |
|---|---|
| GPU | NVIDIA 8GB+ VRAM (RTX 4080 Laptop 12GB / RTX 4060 Laptop 8GB validated) |
| Camera | Intel RealSense depth camera (640×480 @15 FPS) |
| Sensor glove | Custom STM32H523/H70B + 6× MPU6050 IMU + tactile/force sensors |
| Software | Windows 10/11, CUDA 12.x, Python 3.10, Unity 2022.x |

### Installation

```bash
# 1. Create conda environment
conda create -n hamer python=3.10 -y
conda activate hamer

# 2. Install Python dependencies
pip install -r python/requirements.txt

# 3. Download model weights (see docs/PROJECT_REPRODUCTION.md)
#    - HAMER checkpoint (~300MB): https://huggingface.co/geopavlakos/hamer
#    - ViTPose+ wholebody (~2.3GB): https://github.com/ViTAE-Transformer/ViTPose/releases
#    - MANO model: included in HaMeR package
```

### Running

```bash
# Connect hardware: depth camera (USB) + IMU glove (COM port)

# Option A: Full fusion pipeline (recommended)
python python/fusion_pipeline.py

# Option B: Vision-only pipeline (HaMeR + ViTPose)
python python/run_vitpose_v3.py

# Option C: Standalone HaMeR camera inference
python python/run_hamer_camera.py

# Then: Open Unity scene → Play
# Unity receives fused data via UDP (port 8080 + 5055)
```

---

## Fusion Methods

Two complementary fusion methods are implemented:

### Method A: Per-Finger Confidence-Gated Switching
- Vision confidence ≥ 0.3 → use HaMeR visual anchor for that finger
- Vision confidence < 0.3 → fall back to IMU for that finger
- Each finger independently selects its source — no single-modality global lock

### Method B: Slerp Complementary Filter
- Smooth blend: `Q_fused = Slerp(Q_imu, Q_visual, α)`
- Dynamic α: 0.85 (vision reliable, confidence ≥ 0.7) → 1.0 (pure IMU, occluded)
- Linear interpolation in intermediate range (0.5 ≤ c < 0.7)
- Coordinate alignment via calibration offset quaternion

Both methods include: **orientation gating** (palm/back/side), **open-palm auto-drift refresh**, **vision hold watchdog** (5-frame hold), and **occlusion propagation with exponential recovery**.

See [docs/fusion_algorithm.md](docs/fusion_algorithm.md) for complete mathematical formulation.

---

## Hardware

### Sensor Glove
- **MCU**: STM32H523 (I2C/Serial variant) or STM32H70B (WiFi variant)
- **IMU**: 6× MPU6050 (one per finger + palm), quaternion output at 100Hz
- **Tactile**: Force/tactile sensors per finger, transmitted alongside IMU data
- **Communication**: Serial (COM port, 460800 baud) or WiFi (H70B variant)
- **Firmware source**: See [firmware/](firmware/) directory

### Vision Pipeline
- **Camera**: Intel RealSense depth camera (RGB 640×480 @15 FPS)
- **Detection**: ViTPose+ Huge (whole-body, runs every 6 frames for efficiency)
- **Mesh recovery**: HaMeR (MANO parametric hand model, per-frame inference)
- **Latency**: ViTPose ~112ms (every 6 frames) + HaMeR ~50ms = ~135ms total

---

## Latency Analysis

| Stage | Time | Notes |
|---|---|---|
| Camera capture | ~66ms | 640×480 @15 FPS |
| ViTPose detection | ~112ms | Every 6 frames (effective ~19ms/frame) |
| HaMeR inference | ~50ms | Per hand, every detected frame |
| IMU processing | <1ms | 100Hz stream, lightweight parsing |
| Fusion computation | <1ms | Slerp is O(1) per quaternion |
| UDP transmission | <1ms | Local network |
| **Full pipeline** | **~135ms** | **Effective 7.4 FPS** |

### Optimizations Applied
- **Frame skipping**: ViTPose runs only every 6th frame; intermediate frames reuse last known hand positions
- **Wrist disambiguation**: When ViTPose assigns both left/right to the same hand, wrist keypoint confidence determines the correct label
- **No OpenGL rendering**: 2D skeleton projection via affine transform avoids pyrender/OpenGL issues on Windows
- **CPU thread pool**: OMP/MKL thread count hard-limited to prevent 1790% CPU usage

---

## Roadmap

| Priority | Item | Status |
|---|---|---|
| 🔴 Critical | Accuracy evaluation (MPJPE, PA-MPJPE) | **Pending** — needs 50+ annotated frames |
| 🔴 Critical | Data samples release (5–10 sequences) | **Pending** — needs hardware collection |
| 🟠 Important | Unity project merge into this repo | **Pending** — needs migration from separate repos |
| 🟠 Important | Firmware merge into this repo | **Pending** — needs migration from Glove_sensor_system |
| 🟠 Important | Project reproduction doc merge | **Pending** — needs migration from vitpose-imu-glove-unity |
| 🟡 Nice-to-have | Demo video (30s) | **Pending** |
| 🟡 Nice-to-have | Project page on GitHub Pages | **Pending** |

---

## Citation

If you use this system in your research, please cite:

```bibtex
@software{hand_tracking_fusion_2026,
  author = {Tianlin, Alexander},
  title = {Hand Tracking Fusion System: Multi-Modal IMU × Vision × Tactile Glove for 3D Hand Pose Estimation},
  year = {2026},
  url = {https://github.com/alexandertianlin/hand-tracking-fusion-system}
}
```

---

## License

MIT License — see [LICENSE](LICENSE) file.

---

## Acknowledgments

- [HaMeR](https://github.com/geopavlakos/hamer) — Hand Mesh Recovery (G. Pavlakos et al., CVPR 2024)
- [ViTPose](https://github.com/ViTAE-Transformer/ViTPose) — Vision Transformer Pose Estimation
- [MediaPipe](https://mediapipe.dev) — Hand Landmark Detection (Google)
- [MANO](https://mano.is.tue.mpg.com/) — Hand Model (MPI)

---

*This repo consolidates 6 previously separate repos: hamer, vitpose-imu-glove-unity, vision-tactile-fusion, vision-aided-imu-gesture-glove, vision-imu-gesture-glove, agiletact, Glove_sensor_system.*
