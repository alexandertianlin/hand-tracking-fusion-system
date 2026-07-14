# Evaluation Plan

> This directory will contain quantitative evaluation results once testing is complete.
> All metrics listed below are **pending** — data collection and evaluation scripts need to be run.

---

## Planned Evaluations

### 1. Accuracy Evaluation (`accuracy_evaluation.md`)

| Metric | Definition | Target | Method |
|---|---|---|---|
| **MPJPE** | Mean Per-Joint Position Error (mm) | < 10mm | Capture 50+ annotated frames, compute error vs ground truth |
| **PA-MPJPE** | Procrustes-Aligned MPJPE (mm) | < 8mm | Same, after rigid alignment removes global pose |
| **Per-finger F1** | Open/fist classification accuracy | > 0.9 per finger | Collect per-finger state annotations, compute F1 |

**Script**: `python/test_accuracy.py` (to be enhanced with MPJPE computation)

**Ground truth**: MANO mesh vertices from HaMeR (self-consistency check) + manual 3D annotations

### 2. Fusion Comparison (`fusion_comparison.md`)

| Comparison | Metric | Method |
|---|---|---|
| Vision-only vs IMU-only vs Fused | Drift over 60s (degrees) | Static hold test: hand stays still for 60s, measure orientation deviation |
| IMU-only drift rate | Degrees/minute | 60s hold, compare start vs end orientation |
| Fused drift improvement | % reduction vs IMU-only | Same test with fusion enabled |
| Occlusion recovery time | ms | Simulate occlusion (cover camera), measure time to stable pose after uncovering |

### 3. Latency Analysis (`latency_analysis.md`)

Already partially available from `hamer` repo:
- Full pipeline: ~135ms → 7.4 FPS
- ViTPose: ~112ms (every 6 frames)
- HaMeR: ~50ms per hand

**To add**: Fusion computation overhead, IMU parsing overhead, UDP transmission overhead

---

## Data Collection Plan

1. **Static hold sequences** (5 sequences × 60s each): hand stationary, measure drift
2. **Dynamic gesture sequences** (5 sequences × 30s each): open/close/rotate, measure tracking accuracy
3. **Occlusion sequences** (5 sequences): camera covered for 5s, measure recovery
4. **Calibration validation** (3 sequences): before/after calibration, measure offset stability

All sequences will include synchronized IMU + vision + timestamp data.

---

*This plan will be executed once the consolidated repo is pushed to GitHub and hardware is available for data collection.*
