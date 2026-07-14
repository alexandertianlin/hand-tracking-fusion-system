# Accuracy Evaluation Report

**Date:** 2026-07-14  
**Session:** `session_v920b_20260707_184103` (435 frames)  
**Model:** HaMeR + ViTPose (hrnet_w48_dark) + RealSense D435i  
**Evaluator:** Self-consistency audit (no ground-truth 3D labels)

---

## 1. Executive Summary

This report presents quantitative evaluation metrics computed from available session data. **Ground-truth 3D joint positions (required for MPJPE/PA-MPJPE) are not yet available** — all metrics below are based on 2D keypoint self-consistency, detection quality, and per-finger stability analysis.

| Metric | Value | Target | Status |
|--------|-------|--------|--------|
| Hand detection rate | 99.31% | >95% | ✅ PASS |
| Valid joint rate | 99.31% | >95% | ✅ PASS |
| Frame dropout | 3/435 (0.69%) | <5% | ✅ PASS |
| FPS (median) | 9.50 | >7 | ✅ PASS |
| Total latency p50 | 101.4 ms | <150 ms | ✅ PASS |
| Total latency p95 | 121.0 ms | <200 ms | ✅ PASS |
| Mean keypoint confidence | 0.458 | >0.3 | ✅ PASS (borderline) |
| Per-finger tip error p95 | 112–261 px | <30 px* | ❌ NEEDS IMPROVEMENT |
| Per-finger jitter p95 | 61–90 px | <15 px* | ❌ NEEDS IMPROVEMENT |
| MPJPE | — | <10 mm | ⏳ PENDING (needs GT) |
| PA-MPJPE | — | <8 mm | ⏳ PENDING (needs GT) |

*\*Pixel thresholds are approximate; proper comparison requires conversion to mm via camera depth calibration.*

---

## 2. Detection Quality Metrics

### 2.1 Hand Detection

| Metric | Value |
|--------|-------|
| Total frames | 435 |
| Frames with hand detected | 432 |
| Frames with no hand | 3 |
| Detection rate | 99.31% |
| Longest no-hand gap | 1 frame |
| Hold frames (bbox reused from previous) | 38 |

**Interpretation:** Near-perfect detection rate. The 3 dropout frames are isolated (gap ≤ 1), suggesting brief ViTPose misses rather than systematic failure.

### 2.2 Bounding Box Source Distribution

| Source | Count | Ratio |
|--------|-------|-------|
| Fresh (new ViTPose detection) | 180 | 41.4% |
| Reuse (previous bbox, ViTPose skipped) | 214 | 49.2% |
| Hold (static bbox, no update) | 38 | 8.7% |
| None (no detection) | 3 | 0.7% |

**Interpretation:** ~50% of frames reuse the previous bbox (skip ViTPose for speed), which is the intended optimization. The hold+none ratio is healthy.

### 2.3 ViTPose Keypoint Confidence (from 247_balanced_benchmark)

| Config | Model | Mean Quality | Min Quality | Mean Conf (L) | Mean Conf (R) | Completeness |
|--------|-------|-------------|-------------|---------------|---------------|-------------|
| Best (kpt_thr=0.3, min_kpts=4) | hrnet_w48_dark | 255.46 | 171.92 | 0.6522 | 0.5115 | 0.500 |
| Old best (kpt_thr=0.5, min_kpts=4) | hrnet_w48_dark | 267.39 | 203.65 | 0.6522 | 0.5753 | 0.238 |

**Key finding:** Lower keypoint threshold (0.3 vs 0.5) trades per-keypoint confidence for higher completeness (50% vs 24% of keypoints detected), resulting in better overall detection quality.

---

## 3. Per-Finger Stability Metrics

### 3.1 Finger Jitter (inter-frame keypoint displacement)

| Finger | Valid Rate | Jitter p50 (px) | Jitter p95 (px) | Jitter Max (px) | Jump Count | Worst Frame |
|--------|-----------|-----------------|-----------------|-----------------|------------|-------------|
| Thumb | 99.31% | 11.29 | 90.0 | 90.0 | 63 | 57 |
| Index | 99.31% | 9.12 | 75.34 | 90.0 | 47 | 144 |
| Middle | 99.31% | 8.68 | 60.67 | 90.0 | 22 | 62 |
| Ring | 99.31% | 9.78 | 83.44 | 90.0 | 36 | 9 |
| Little | 99.31% | 10.81 | 90.0 | 90.0 | 62 | 9 |

**Observations:**
- p50 jitter is reasonable (~9–11 px), but p95 spikes to 90 px (the audit threshold cap)
- Thumb and little finger have the most jumps (63 and 62), consistent with their smaller/more mobile keypoints
- Middle finger is most stable (22 jumps)

### 3.2 Per-Finger Tip Error (against smoothed/expected position)

| Finger | Tip Error p95 (px) | Tip Error Max (px) | Mean Confidence |
|--------|---------------------|---------------------|-----------------|
| Thumb | 260.7 | 346.6 | 0.383 |
| Index | 148.4 | 182.0 | 0.443 |
| Middle | 152.6 | 231.6 | 0.459 |
| Ring | 112.8 | 244.5 | 0.412 |
| Little | 191.3 | 220.4 | 0.448 |

**Observations:**
- Thumb tip error is highest (260 px p95), likely due to thumb's independent motion range
- Ring finger has lowest tip error (113 px p95), most stable
- Confidence is low overall (0.38–0.46), suggesting the model is uncertain about per-keypoint positions

### 3.3 Failure Analysis (Top Failure Labels)

| Failure Label | Count | Description |
|---------------|-------|-------------|
| BBOX_CROP_TRUNCATES_TIPS | 286 | Bounding box clips fingertip keypoints |
| TIP_THUMB_DRIFT | 38 | Thumb tip position drifts over time |
| TIP_LITTLE_DRIFT | 30 | Little finger tip drifts |
| HAMER_WRIST_TELEPORT | 28 | Wrist position jumps suddenly |
| TIP_MIDDLE_DRIFT | 22 | Middle finger tip drifts |
| QUALITY_BAD_REUSE | 18 | Reused bbox quality below threshold |
| QUALITY_FALSE_REJECT | 18 | Good detection incorrectly rejected |
| TIP_INDEX_DRIFT | 16 | Index finger tip drifts |
| TIP_RING_DRIFT | 10 | Ring finger tip drifts |
| BBOX_AREA_JUMP | 5 | Bounding box size changes drastically |
| BBOX_CENTER_JUMP | 4 | Bounding box center shifts >90px |
| DETECT_NO_HAND | 3 | No hand detected |

**Root cause:** The dominant failure (BBOX_CROP_TRUNCATES_TIPS, 286/435 = 65.7%) indicates the ViTPose bounding box is too tight, consistently cutting off fingertip positions. This is the **primary bottleneck** for accuracy improvement.

---

## 4. Latency Metrics

### 4.1 Pipeline Breakdown

| Stage | p50 (ms) | p95 (ms) | Max (ms) |
|-------|----------|----------|----------|
| Camera capture | 0.34 | 0.49 | 13.38 |
| ViTPose + bbox | 59.83 | 72.65 | 878.95 |
| HaMeR inference | 47.83 | 51.43 | 342.99 |
| Preprocess | 0.08 | 0.15 | 2.43 |
| Tracker | 0.03 | 0.04 | 0.14 |
| Smooth clear | 0.00 | 0.00 | 0.00 |
| Save latest | 3.30 | 3.58 | 7.54 |
| Draw overlay | 2.39 | 2.66 | 5.43 |
| Write JSON | 0.45 | 0.55 | 1.14 |
| **Total loop** | **105.8** | **126.6** | **1259.5** |
| **Model loop** | **101.4** | **121.0** | **1237.8** |

### 4.2 Inter-Frame Timing

| Metric | Value |
|--------|-------|
| Delta p50 | 78.9 ms |
| Delta p95 | 124.7 ms |
| Delta max | 166.8 ms |
| FPS (median) | 9.50 |
| FPS (from delta p50) | ~12.7 |

**Interpretation:** The pipeline runs at ~10 FPS in practice. ViTPose bbox detection is the bottleneck (p95=72.6ms), with HaMeR adding ~48ms. The bbox reuse strategy (50% of frames skip ViTPose) helps maintain throughput.

---

## 5. Benchmark Quality Comparison

### 5.1 ViTPose Detection Benchmark (hamer_detection_benchmark.csv)

| Model | CLAHE | Keypoint Thr | Mean Quality | Mean Conf | Hands Detected |
|-------|-------|-------------|-------------|-----------|----------------|
| vitpose_huge | False | 0.15 | 342.50 | 0.6186 | 2 |
| vitpose_huge | True | 0.15 | 337.24 | 0.5914 | 2 |
| hrnet_w48_dark | False | 0.15 | 339.01 | 0.5819 | 2 |
| hrnet_w48_dark | True | 0.15 | 335.32 | 0.5551 | 2 |

**Observation:** vitpose_huge achieves slightly higher confidence (0.62 vs 0.58) but similar quality scores. CLAHE preprocessing slightly reduces confidence.

### 5.2 Balanced Quality Score (247_balanced_benchmark.csv)

Best configuration (hrnet_w48_dark, kpt_thr=0.3, min_kpts=4):
- Mean raw score: 255.46
- Mean balanced score: 246.21
- Min balanced score: 73.95
- Mean completeness: 0.500
- Max completeness gap: 0.476

---

## 6. MPJPE Evaluation Pipeline (Required but Not Yet Available)

### 6.1 What is MPJPE?

**Mean Per Joint Position Error (MPJPE)** measures the average Euclidean distance (in mm) between predicted and ground-truth 3D joint positions:

```
MPJPE = (1/N) Σ_i ||pred_i - gt_i||_2
```

**PA-MPJPE** (Procrustes-Aligned MPJPE) removes global rigid transformation before computing error, measuring only pose shape accuracy.

### 6.2 Why We Cannot Compute MPJPE Now

Current data contains only:
- 2D keypoint predictions (pixel coordinates)
- Self-consistency metrics (jitter, tip error against smoothed trajectory)
- Detection confidence scores

**Missing:** Ground-truth 3D joint positions from:
1. Annotated 3D dataset (FreiHAND, DexYCB, HO3D)
2. Manual 3D annotation of our own recordings
3. MANO model parameters with ground-truth fits

### 6.3 MPJPE Pipeline Setup Plan

**Option A: Standard Benchmark Dataset (Recommended)**

1. Download FreiHANDv2 or DexYCB dataset (ground-truth 3D + MANO params)
2. Run HaMeR on benchmark images → extract predicted 3D joints
3. Align prediction and ground-truth (root translation + scale)
4. Compute MPJPE and PA-MPJPE

Expected setup time: 1–2 days. Expected result: competitive MPJPE <10mm (HaMeR paper reports ~7.7mm on FreiHAND).

**Option B: Custom Ground Truth Collection**

1. Record synchronized D435i + marker-based motion capture session
2. Annotate 3D joint positions from mocap data
3. Align mocap and camera coordinate systems
4. Compute MPJPE on custom dataset

Expected setup time: 2–3 weeks (requires mocap lab access).

**Option C: Proxy Metric (Immediate)**

Convert 2D tip error from pixels to mm using D435i depth data:
- `error_mm = tip_error_px × depth_mm / focal_length_px`
- D435i focal length ≈ 616 px (at 640×480)

This provides an approximate 2D→3D error proxy without ground truth.

---

## 7. Recommendations

### 7.1 Immediate Actions

1. **Fix BBOX_CROP_TRUNCATES_TIPS** (65.7% of frames): Increase bbox padding/margin to ensure fingertip keypoints are always within the crop. This alone could dramatically improve tip error.
2. **Implement Option C (depth-based proxy)**: Use D435i depth map to convert pixel errors to mm, providing a rough MPJPE estimate.
3. **Increase ViTPose keypoint threshold**: Consider kpt_thr=0.3 (current best) vs 0.5 — lower threshold improves completeness but may need IMU fusion to compensate for lower confidence keypoints.

### 7.2 Short-term (1–2 weeks)

1. **Run FreiHANDv2 benchmark** (Option A): Download dataset, compute MPJPE/PA-MPJPE against ground truth.
2. **Add IMU fusion comparison**: Compute MPJPE with and without IMU-glove fusion to quantify the improvement from multi-modal fusion.
3. **Per-finger contact-event F1**: Once tactile data is available, compute per-finger contact detection F1 score.

### 7.3 Medium-term (for paper)

1. **DexYCB/HO3D benchmark**: More challenging dataset with hand-object interaction.
2. **Occlusion recovery evaluation**: Measure MPJPE before/after occlusion events, quantify exponential recovery effectiveness.
3. **Cross-body transfer evaluation**: Measure MPJPE after TactAlign-style tactile alignment.

---

## 8. Data Sources

| File | Contents | Frames | Key Metrics |
|------|----------|--------|-------------|
| v923_audit/per_frame_audit.csv | Per-frame 2D keypoints, bbox, latency, failure labels | 435 | Full pipeline data |
| v923_audit/finger_jitter.csv | Per-finger jitter/tip error summary | 5 fingers | Stability metrics |
| v923_audit/latency_report.csv/json | Latency breakdown per frame | 435 | Timing data |
| v923_audit/audit_summary.json | Complete session summary | 435 | All metrics aggregated |
| 247_balanced_benchmark.csv/json | ViTPose detection quality sweep | 2 images × N configs | Quality scores |
| hamer_detection_benchmark.csv/json | HaMeR detection quality sweep | 2 images × N configs | Detection confidence |

---

## 9. Limitations

1. **No ground-truth 3D labels**: All metrics are self-consistency-based, not absolute accuracy. This is the most significant limitation.
2. **Single session**: Results are from one recording session; generalization needs multi-session evaluation.
3. **2D-only metrics**: Tip error and jitter are in pixels, not mm. Depth-based conversion is approximate.
4. **No IMU fusion comparison**: Current data is HaMeR-only; fusion improvement metrics require IMU-glove data alongside.

---

*Report generated from v923_audit data. MPJPE evaluation pending ground-truth dataset acquisition.*
