"""Phase 0 auto-detect: classify each fingertip board's mounting orientation.

From a single flat-pose capture CSV (palm + fingertip boards co-aligned on a
flat surface), this tool determines, for every fingertip node found in the CSV,
which of the 24 right-handed axis permutations best maps the tip's published
quaternion onto the palm's. The result is a JSON report intended to be handed
to a human or AI agent for review before landing the fix in firmware.

The complementary tool ``frame_alignment.py`` produces a single arbitrary
mounting quaternion per finger (mathematically complete but unreadable). This
tool produces a clean axis-permutation answer (e.g. ``X <- +Yraw,
Y <- -Xraw, Z <- +Zraw``), which is the form a fingertip firmware repo would
land as its raw-axis remap.

CSV format consumed: the ``capture`` CSV produced by ``monitor.py``. Columns
used: ``time_s``, ``node_id``, ``palm_live_quat_{w,x,y,z}``, ``quat_{w,x,y,z}``.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from itertools import permutations, product
from pathlib import Path

from frame_alignment import (
    DEFAULT_TARGETS,
    Quat,
    _conjugate,
    _distance_deg,
    _mean_quaternion,
    _multiply,
    _normalize,
    compute_mount_quaternion,
    load_samples,
)


SCHEMA_VERSION = "phase0-autodetect-1.0"

RESIDUAL_GOOD_DEG = 2.0
RESIDUAL_OK_DEG = 5.0
SPREAD_WARN_DEG = 0.5
CONFIDENCE_MARGIN_DEG = 5.0


Matrix3 = tuple[tuple[int, int, int], tuple[int, int, int], tuple[int, int, int]]


@dataclass(frozen=True)
class Permutation:
    """One of the 24 right-handed axis permutations.

    ``matrix`` maps a vector in the raw sensor frame to a vector in the
    canonical glove frame: ``v_canonical = matrix @ v_raw``. ``quaternion``
    is the equivalent rotation in (w, x, y, z) form. ``name`` is a short
    label like ``identity`` or ``yaw90``.
    """

    matrix: Matrix3
    quaternion: Quat
    name: str

    def axes_mapping(self) -> dict[str, str]:
        """Return ``{'X': '+Yraw', 'Y': '-Xraw', 'Z': '+Zraw'}`` style dict."""
        axis_letters = ("X", "Y", "Z")
        result: dict[str, str] = {}
        for canonical_idx, row in enumerate(self.matrix):
            for raw_idx, value in enumerate(row):
                if value != 0:
                    sign = "+" if value > 0 else "-"
                    result[axis_letters[canonical_idx]] = f"{sign}{axis_letters[raw_idx]}raw"
                    break
        return result

    def axes_text(self) -> str:
        mapping = self.axes_mapping()
        return ", ".join(f"{canonical} <- {raw}" for canonical, raw in mapping.items())


# ---------------------------------------------------------------------------
# 24 right-handed permutations
# ---------------------------------------------------------------------------

def _matrix_determinant(matrix: Matrix3) -> int:
    a = matrix
    return (
        a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])
    )


def _matrix_to_quaternion(matrix: Matrix3) -> Quat:
    m = [[float(matrix[i][j]) for j in range(3)] for i in range(3)]
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif (m[0][0] >= m[1][1]) and (m[0][0] >= m[2][2]):
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] >= m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s
    q = _normalize((w, x, y, z))
    if q[0] < 0.0:
        q = (-q[0], -q[1], -q[2], -q[3])
    return q


def _name_for_permutation(matrix: Matrix3) -> str:
    axis_letters = ("X", "Y", "Z")
    parts: list[str] = []
    for row in matrix:
        for raw_idx, value in enumerate(row):
            if value != 0:
                sign = "+" if value > 0 else "-"
                parts.append(f"{sign}{axis_letters[raw_idx]}")
                break
    permuted = "".join(part[1] for part in parts)
    signs = "".join(part[0] for part in parts)
    if permuted == "XYZ" and signs == "+++":
        return "identity"
    return f"perm[{','.join(parts)}]"


def enumerate_right_handed_permutations() -> tuple[Permutation, ...]:
    """Return all 24 right-handed axis permutations of the cube."""
    results: list[Permutation] = []
    seen: set[Matrix3] = set()
    for perm in permutations((0, 1, 2)):
        for signs in product((1, -1), repeat=3):
            row0 = [0, 0, 0]
            row1 = [0, 0, 0]
            row2 = [0, 0, 0]
            row0[perm[0]] = signs[0]
            row1[perm[1]] = signs[1]
            row2[perm[2]] = signs[2]
            matrix: Matrix3 = (
                (row0[0], row0[1], row0[2]),
                (row1[0], row1[1], row1[2]),
                (row2[0], row2[1], row2[2]),
            )
            if matrix in seen:
                continue
            if _matrix_determinant(matrix) != 1:
                continue
            seen.add(matrix)
            quaternion = _matrix_to_quaternion(matrix)
            results.append(
                Permutation(
                    matrix=matrix,
                    quaternion=quaternion,
                    name=_name_for_permutation(matrix),
                )
            )
    if len(results) != 24:  # pragma: no cover - sanity
        raise RuntimeError(
            f"Expected 24 right-handed permutations, generated {len(results)}"
        )
    return tuple(results)


PERMUTATIONS: tuple[Permutation, ...] = enumerate_right_handed_permutations()


# ---------------------------------------------------------------------------
# Per-finger analysis
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class PermutationCandidate:
    permutation: Permutation
    residual_deg: float


def _rank_permutations(
    mean_palm: Quat,
    mean_tip: Quat,
) -> list[PermutationCandidate]:
    candidates: list[PermutationCandidate] = []
    for perm in PERMUTATIONS:
        simulated = _multiply(perm.quaternion, mean_tip)
        residual = _distance_deg(mean_palm, simulated)
        candidates.append(PermutationCandidate(permutation=perm, residual_deg=residual))
    candidates.sort(key=lambda c: c.residual_deg)
    return candidates


def _format_palm_fallback_block(
    uart_index: int,
    finger_label: str,
    node_id: int,
    quaternion: Quat,
    residual_deg: float,
) -> str:
    w, x, y, z = quaternion
    lines = [
        f"/* {finger_label} (node {node_id}) - permutation residual: {residual_deg:.2f} deg */",
        f"#define PALM_EXTERNAL_NODE_UART{uart_index}_REMAP_W {w:+.6f}f",
        f"#define PALM_EXTERNAL_NODE_UART{uart_index}_REMAP_X {x:+.6f}f",
        f"#define PALM_EXTERNAL_NODE_UART{uart_index}_REMAP_Y {y:+.6f}f",
        f"#define PALM_EXTERNAL_NODE_UART{uart_index}_REMAP_Z {z:+.6f}f",
        f"#define PALM_EXTERNAL_NODE_UART{uart_index}_REMAP_ORDER PALM_EXTERNAL_NODE_REMAP_LEFT_MULTIPLY",
    ]
    return "\n".join(lines)


def _verdict(
    best_residual_deg: float,
    confidence_margin_deg: float,
    palm_spread_deg: float,
    tip_spread_deg: float,
) -> tuple[str, str]:
    """Return (verdict, reason)."""
    if palm_spread_deg > SPREAD_WARN_DEG or tip_spread_deg > SPREAD_WARN_DEG:
        return (
            "POOR",
            (
                f"capture was not still (palm spread {palm_spread_deg:.2f} deg, "
                f"tip spread {tip_spread_deg:.2f} deg); recapture with nothing moving"
            ),
        )
    if best_residual_deg <= RESIDUAL_GOOD_DEG and confidence_margin_deg >= CONFIDENCE_MARGIN_DEG:
        return ("GOOD", "best permutation fits within tolerance with high confidence")
    if best_residual_deg <= RESIDUAL_OK_DEG:
        return (
            "OK",
            (
                "best permutation fits acceptably but consider recapturing "
                "for a tighter result"
            ),
        )
    return (
        "POOR",
        (
            f"no permutation fits within {RESIDUAL_OK_DEG:.0f} deg; the boards "
            "were probably not co-aligned during capture"
        ),
    )


def analyze_finger(
    node_id: int,
    uart_index: int,
    finger_label: str,
    palm_samples: list[Quat],
    tip_samples: list[Quat],
) -> dict | None:
    if not palm_samples or not tip_samples:
        return None

    (
        mean_palm,
        mean_tip,
        free_form_mount,
        palm_spread_deg,
        tip_spread_deg,
        raw_residual_deg,
        free_form_residual_deg,
    ) = compute_mount_quaternion(palm_samples, tip_samples)

    ranked = _rank_permutations(mean_palm, mean_tip)
    best = ranked[0]
    runner_up = ranked[1] if len(ranked) > 1 else None

    runner_up_residual_deg = runner_up.residual_deg if runner_up is not None else float("inf")
    confidence_margin_deg = runner_up_residual_deg - best.residual_deg

    verdict, verdict_reason = _verdict(
        best_residual_deg=best.residual_deg,
        confidence_margin_deg=confidence_margin_deg,
        palm_spread_deg=palm_spread_deg,
        tip_spread_deg=tip_spread_deg,
    )

    palm_fallback_block = _format_palm_fallback_block(
        uart_index=uart_index,
        finger_label=finger_label,
        node_id=node_id,
        quaternion=best.permutation.quaternion,
        residual_deg=best.residual_deg,
    )

    return {
        "node": node_id,
        "uart": uart_index,
        "label": finger_label,
        "samples": len(tip_samples),
        "palm_spread_deg": round(palm_spread_deg, 4),
        "tip_spread_deg": round(tip_spread_deg, 4),
        "raw_residual_deg": round(raw_residual_deg, 4),
        "mean_palm_quat": [round(c, 6) for c in mean_palm],
        "mean_tip_quat": [round(c, 6) for c in mean_tip],
        "best_permutation": {
            "name": best.permutation.name,
            "axes": best.permutation.axes_mapping(),
            "axes_text": best.permutation.axes_text(),
            "quaternion": [round(c, 6) for c in best.permutation.quaternion],
            "residual_deg": round(best.residual_deg, 4),
            "runner_up_name": runner_up.permutation.name if runner_up else None,
            "runner_up_residual_deg": (
                round(runner_up.residual_deg, 4) if runner_up is not None else None
            ),
            "confidence_margin_deg": round(confidence_margin_deg, 4),
        },
        "free_form_mount": {
            "quaternion": [round(c, 6) for c in free_form_mount],
            "residual_deg": round(free_form_residual_deg, 4),
        },
        "verdict": verdict,
        "verdict_reason": verdict_reason,
        "fix": {
            "fingertip_firmware_raw_axis_remap": best.permutation.axes_text(),
            "palm_fallback_imu_config_block": palm_fallback_block,
        },
    }


# ---------------------------------------------------------------------------
# Whole-CSV pass + report assembly
# ---------------------------------------------------------------------------

def _palm_summary(csv_path: Path) -> tuple[int, float, float, float]:
    """Return (palm_sample_count, palm_spread_deg, t_min, t_max).

    Reads palm rows (node_id == 0) once to estimate capture duration and palm
    stillness independently of the per-finger loop.
    """
    palm_samples: list[Quat] = []
    t_min: float | None = None
    t_max: float | None = None
    with csv_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            ts = row.get("time_s")
            if ts:
                try:
                    ts_value = float(ts)
                except ValueError:
                    ts_value = None
                if ts_value is not None:
                    t_min = ts_value if t_min is None else min(t_min, ts_value)
                    t_max = ts_value if t_max is None else max(t_max, ts_value)
            node_text = row.get("node_id", "")
            if node_text != "0":
                continue
            try:
                quat = (
                    float(row["palm_live_quat_w"]),
                    float(row["palm_live_quat_x"]),
                    float(row["palm_live_quat_y"]),
                    float(row["palm_live_quat_z"]),
                )
            except (KeyError, ValueError, TypeError):
                continue
            palm_samples.append(quat)

    _, spread_deg = _mean_quaternion(palm_samples)
    duration = (t_max - t_min) if (t_min is not None and t_max is not None) else 0.0
    return len(palm_samples), spread_deg, t_min or 0.0, t_max or 0.0


def run_autodetect(csv_path: Path) -> dict:
    """Run the full auto-detect pass over a capture CSV."""
    if not csv_path.is_file():
        raise FileNotFoundError(f"CSV file not found: {csv_path}")

    palm_samples_count, palm_spread_deg, t_min, t_max = _palm_summary(csv_path)
    duration_s = max(0.0, t_max - t_min)

    finger_reports: list[dict] = []
    for node_id, uart_index, label in DEFAULT_TARGETS:
        palm_samples, tip_samples = load_samples(csv_path, node_id)
        report = analyze_finger(
            node_id=node_id,
            uart_index=uart_index,
            finger_label=label,
            palm_samples=palm_samples,
            tip_samples=tip_samples,
        )
        if report is not None:
            finger_reports.append(report)

    summary_parts = [
        f"{f['label']} {f['verdict']} ({f['best_permutation']['name']}, "
        f"residual {f['best_permutation']['residual_deg']:.2f} deg)"
        for f in finger_reports
    ]
    summary_text = ", ".join(summary_parts) if summary_parts else "no fingertip data found"

    return {
        "schema_version": SCHEMA_VERSION,
        "csv_path": str(csv_path),
        "captured_seconds": round(duration_s, 3),
        "palm": {
            "samples": palm_samples_count,
            "spread_deg": round(palm_spread_deg, 4),
        },
        "fingers": finger_reports,
        "summary_text": summary_text,
    }


def write_report_json(report: dict, json_path: Path) -> None:
    json_path.parent.mkdir(parents=True, exist_ok=True)
    with json_path.open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, ensure_ascii=False)
        handle.write("\n")


def default_report_path(csv_path: Path) -> Path:
    return csv_path.with_suffix(csv_path.suffix + ".phase0.json")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Classify each fingertip board's mounting orientation against the "
            "palm board from a single flat-pose capture CSV (produced by "
            "monitor.py). Emits a JSON report listing the best-matching "
            "axis permutation per finger."
        )
    )
    parser.add_argument("csv_path", type=Path, help="Capture CSV from monitor.py.")
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help=(
            "Where to write the JSON report. Defaults to "
            "<csv_path>.phase0.json next to the input CSV."
        ),
    )
    return parser


def main() -> None:
    parser = _build_argument_parser()
    args = parser.parse_args()
    csv_path: Path = args.csv_path
    if not csv_path.is_file():
        raise SystemExit(f"CSV file not found: {csv_path}")

    out_path: Path = args.out if args.out is not None else default_report_path(csv_path)

    report = run_autodetect(csv_path)
    write_report_json(report, out_path)

    print(f"Wrote {out_path}")
    print(report["summary_text"])
    if not report["fingers"]:
        raise SystemExit(
            "No fingertip samples found in the CSV. Check that the capture ran "
            "while the palm board and at least one fingertip board were "
            "streaming fused frames."
        )


if __name__ == "__main__":
    main()
