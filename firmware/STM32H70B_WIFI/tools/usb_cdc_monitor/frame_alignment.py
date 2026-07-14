from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path


Quat = tuple[float, float, float, float]

DEFAULT_TARGETS: tuple[tuple[int, int, str], ...] = (
    (20, 2, "thumb"),
    (30, 3, "index"),
    (40, 4, "middle"),
    (50, 5, "ring"),
    (60, 6, "pinky"),
)

PALM_NODE_ID = 0
RESIDUAL_GOOD_DEG = 2.0
RESIDUAL_WARN_DEG = 5.0


@dataclass(frozen=True)
class AlignmentResult:
    node_id: int
    uart_index: int
    finger_label: str
    sample_count: int
    palm_spread_deg: float
    tip_spread_deg: float
    mean_palm: Quat
    mean_tip: Quat
    mount_quat: Quat
    raw_residual_deg: float
    post_mount_residual_deg: float


def _normalize(q: Quat) -> Quat:
    w, x, y, z = q
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 0.0:
        return (1.0, 0.0, 0.0, 0.0)
    return (w / norm, x / norm, y / norm, z / norm)


def _conjugate(q: Quat) -> Quat:
    w, x, y, z = q
    return (w, -x, -y, -z)


def _multiply(a: Quat, b: Quat) -> Quat:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _rotation_angle_deg(q: Quat) -> float:
    w = max(-1.0, min(1.0, abs(_normalize(q)[0])))
    return math.degrees(2.0 * math.acos(w))


def _distance_deg(a: Quat, b: Quat) -> float:
    return _rotation_angle_deg(_multiply(_conjugate(a), b))


def _mean_quaternion(samples: list[Quat]) -> tuple[Quat, float]:
    """Return hemispherically aligned normalized mean and max-from-mean spread in deg."""
    if not samples:
        return ((1.0, 0.0, 0.0, 0.0), 0.0)

    reference = _normalize(samples[0])
    acc = [0.0, 0.0, 0.0, 0.0]
    aligned: list[Quat] = []
    for sample in samples:
        q = _normalize(sample)
        dot = q[0] * reference[0] + q[1] * reference[1] + q[2] * reference[2] + q[3] * reference[3]
        if dot < 0.0:
            q = (-q[0], -q[1], -q[2], -q[3])
        aligned.append(q)
        acc[0] += q[0]
        acc[1] += q[1]
        acc[2] += q[2]
        acc[3] += q[3]
    mean = _normalize((acc[0], acc[1], acc[2], acc[3]))
    spread_deg = max((_distance_deg(mean, q) for q in aligned), default=0.0)
    return mean, spread_deg


def _read_float(row: dict[str, str], key: str) -> float | None:
    value = row.get(key, "")
    if not value:
        return None
    try:
        return float(value)
    except ValueError:
        return None


def _read_quat(row: dict[str, str], prefix: str) -> Quat | None:
    values = [_read_float(row, f"{prefix}_{axis}") for axis in "wxyz"]
    if any(value is None for value in values):
        return None
    return (float(values[0]), float(values[1]), float(values[2]), float(values[3]))  # type: ignore[arg-type]


def load_samples(csv_path: Path, node_id: int) -> tuple[list[Quat], list[Quat]]:
    palm_samples: list[Quat] = []
    tip_samples: list[Quat] = []

    with csv_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            row_node_text = row.get("node_id", "")
            if not row_node_text:
                continue
            try:
                row_node_id = int(row_node_text)
            except ValueError:
                continue
            if row_node_id != node_id:
                continue

            palm_quat = _read_quat(row, "palm_live_quat")
            tip_quat = _read_quat(row, "quat")
            if palm_quat is None or tip_quat is None:
                continue

            palm_samples.append(palm_quat)
            tip_samples.append(tip_quat)

    return palm_samples, tip_samples


def compute_mount_quaternion(
    palm_samples: list[Quat],
    tip_samples: list[Quat],
) -> tuple[Quat, Quat, Quat, float, float, float, float]:
    mean_palm, palm_spread = _mean_quaternion(palm_samples)
    mean_tip, tip_spread = _mean_quaternion(tip_samples)

    raw_residual_deg = _distance_deg(mean_palm, mean_tip)

    # q_out = q_mount * q_tip  (LEFT_MULTIPLY), we want q_out == q_palm when same-pose
    mount = _multiply(mean_palm, _conjugate(mean_tip))
    mount = _normalize(mount)
    if mount[0] < 0.0:
        mount = (-mount[0], -mount[1], -mount[2], -mount[3])

    simulated_tip_out = _multiply(mount, mean_tip)
    post_residual_deg = _distance_deg(mean_palm, simulated_tip_out)

    return (
        mean_palm,
        mean_tip,
        mount,
        palm_spread,
        tip_spread,
        raw_residual_deg,
        post_residual_deg,
    )


def analyze(
    csv_path: Path,
    node_id: int,
    uart_index: int,
    finger_label: str,
) -> AlignmentResult | None:
    palm_samples, tip_samples = load_samples(csv_path, node_id)
    if not palm_samples or not tip_samples:
        return None

    (
        mean_palm,
        mean_tip,
        mount,
        palm_spread,
        tip_spread,
        raw_residual,
        post_residual,
    ) = compute_mount_quaternion(palm_samples, tip_samples)

    return AlignmentResult(
        node_id=node_id,
        uart_index=uart_index,
        finger_label=finger_label,
        sample_count=len(tip_samples),
        palm_spread_deg=palm_spread,
        tip_spread_deg=tip_spread,
        mean_palm=mean_palm,
        mean_tip=mean_tip,
        mount_quat=mount,
        raw_residual_deg=raw_residual,
        post_mount_residual_deg=post_residual,
    )


def format_imu_config_snippet(result: AlignmentResult) -> str:
    w, x, y, z = result.mount_quat
    lines = [
        f"/* {result.finger_label} (node {result.node_id}) - residual after remap: {result.post_mount_residual_deg:.2f} deg */",
        f"#define PALM_EXTERNAL_NODE_UART{result.uart_index}_REMAP_W {w:+.6f}f",
        f"#define PALM_EXTERNAL_NODE_UART{result.uart_index}_REMAP_X {x:+.6f}f",
        f"#define PALM_EXTERNAL_NODE_UART{result.uart_index}_REMAP_Y {y:+.6f}f",
        f"#define PALM_EXTERNAL_NODE_UART{result.uart_index}_REMAP_Z {z:+.6f}f",
        f"#define PALM_EXTERNAL_NODE_UART{result.uart_index}_REMAP_ORDER PALM_EXTERNAL_NODE_REMAP_LEFT_MULTIPLY",
    ]
    return "\n".join(lines)


def _verdict_for(residual_deg: float) -> str:
    if residual_deg <= RESIDUAL_GOOD_DEG:
        return "GOOD - safe to paste"
    if residual_deg <= RESIDUAL_WARN_DEG:
        return "OK - acceptable, re-capture for a tighter value if possible"
    return "POOR - recapture the flat pose, hand was moving or boards were not co-aligned"


def format_result_text(result: AlignmentResult) -> str:
    lines = [
        f"{result.finger_label} (node {result.node_id}, UART{result.uart_index}):",
        f"  samples: {result.sample_count}",
        f"  palm spread: {result.palm_spread_deg:.2f} deg",
        f"  tip spread:  {result.tip_spread_deg:.2f} deg",
        f"  raw tip-vs-palm residual (before remap): {result.raw_residual_deg:.2f} deg",
        f"  simulated residual after remap:           {result.post_mount_residual_deg:.2f} deg",
        f"  verdict: {_verdict_for(result.post_mount_residual_deg)}",
        "",
        format_imu_config_snippet(result),
    ]
    return "\n".join(lines)


def _resolve_targets(
    node_filter: list[int] | None,
) -> tuple[tuple[int, int, str], ...]:
    if not node_filter:
        return DEFAULT_TARGETS
    lookup = {node_id: (uart_index, label) for node_id, uart_index, label in DEFAULT_TARGETS}
    resolved: list[tuple[int, int, str]] = []
    for node_id in node_filter:
        if node_id not in lookup:
            raise SystemExit(
                f"node id {node_id} is not a known fingertip node; "
                f"known nodes: {sorted(lookup)}"
            )
        uart_index, label = lookup[node_id]
        resolved.append((node_id, uart_index, label))
    return tuple(resolved)


def _build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Compute palm-side mounting quaternions for fingertip boards from a flat-hand "
            "capture CSV produced by monitor.py. Capture ~3 seconds with the palm board and "
            "each fingertip board laid flat in the same physical orientation."
        )
    )
    parser.add_argument("csv_path", type=Path, help="Capture CSV from monitor.py.")
    parser.add_argument(
        "--node",
        type=int,
        action="append",
        dest="nodes",
        help="Restrict to one fingertip node id (can be given multiple times). Default: all 20/30/40/50/60.",
    )
    return parser


def main() -> None:
    parser = _build_argument_parser()
    args = parser.parse_args()
    csv_path: Path = args.csv_path
    if not csv_path.is_file():
        raise SystemExit(f"CSV file not found: {csv_path}")

    targets = _resolve_targets(args.nodes)
    any_result = False
    snippets: list[str] = []

    for node_id, uart_index, label in targets:
        result = analyze(csv_path, node_id, uart_index, label)
        if result is None:
            print(f"{label} (node {node_id}, UART{uart_index}): no samples in CSV")
            print()
            continue
        any_result = True
        print(format_result_text(result))
        print()
        snippets.append(format_imu_config_snippet(result))

    if any_result:
        print("--- Paste into Core/Inc/imu/imu_config.h ---")
        print("\n\n".join(snippets))
        print("--- end ---")
    else:
        raise SystemExit(
            "No fingertip samples found in the CSV. Check that the capture ran while the "
            "palm board and at least one fingertip board were streaming fused frames."
        )


if __name__ == "__main__":
    main()
