from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path
import re


CANDIDATE_NODE_IDS = (30, 40, 50)
FINGER_NAMES = ("index", "middle", "ring")
FILENAME_FINGER_RE = re.compile(r"(index|middle|ring)", re.IGNORECASE)


@dataclass(frozen=True)
class IsolationNodeStats:
    node_id: int
    sample_count: int
    max_relative_rotation_deg: float
    yaw_range_deg: float
    pitch_range_deg: float
    roll_range_deg: float
    score_deg: float


@dataclass(frozen=True)
class IsolationFileResult:
    path: Path
    finger_label: str | None
    palm_motion_deg: float
    dominant_node_id: int | None
    dominant_margin_deg: float
    node_stats: tuple[IsolationNodeStats, ...]


@dataclass(frozen=True)
class IsolationFingerConsensus:
    finger_label: str
    file_count: int
    winning_node_id: int | None
    stable: bool
    details: str


def quaternion_normalize(quaternion: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    w, x, y, z = quaternion
    norm = math.sqrt((w * w) + (x * x) + (y * y) + (z * z))
    if norm <= 0.0:
        return (1.0, 0.0, 0.0, 0.0)
    w /= norm
    x /= norm
    y /= norm
    z /= norm
    if w < 0.0:
        return (-w, -x, -y, -z)
    return (w, x, y, z)


def quaternion_distance_deg(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> float:
    dot = sum(a * b for a, b in zip(quaternion_normalize(left), quaternion_normalize(right)))
    dot = max(-1.0, min(1.0, abs(dot)))
    return math.degrees(2.0 * math.acos(dot))


def detect_finger_label(path: Path) -> str | None:
    match = FILENAME_FINGER_RE.search(path.stem)
    if match is None:
        return None
    return match.group(1).lower()


def _read_float(row: dict[str, str], key: str) -> float | None:
    value = row.get(key, "")
    if not value:
        return None
    try:
        return float(value)
    except ValueError:
        return None


def analyze_isolation_csv(
    csv_path: str | Path,
    candidate_node_ids: tuple[int, ...] = CANDIDATE_NODE_IDS,
) -> IsolationFileResult:
    path = Path(csv_path)
    relative_quats: dict[int, list[tuple[float, float, float, float]]] = {
        node_id: [] for node_id in candidate_node_ids
    }
    axis_values: dict[int, dict[str, list[float]]] = {
        node_id: {"yaw": [], "pitch": [], "roll": []} for node_id in candidate_node_ids
    }
    palm_quats: list[tuple[float, float, float, float]] = []

    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            node_id_text = row.get("node_id", "")
            if node_id_text:
                try:
                    node_id = int(node_id_text)
                except ValueError:
                    node_id = None
            else:
                node_id = None

            palm_quat = tuple(
                _read_float(row, f"palm_live_quat_{axis}") for axis in "wxyz"
            )
            if all(value is not None for value in palm_quat):
                palm_quats.append(quaternion_normalize(palm_quat))  # type: ignore[arg-type]

            if node_id not in candidate_node_ids:
                continue

            relative_quat = tuple(
                _read_float(row, f"relative_to_palm_quat_{axis}") for axis in "wxyz"
            )
            if all(value is not None for value in relative_quat):
                relative_quats[node_id].append(quaternion_normalize(relative_quat))  # type: ignore[arg-type]

            yaw = _read_float(row, "relative_to_palm_yaw_deg")
            pitch = _read_float(row, "relative_to_palm_pitch_deg")
            roll = _read_float(row, "relative_to_palm_roll_deg")
            if yaw is not None:
                axis_values[node_id]["yaw"].append(yaw)
            if pitch is not None:
                axis_values[node_id]["pitch"].append(pitch)
            if roll is not None:
                axis_values[node_id]["roll"].append(roll)

    node_stats: list[IsolationNodeStats] = []
    for node_id in candidate_node_ids:
        quats = relative_quats[node_id]
        baseline = quats[0] if quats else None
        max_relative_rotation_deg = 0.0
        if baseline is not None:
            max_relative_rotation_deg = max(
                quaternion_distance_deg(baseline, quaternion) for quaternion in quats
            )

        yaw_values = axis_values[node_id]["yaw"]
        pitch_values = axis_values[node_id]["pitch"]
        roll_values = axis_values[node_id]["roll"]
        yaw_range_deg = (max(yaw_values) - min(yaw_values)) if yaw_values else 0.0
        pitch_range_deg = (max(pitch_values) - min(pitch_values)) if pitch_values else 0.0
        roll_range_deg = (max(roll_values) - min(roll_values)) if roll_values else 0.0
        score_deg = max(max_relative_rotation_deg, yaw_range_deg, pitch_range_deg, roll_range_deg)
        node_stats.append(
            IsolationNodeStats(
                node_id=node_id,
                sample_count=len(quats),
                max_relative_rotation_deg=max_relative_rotation_deg,
                yaw_range_deg=yaw_range_deg,
                pitch_range_deg=pitch_range_deg,
                roll_range_deg=roll_range_deg,
                score_deg=score_deg,
            )
        )

    node_stats.sort(key=lambda stats: (-stats.score_deg, stats.node_id))
    dominant_node_id = node_stats[0].node_id if node_stats and node_stats[0].score_deg > 0.0 else None
    dominant_margin_deg = 0.0
    if len(node_stats) >= 2:
        dominant_margin_deg = node_stats[0].score_deg - node_stats[1].score_deg

    palm_motion_deg = 0.0
    if palm_quats:
        baseline = palm_quats[0]
        palm_motion_deg = max(quaternion_distance_deg(baseline, quaternion) for quaternion in palm_quats)

    return IsolationFileResult(
        path=path,
        finger_label=detect_finger_label(path),
        palm_motion_deg=palm_motion_deg,
        dominant_node_id=dominant_node_id,
        dominant_margin_deg=dominant_margin_deg,
        node_stats=tuple(node_stats),
    )


def summarize_consensus(results: list[IsolationFileResult]) -> list[IsolationFingerConsensus]:
    grouped: dict[str, list[IsolationFileResult]] = {finger: [] for finger in FINGER_NAMES}
    for result in results:
        if result.finger_label in grouped:
            grouped[result.finger_label].append(result)

    consensuses: list[IsolationFingerConsensus] = []
    for finger_label in FINGER_NAMES:
        finger_results = grouped[finger_label]
        if not finger_results:
            consensuses.append(
                IsolationFingerConsensus(
                    finger_label=finger_label,
                    file_count=0,
                    winning_node_id=None,
                    stable=False,
                    details="No files detected for this finger.",
                )
            )
            continue

        winning_nodes = [result.dominant_node_id for result in finger_results if result.dominant_node_id is not None]
        unique_nodes = sorted({node_id for node_id in winning_nodes if node_id is not None})
        stable = len(unique_nodes) == 1 and len(winning_nodes) == len(finger_results)
        winning_node_id = unique_nodes[0] if stable else None
        details = ", ".join(
            f"{result.path.name}: node {result.dominant_node_id if result.dominant_node_id is not None else '?'} "
            f"(margin {result.dominant_margin_deg:.1f} deg, palm {result.palm_motion_deg:.1f} deg)"
            for result in finger_results
        )
        consensuses.append(
            IsolationFingerConsensus(
                finger_label=finger_label,
                file_count=len(finger_results),
                winning_node_id=winning_node_id,
                stable=stable,
                details=details,
            )
        )
    return consensuses


def format_result_text(result: IsolationFileResult) -> str:
    lines = [
        f"{result.path.name}: dominant node {result.dominant_node_id if result.dominant_node_id is not None else '?'}",
        f"  palm motion {result.palm_motion_deg:.1f} deg",
        f"  dominant margin {result.dominant_margin_deg:.1f} deg",
    ]
    for stats in result.node_stats:
        lines.append(
            f"  node {stats.node_id}: score {stats.score_deg:.1f} deg "
            f"(quat {stats.max_relative_rotation_deg:.1f}, yaw {stats.yaw_range_deg:.1f}, "
            f"pitch {stats.pitch_range_deg:.1f}, roll {stats.roll_range_deg:.1f}, "
            f"samples {stats.sample_count})"
        )
    return "\n".join(lines)


def format_consensus_text(results: list[IsolationFileResult]) -> str:
    lines = []
    for consensus in summarize_consensus(results):
        if consensus.file_count == 0:
            lines.append(f"{consensus.finger_label}: no capture files")
        elif consensus.stable:
            lines.append(
                f"{consensus.finger_label}: stable -> node {consensus.winning_node_id} across {consensus.file_count} files"
            )
        else:
            lines.append(
                f"{consensus.finger_label}: not stable yet across {consensus.file_count} files"
            )
        lines.append(f"  {consensus.details}")
    return "\n".join(lines)


def _build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Analyze single-finger isolation capture CSVs.")
    parser.add_argument("csv_files", nargs="+", help="Isolation capture CSV files to analyze.")
    return parser


def main() -> None:
    parser = _build_argument_parser()
    args = parser.parse_args()
    results = [analyze_isolation_csv(path) for path in args.csv_files]
    for result in results:
        print(format_result_text(result))
        print()
    print("Consensus")
    print(format_consensus_text(results))


if __name__ == "__main__":
    main()
