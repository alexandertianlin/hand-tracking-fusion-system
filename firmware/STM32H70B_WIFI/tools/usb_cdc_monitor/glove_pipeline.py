from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
from typing import Iterable, Mapping, Sequence


Quat = tuple[float, float, float, float]
Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class FingerSpec:
    name: str
    node_id: int


DEFAULT_FINGER_SPECS: tuple[FingerSpec, ...] = (
    FingerSpec("thumb", 20),
    FingerSpec("index", 30),
    FingerSpec("middle", 40),
    FingerSpec("ring", 50),
    FingerSpec("pinky", 60),
)


@dataclass(frozen=True)
class FingerCalibration:
    name: str
    node_id: int
    neutral_rel_quat: Quat
    sample_count: int


@dataclass(frozen=True)
class GloveCalibration:
    schema_version: str
    created_at_utc: str
    q_palm_ref: Quat
    fingers: dict[int, FingerCalibration]
    calibration_id: str


@dataclass(frozen=True)
class FingerSnapshot:
    name: str
    node_id: int
    present: bool
    valid: bool
    calibrated: bool
    quat_rel: Quat
    quat_rel_calibrated: Quat
    flex_deg: float
    swing_deg: float
    twist_deg: float
    status: int = 0


@dataclass(frozen=True)
class GloveSnapshot:
    calibrated: bool
    calibration_id: str | None
    palm_quat: Quat
    palm_ypr_deg: tuple[float, float, float]
    fingers: tuple[FingerSnapshot, ...] = field(default_factory=tuple)


def quaternion_normalize(quaternion: Quat) -> Quat:
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


def quaternion_conjugate(quaternion: Quat) -> Quat:
    w, x, y, z = quaternion
    return (w, -x, -y, -z)


def quaternion_inverse(quaternion: Quat) -> Quat:
    return quaternion_conjugate(quaternion_normalize(quaternion))


def quaternion_multiply(left: Quat, right: Quat) -> Quat:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return quaternion_normalize(
        (
            (lw * rw) - (lx * rx) - (ly * ry) - (lz * rz),
            (lw * rx) + (lx * rw) + (ly * rz) - (lz * ry),
            (lw * ry) - (lx * rz) + (ly * rw) + (lz * rx),
            (lw * rz) + (lx * ry) - (ly * rx) + (lz * rw),
        )
    )


def quaternion_to_euler_deg(quaternion: Quat) -> tuple[float, float, float]:
    w, x, y, z = quaternion_normalize(quaternion)

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.degrees(math.asin(sinp))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    return (yaw, pitch, roll)


def palm_ypr(quaternion: Quat) -> tuple[float, float, float]:
    yaw_deg, pitch_deg, roll_deg = quaternion_to_euler_deg(quaternion)
    return (yaw_deg, roll_deg, pitch_deg)


def finger_rel(q_palm: Quat, q_tip: Quat) -> Quat:
    return quaternion_multiply(quaternion_inverse(q_palm), quaternion_normalize(q_tip))


def apply_calib(q_rel: Quat, neutral_rel_quat: Quat | None) -> Quat:
    if neutral_rel_quat is None:
        return quaternion_normalize(q_rel)
    return quaternion_multiply(quaternion_inverse(neutral_rel_quat), quaternion_normalize(q_rel))


def _dot(left: Quat, right: Quat) -> float:
    return sum(a * b for a, b in zip(left, right))


def mean_quaternion(samples: Sequence[Quat]) -> Quat:
    if not samples:
        return (1.0, 0.0, 0.0, 0.0)

    reference = quaternion_normalize(samples[0])
    acc = [0.0, 0.0, 0.0, 0.0]
    for sample in samples:
        aligned = quaternion_normalize(sample)
        if _dot(aligned, reference) < 0.0:
            aligned = (-aligned[0], -aligned[1], -aligned[2], -aligned[3])
        acc[0] += aligned[0]
        acc[1] += aligned[1]
        acc[2] += aligned[2]
        acc[3] += aligned[3]
    return quaternion_normalize((acc[0], acc[1], acc[2], acc[3]))


def _quat_to_rotvec_deg(quaternion: Quat) -> Vec3:
    w, x, y, z = quaternion_normalize(quaternion)
    vector_norm = math.sqrt((x * x) + (y * y) + (z * z))
    if vector_norm <= 1e-8:
        return (0.0, 0.0, 0.0)

    angle_rad = 2.0 * math.atan2(vector_norm, w)
    axis = (x / vector_norm, y / vector_norm, z / vector_norm)
    angle_deg = math.degrees(angle_rad)
    return (axis[0] * angle_deg, axis[1] * angle_deg, axis[2] * angle_deg)


def swing_twist_decompose(q_rel: Quat, axis: Vec3 = (1.0, 0.0, 0.0)) -> tuple[Quat, Quat]:
    q_rel = quaternion_normalize(q_rel)
    ax, ay, az = axis
    axis_norm = math.sqrt((ax * ax) + (ay * ay) + (az * az))
    if axis_norm <= 0.0:
        raise ValueError("axis must be non-zero")
    ax /= axis_norm
    ay /= axis_norm
    az /= axis_norm

    w, x, y, z = q_rel
    projection = (x * ax) + (y * ay) + (z * az)
    q_twist = quaternion_normalize((w, ax * projection, ay * projection, az * projection))
    q_swing = quaternion_multiply(q_rel, quaternion_inverse(q_twist))
    return q_swing, q_twist


def swing_twist_angles(q_rel: Quat, axis: Vec3 = (1.0, 0.0, 0.0)) -> tuple[float, float, float]:
    q_swing, q_twist = swing_twist_decompose(q_rel, axis=axis)
    swing_rotvec_deg = _quat_to_rotvec_deg(q_swing)
    twist_rotvec_deg = _quat_to_rotvec_deg(q_twist)
    # Host-side interpretation tweak: on the current glove hardware, the motion
    # users perceive as curl tracks the former twist-about-X channel better than
    # the former swing-Y channel. Keep this in Python first until validated.
    flex_deg = twist_rotvec_deg[0]
    swing_deg = swing_rotvec_deg[2]
    twist_deg = swing_rotvec_deg[1]
    return flex_deg, swing_deg, twist_deg


def _healthy_status(status: int) -> bool:
    return (status & 0x04) != 0 and (status & 0x20) == 0 and (status & 0x40) == 0


def build_calibration(
    palm_samples: Sequence[Quat],
    relative_samples_by_node: Mapping[int, Sequence[Quat]],
    finger_specs: Sequence[FingerSpec] = DEFAULT_FINGER_SPECS,
) -> GloveCalibration:
    if not palm_samples:
        raise ValueError("need at least one palm sample for calibration")

    q_palm_ref = mean_quaternion(palm_samples)
    finger_entries: dict[int, FingerCalibration] = {}
    for spec in finger_specs:
        rel_samples = list(relative_samples_by_node.get(spec.node_id, ()))
        if not rel_samples:
            continue
        neutral_rel_quat = mean_quaternion(rel_samples)
        finger_entries[spec.node_id] = FingerCalibration(
            name=spec.name,
            node_id=spec.node_id,
            neutral_rel_quat=neutral_rel_quat,
            sample_count=len(rel_samples),
        )

    payload = {
        "schema_version": "glove-calibration-1.0",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "q_palm_ref": list(q_palm_ref),
        "fingers": [
            {
                "name": entry.name,
                "node_id": entry.node_id,
                "neutral_rel_quat": list(entry.neutral_rel_quat),
                "sample_count": entry.sample_count,
            }
            for entry in sorted(finger_entries.values(), key=lambda item: item.node_id)
        ],
    }
    serialized = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    calibration_id = hashlib.sha1(serialized.encode("utf-8")).hexdigest()[:8]

    return GloveCalibration(
        schema_version=payload["schema_version"],
        created_at_utc=payload["created_at_utc"],
        q_palm_ref=q_palm_ref,
        fingers=finger_entries,
        calibration_id=calibration_id,
    )


def calibration_to_json_dict(calibration: GloveCalibration) -> dict[str, object]:
    return {
        "schema_version": calibration.schema_version,
        "created_at_utc": calibration.created_at_utc,
        "calibration_id": calibration.calibration_id,
        "q_palm_ref": list(calibration.q_palm_ref),
        "fingers": [
            {
                "name": entry.name,
                "node_id": entry.node_id,
                "neutral_rel_quat": list(entry.neutral_rel_quat),
                "sample_count": entry.sample_count,
            }
            for entry in sorted(calibration.fingers.values(), key=lambda item: item.node_id)
        ],
    }


def save_calibration(calibration: GloveCalibration, path: Path) -> None:
    path.write_text(json.dumps(calibration_to_json_dict(calibration), indent=2) + "\n", encoding="utf-8")


def load_calibration(path: Path) -> GloveCalibration:
    raw = json.loads(path.read_text(encoding="utf-8"))
    fingers: dict[int, FingerCalibration] = {}
    for item in raw.get("fingers", []):
        node_id = int(item["node_id"])
        fingers[node_id] = FingerCalibration(
            name=str(item["name"]),
            node_id=node_id,
            neutral_rel_quat=tuple(float(value) for value in item["neutral_rel_quat"]),
            sample_count=int(item.get("sample_count", 0)),
        )
    return GloveCalibration(
        schema_version=str(raw["schema_version"]),
        created_at_utc=str(raw["created_at_utc"]),
        calibration_id=str(raw["calibration_id"]),
        q_palm_ref=tuple(float(value) for value in raw["q_palm_ref"]),
        fingers=fingers,
    )


def compute_snapshot(
    palm_quat: Quat,
    finger_quats_by_node: Mapping[int, Quat],
    finger_status_by_node: Mapping[int, int] | None = None,
    calibration: GloveCalibration | None = None,
    finger_specs: Sequence[FingerSpec] = DEFAULT_FINGER_SPECS,
) -> GloveSnapshot:
    palm_quat = quaternion_normalize(palm_quat)
    status_lookup = finger_status_by_node or {}
    fingers: list[FingerSnapshot] = []
    for spec in finger_specs:
        q_tip = finger_quats_by_node.get(spec.node_id)
        status = int(status_lookup.get(spec.node_id, 0))
        if q_tip is None:
            fingers.append(
                FingerSnapshot(
                    name=spec.name,
                    node_id=spec.node_id,
                    present=False,
                    valid=False,
                    calibrated=False,
                    quat_rel=(1.0, 0.0, 0.0, 0.0),
                    quat_rel_calibrated=(1.0, 0.0, 0.0, 0.0),
                    flex_deg=0.0,
                    swing_deg=0.0,
                    twist_deg=0.0,
                    status=status,
                )
            )
            continue

        q_rel = finger_rel(palm_quat, q_tip)
        calibration_entry = None if calibration is None else calibration.fingers.get(spec.node_id)
        q_rel_calibrated = apply_calib(q_rel, None if calibration_entry is None else calibration_entry.neutral_rel_quat)
        flex_deg, swing_deg, twist_deg = swing_twist_angles(q_rel_calibrated)
        fingers.append(
            FingerSnapshot(
                name=spec.name,
                node_id=spec.node_id,
                present=True,
                valid=_healthy_status(status),
                calibrated=calibration_entry is not None,
                quat_rel=q_rel,
                quat_rel_calibrated=q_rel_calibrated,
                flex_deg=flex_deg,
                swing_deg=swing_deg,
                twist_deg=twist_deg,
                status=status,
            )
        )

    return GloveSnapshot(
        calibrated=calibration is not None,
        calibration_id=None if calibration is None else calibration.calibration_id,
        palm_quat=palm_quat,
        palm_ypr_deg=palm_ypr(palm_quat),
        fingers=tuple(fingers),
    )
