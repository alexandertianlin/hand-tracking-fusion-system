from __future__ import annotations

from collections import deque
import csv
from dataclasses import dataclass
from datetime import datetime
import json
import math
from pathlib import Path
import queue
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import serial
from serial import SerialException
from serial.tools import list_ports

from isolation_analysis import analyze_isolation_csv, format_consensus_text, format_result_text
import glove_pipeline
from palm_parser import FrameParser, ParserStats, ParsedFrame
import phase0_autodetect

try:
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure

    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False


APP_TITLE = "STM32 USB CDC Binary Monitor"
DEFAULT_BAUDRATE = 115200
PLOT_HISTORY = 200
DRIFT_WINDOW_S = 10.0
MOTION_EVENT_DEG = 15.0
UI_POLL_MS = 50
UI_FRAME_BATCH_LIMIT = 120
HEAVY_UI_REFRESH_S = 0.10
RAW_FRAME_REFRESH_S = 0.10
POSE_STATUS_STALE_S = 0.35
POSE_DEFAULT_HOLD_S = 5.0
POSE_DEFAULT_TOLERANCE_DEG = 35.0
PHASE0_AUTO_CAPTURE_S = 3.0
GLOVE_CALIBRATION_CAPTURE_S = 2.0
GLOVE_CALIBRATION_FILENAME = "calibration.json"
GLOVE_CAPTURE_JSONL_SUFFIX = ".glove.jsonl"
IMU_DRIFT_DURATION_PRESETS = ("10", "30", "80", "120", "Custom")
IMU_DRIFT_CUSTOM_DEFAULT_S = 180.0
TRANSPORT_PRESETS = {
    "Palm USB (115200)": "115200",
    "Fingertip UART (460800)": "460800",
    "UART 230400": "230400",
    "UART 921600": "921600",
    "Custom": "",
}



@dataclass
class ReaderEvent:
    kind: str
    frame: ParsedFrame | None = None
    stats: ParserStats | None = None
    message: str = ""


@dataclass(frozen=True)
class AngleSample:
    time_s: float
    yaw_deg: float
    yaw_unwrapped_deg: float
    pitch_deg: float
    roll_deg: float
    status: int


@dataclass(frozen=True)
class PoseTestStep:
    key: str
    label: str
    instruction: str
    target_euler_deg: tuple[float, float, float]
    hold_seconds: float = POSE_DEFAULT_HOLD_S
    tolerance_deg: float = POSE_DEFAULT_TOLERANCE_DEG


@dataclass(frozen=True)
class ImuDriftRanking:
    node_id: int
    label: str
    duration_s: float
    dominant_axis: str
    dominant_rate: float
    yaw_rate: float
    pitch_rate: float
    roll_rate: float
    score: float


POSE_TEST_STEPS = (
    PoseTestStep("flat", "Flat", "Keep the selected node flat for 5.0 s.", (0.0, 0.0, 0.0)),
    PoseTestStep("hi_five", "Hi-five", "Move to a hi-five pose and hold for 5.0 s.", (0.0, -90.0, 0.0)),
    PoseTestStep("face_down", "Face-down", "Rotate face-down and hold for 5.0 s.", (0.0, 180.0, 0.0)),
    PoseTestStep(
        "palm_left_together",
        "Palm Left",
        "Turn palm-left with all fingers together for 5.0 s.",
        (-90.0, 0.0, 0.0),
    ),
    PoseTestStep(
        "palm_right_together",
        "Palm Right",
        "Turn palm-right with all fingers together for 5.0 s.",
        (90.0, 0.0, 0.0),
    ),
    PoseTestStep(
        "face_down_spread",
        "Face-down Spread",
        "Hold face-down with all fingers spread for 5.0 s.",
        (0.0, 180.0, 0.0),
    ),
)


@dataclass
class FusedNodeState:
    node_id: int
    packet_count: int = 0
    last_frame: ParsedFrame | None = None
    last_frame_monotonic: float | None = None
    session_start_monotonic: float | None = None
    last_yaw_unwrapped: float | None = None
    angle_history: deque[AngleSample] | None = None
    capture_samples: list[AngleSample] | None = None
    capture_status_issue_flags: set[str] | None = None
    capture_status_samples: int = 0
    capture_status_good_samples: int = 0

    def __post_init__(self) -> None:
        if self.angle_history is None:
            self.angle_history = deque(maxlen=4096)
        if self.capture_samples is None:
            self.capture_samples = []
        if self.capture_status_issue_flags is None:
            self.capture_status_issue_flags = set()


def quaternion_to_euler_deg(quaternion: tuple[float, float, float, float]) -> tuple[float, float, float]:
    w, x, y, z = quaternion

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.degrees(math.asin(sinp))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    return yaw, roll, pitch


def unwrap_angle_deg(previous_unwrapped: float | None, angle_deg: float) -> float:
    if previous_unwrapped is None:
        return angle_deg

    delta = angle_deg - previous_unwrapped
    while delta > 180.0:
        delta -= 360.0
    while delta < -180.0:
        delta += 360.0
    return previous_unwrapped + delta


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


def quaternion_conjugate(quaternion: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    w, x, y, z = quaternion
    return (w, -x, -y, -z)


def quaternion_multiply(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return (
        (lw * rw) - (lx * rx) - (ly * ry) - (lz * rz),
        (lw * rx) + (lx * rw) + (ly * rz) - (lz * ry),
        (lw * ry) - (lx * rz) + (ly * rw) + (lz * rx),
        (lw * rz) + (lx * ry) - (ly * rx) + (lz * rw),
    )


def euler_deg_to_quaternion(yaw_deg: float, pitch_deg: float, roll_deg: float) -> tuple[float, float, float, float]:
    yaw = math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)
    roll = math.radians(roll_deg)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return quaternion_normalize(
        (
            (cr * cp * cy) + (sr * sp * sy),
            (sr * cp * cy) - (cr * sp * sy),
            (cr * sp * cy) + (sr * cp * sy),
            (cr * cp * sy) - (sr * sp * cy),
        )
    )


def quaternion_relative_to(
    baseline: tuple[float, float, float, float],
    current: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    return quaternion_normalize(quaternion_multiply(quaternion_conjugate(baseline), current))


def quaternion_distance_deg(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> float:
    dot = sum(a * b for a, b in zip(quaternion_normalize(left), quaternion_normalize(right)))
    dot = max(-1.0, min(1.0, abs(dot)))
    return math.degrees(2.0 * math.acos(dot))


def quaternion_rotation_angle_deg(quaternion: tuple[float, float, float, float]) -> float:
    normalized = quaternion_normalize(quaternion)
    scalar = max(-1.0, min(1.0, abs(normalized[0])))
    return math.degrees(2.0 * math.acos(scalar))


def format_rate_per_10s(rate_deg_per_10s: float | None) -> str:
    if rate_deg_per_10s is None:
        return "-"
    return f"{rate_deg_per_10s:+.2f} deg/10s"


def clone_stats(stats: ParserStats) -> ParserStats:
    return ParserStats(
        packets=stats.packets,
        raw_packets=stats.raw_packets,
        fused_packets=stats.fused_packets,
        checksum_errors=stats.checksum_errors,
        sequence_gaps=stats.sequence_gaps,
        bytes_discarded=stats.bytes_discarded,
    )


class SerialReader(threading.Thread):
    def __init__(self, port: str, baudrate: int, output_queue: queue.Queue[ReaderEvent]) -> None:
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.output_queue = output_queue
        self.stop_event = threading.Event()
        self.serial_handle: serial.Serial | None = None

    def stop(self) -> None:
        self.stop_event.set()
        handle = self.serial_handle
        if handle is not None and handle.is_open:
            try:
                handle.close()
            except SerialException:
                pass

    def run(self) -> None:
        parser = FrameParser()
        last_stats = clone_stats(parser.stats)

        try:
            self.serial_handle = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1,
            )
            self.output_queue.put(ReaderEvent(kind="connected", message=self.port))

            while not self.stop_event.is_set():
                chunk = self.serial_handle.read(256)
                frames = parser.feed(chunk)

                if frames:
                    current_stats = clone_stats(parser.stats)
                    last_stats = current_stats
                    for frame in frames:
                        self.output_queue.put(
                            ReaderEvent(kind="frame", frame=frame, stats=current_stats)
                        )
                    continue

                current_stats = clone_stats(parser.stats)
                if current_stats != last_stats:
                    last_stats = current_stats
                    self.output_queue.put(ReaderEvent(kind="stats", stats=current_stats))

        except SerialException as exc:
            if not self.stop_event.is_set():
                self.output_queue.put(ReaderEvent(kind="error", message=str(exc)))
        finally:
            handle = self.serial_handle
            if handle is not None and handle.is_open:
                try:
                    handle.close()
                except SerialException:
                    pass
            self.serial_handle = None
            self.output_queue.put(ReaderEvent(kind="disconnected", message=self.port))


class MonitorApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title(APP_TITLE)
        self.root.minsize(980, 760)

        self.reader_queue: queue.Queue[ReaderEvent] = queue.Queue()
        self.reader: SerialReader | None = None
        self.last_frame: ParsedFrame | None = None
        self.last_stats = ParserStats()
        self.connected_port = ""

        self.port_var = tk.StringVar()
        self.baudrate_var = tk.StringVar(value=str(DEFAULT_BAUDRATE))
        self.transport_preset_var = tk.StringVar(value="Palm USB (115200)")
        self.connection_var = tk.StringVar(value="Disconnected")
        self.packet_var = tk.StringVar(value="0")
        self.raw_packet_var = tk.StringVar(value="0")
        self.fused_packet_var = tk.StringVar(value="0")
        self.checksum_var = tk.StringVar(value="0")
        self.gap_var = tk.StringVar(value="0")
        self.discarded_var = tk.StringVar(value="0")
        self.protocol_var = tk.StringVar(value="-")
        self.sequence_var = tk.StringVar(value="-")
        self.node_id_var = tk.StringVar(value="-")
        self.selected_node_var = tk.StringVar(value="palm-local (0)")
        self.selected_node_label_var = tk.StringVar(value="palm-local (0)")
        self.status_hex_var = tk.StringVar(value="0x00")
        self.status_bits_var = tk.StringVar(value="-")
        self.delta_var = tk.StringVar(value="Waiting for frames")
        self.capture_state_var = tk.StringVar(value="Not recording")
        self.capture_file_var = tk.StringVar(value="-")
        self.validity_var = tk.StringVar(value="-")
        self.summary_var = tk.StringVar(value="No drift summary yet.")
        self.tuning_var = tk.StringVar(value="Capture a still test to get tuning advice.")
        self.imu_drift_duration_var = tk.StringVar(value="30")
        self.imu_drift_custom_duration_var = tk.StringVar(value=f"{IMU_DRIFT_CUSTOM_DEFAULT_S:.0f}")
        self.imu_drift_status_var = tk.StringVar(
            value="Idle. Choose a duration and run a drift test to rank nodes by drift."
        )
        self.imu_drift_result_var = tk.StringVar(value="No IMU drift test results yet.")
        self.plot_enabled_var = tk.BooleanVar(value=False)
        self.plot_mode_var = tk.StringVar(value="auto")
        self.pose_test_status_var = tk.StringVar(value="Not running")
        self.pose_test_instruction_var = tk.StringVar(value="Press Start Guided Test to begin.")
        self.pose_test_progress_var = tk.StringVar(value="-")
        self.pose_test_node_var = tk.StringVar(value="Reference: palm-local (0)")
        self.pose_test_summary_var = tk.StringVar(value="No guided test results yet.")
        self.pose_test_recording_var = tk.StringVar(value="Capture: idle")
        self.pose_test_step_vars = {
            step.key: tk.StringVar(value="pending")
            for step in POSE_TEST_STEPS
        }
        self.isolation_finger_var = tk.StringVar(value="index")
        self.isolation_round_var = tk.StringVar(value="1")
        self.isolation_status_var = tk.StringVar(value="No isolation capture yet.")
        self.isolation_summary_var = tk.StringVar(
            value="Use two rounds of index/middle/ring single-finger captures to confirm which node dominates."
        )
        self.glove_status_var = tk.StringVar(
            value="Phase 1 host math idle. Connect the palm and wait for fused fingertip frames."
        )
        self.glove_calibration_var = tk.StringVar(value="Calibration: not loaded")
        self.glove_palm_ypr_vars = {
            "yaw": tk.StringVar(value="-"),
            "pitch": tk.StringVar(value="-"),
            "roll": tk.StringVar(value="-"),
        }
        self.glove_finger_vars = {
            spec.node_id: {
                "present": tk.StringVar(value="no"),
                "valid": tk.StringVar(value="-"),
                "calibrated": tk.StringVar(value="no"),
                "flex": tk.StringVar(value="-"),
                "swing": tk.StringVar(value="-"),
                "twist": tk.StringVar(value="-"),
            }
            for spec in glove_pipeline.DEFAULT_FINGER_SPECS
        }

        self.imu_vars: dict[str, dict[str, tk.StringVar]] = {}
        self.quat_vars: dict[str, tk.StringVar] = {}
        self.euler_vars = {
            "yaw": tk.StringVar(value="-"),
            "pitch": tk.StringVar(value="-"),
            "roll": tk.StringVar(value="-"),
        }
        self.drift_vars = {
            "yaw": tk.StringVar(value="-"),
            "pitch": tk.StringVar(value="-"),
            "roll": tk.StringVar(value="-"),
        }
        self.raw_frame_text: tk.Text | None = None
        self.connect_button: ttk.Button | None = None
        self.disconnect_button: ttk.Button | None = None
        self.capture_button: ttk.Button | None = None
        self.stop_capture_button: ttk.Button | None = None
        self.port_combo: ttk.Combobox | None = None
        self.node_combo: ttk.Combobox | None = None
        self.node_tree: ttk.Treeview | None = None
        self.scroll_canvas: tk.Canvas | None = None
        self.scroll_container: ttk.Frame | None = None
        self.scroll_window_id: int | None = None
        self.plot_canvas = None
        self.figure = None
        self.axes = []
        self.plot_status_label: ttk.Label | None = None
        self.plot_history = {
            "imu0": [deque(maxlen=PLOT_HISTORY) for _ in range(3)],
            "imu1": [deque(maxlen=PLOT_HISTORY) for _ in range(3)],
        }
        self.quat_plot_history: dict[int, list[deque[float]]] = {}
        self.last_plot_update = 0.0
        self.capture_file = None
        self.capture_writer: csv.DictWriter | None = None
        self.capture_path = ""
        self.capture_start_monotonic: float | None = None
        self.capture_first_frame_kind: str | None = None
        self.capture_protocol_change_count = 0
        self.capture_samples_written = 0
        self.capture_last_summary = ""
        self.capture_status_samples = 0
        self.capture_status_good_samples = 0
        self.capture_status_issue_flags: set[str] = set()
        self.capture_angle_samples: list[AngleSample] = []
        self.fused_nodes: dict[int, FusedNodeState] = {}
        self.selected_node_id: int = 0
        self.pose_test_baselines: dict[int, tuple[float, float, float, float]] = {}
        self.pose_test_active = False
        self.pose_test_node_id: int | None = None
        self.pose_test_step_index = 0
        self.pose_test_step_started_monotonic: float | None = None
        self.pose_test_started_monotonic: float | None = None
        self.pose_test_last_sample_monotonic: float | None = None
        self.pose_test_last_error_deg: float | None = None
        self.pose_test_started_capture = False
        self.pose_test_results: dict[str, str] = {step.key: "pending" for step in POSE_TEST_STEPS}
        self.pose_test_target_quaternions = {
            step.key: euler_deg_to_quaternion(*step.target_euler_deg)
            for step in POSE_TEST_STEPS
        }
        self.pose_test_start_button: ttk.Button | None = None
        self.pose_test_zero_button: ttk.Button | None = None
        self.pose_test_cancel_button: ttk.Button | None = None
        self.isolation_start_button: ttk.Button | None = None
        self.isolation_stop_button: ttk.Button | None = None
        self.isolation_analyze_button: ttk.Button | None = None
        self.phase0_autodetect_button: ttk.Button | None = None
        self.phase0_analyze_button: ttk.Button | None = None
        self.imu_drift_start_button: ttk.Button | None = None
        self.glove_zero_button: ttk.Button | None = None
        self.glove_clear_zero_button: ttk.Button | None = None
        self.glove_calibrate_button: ttk.Button | None = None
        self.glove_reset_calibration_button: ttk.Button | None = None
        self.phase0_status_var = tk.StringVar(
            value="Lay all boards co-aligned on a flat surface, then click Auto-Detect."
        )
        self.phase0_summary_var = tk.StringVar(value="-")
        self._phase0_auto_in_progress = False
        self._phase0_auto_after_id: str | None = None
        self._imu_drift_test_in_progress = False
        self._imu_drift_test_after_id: str | None = None
        self._imu_drift_test_started_monotonic: float | None = None
        self._imu_drift_test_duration_s = 0.0
        self._imu_drift_test_auto_stop = False
        self._node_controls_dirty = False
        self._selected_details_dirty = False
        self._pose_test_dirty = False
        self._pending_raw_frame: ParsedFrame | None = None
        self._last_heavy_ui_refresh = 0.0
        self._last_raw_frame_refresh = 0.0
        self.capture_mode = "generic"
        self.glove_calibration: glove_pipeline.GloveCalibration | None = None
        self.glove_snapshot: glove_pipeline.GloveSnapshot | None = None
        self._glove_ui_dirty = False
        self._glove_calibration_capture_in_progress = False
        self._glove_calibration_after_id: str | None = None
        self._glove_calibration_palm_samples: list[glove_pipeline.Quat] = []
        self._glove_calibration_relative_samples: dict[int, list[glove_pipeline.Quat]] = {
            spec.node_id: [] for spec in glove_pipeline.DEFAULT_FINGER_SPECS
        }
        self.capture_jsonl_file = None

        self._build_ui()
        self.plot_mode_var.trace_add("write", self._on_plot_mode_changed)
        self.selected_node_var.trace_add("write", self._on_selected_node_changed)
        self.transport_preset_var.trace_add("write", self._on_transport_preset_changed)
        self.refresh_ports()
        self._load_glove_calibration()
        self.root.after(UI_POLL_MS, self.process_reader_events)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self) -> None:
        outer = ttk.Frame(self.root)
        outer.pack(fill=tk.BOTH, expand=True)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(0, weight=1)

        canvas = tk.Canvas(outer, highlightthickness=0)
        canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(outer, orient=tk.VERTICAL, command=canvas.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        canvas.configure(yscrollcommand=scrollbar.set)

        container = ttk.Frame(canvas, padding=10)
        container.columnconfigure(0, weight=1)
        self.scroll_canvas = canvas
        self.scroll_container = container
        self.scroll_window_id = canvas.create_window((0, 0), window=container, anchor="nw")

        container.bind("<Configure>", self._on_scroll_container_configure)
        canvas.bind("<Configure>", self._on_scroll_canvas_configure)
        canvas.bind_all("<MouseWheel>", self._on_mousewheel)

        self._build_connection_panel(container)
        self._build_counters_panel(container)
        self._build_imu_panel(container)
        self._build_drift_panel(container)
        self._build_pose_test_panel(container)
        self._build_phase0_panel(container)
        self._build_glove_panel(container)
        self._build_raw_panel(container)
        self._build_plot_panel(container)

    def _on_scroll_container_configure(self, _event: tk.Event) -> None:
        if self.scroll_canvas is None or self.scroll_container is None:
            return
        self.scroll_canvas.configure(scrollregion=self.scroll_canvas.bbox("all"))

    def _on_scroll_canvas_configure(self, event: tk.Event) -> None:
        if self.scroll_canvas is None or self.scroll_window_id is None:
            return
        self.scroll_canvas.itemconfigure(self.scroll_window_id, width=event.width)

    def _on_mousewheel(self, event: tk.Event) -> None:
        if self.scroll_canvas is None:
            return
        if event.delta == 0:
            return
        self.scroll_canvas.yview_scroll(int(-event.delta / 120), "units")

    def _on_transport_preset_changed(self, *_args: object) -> None:
        preset = self.transport_preset_var.get()
        baudrate = TRANSPORT_PRESETS.get(preset, "")
        if baudrate:
            self.baudrate_var.set(baudrate)

    def _build_connection_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Connection", padding=10)
        frame.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        for column in range(7):
            frame.columnconfigure(column, weight=1 if column == 1 else 0)

        ttk.Label(frame, text="Serial Port").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(frame, textvariable=self.port_var, state="readonly", width=28)
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=(6, 10))

        ttk.Button(frame, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=(0, 10))

        ttk.Label(frame, text="Baud").grid(row=0, column=3, sticky="e")
        ttk.Entry(frame, textvariable=self.baudrate_var, width=10).grid(row=0, column=4, sticky="w", padx=(6, 10))

        self.connect_button = ttk.Button(frame, text="Connect", command=self.connect)
        self.connect_button.grid(row=0, column=5, sticky="e")

        self.disconnect_button = ttk.Button(frame, text="Disconnect", command=self.disconnect, state=tk.DISABLED)
        self.disconnect_button.grid(row=0, column=6, sticky="e", padx=(8, 0))

        ttk.Label(frame, text="Preset").grid(row=1, column=0, sticky="w", pady=(10, 0))
        ttk.Combobox(
            frame,
            textvariable=self.transport_preset_var,
            state="readonly",
            values=tuple(TRANSPORT_PRESETS.keys()),
            width=24,
        ).grid(row=1, column=1, sticky="w", padx=(6, 10), pady=(10, 0))

        ttk.Label(frame, text="Status").grid(row=2, column=0, sticky="w", pady=(10, 0))
        ttk.Label(frame, textvariable=self.connection_var).grid(row=2, column=1, columnspan=6, sticky="w", pady=(10, 0))

        ttk.Label(frame, text="Capture").grid(row=3, column=0, sticky="w", pady=(10, 0))
        self.capture_button = ttk.Button(frame, text="Start Capture", command=self.start_capture)
        self.capture_button.grid(row=3, column=1, sticky="w", pady=(10, 0))
        self.stop_capture_button = ttk.Button(frame, text="Stop Capture", command=self.stop_capture, state=tk.DISABLED)
        self.stop_capture_button.grid(row=3, column=2, sticky="w", padx=(8, 0), pady=(10, 0))
        ttk.Label(frame, textvariable=self.capture_state_var).grid(row=3, column=3, columnspan=4, sticky="w", pady=(10, 0))

    def _build_drift_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Drift Analysis", padding=10)
        frame.grid(row=3, column=0, sticky="ew", pady=(0, 10))
        for column in range(6):
            frame.columnconfigure(column, weight=1)

        metrics = [
            ("Yaw (deg)", self.euler_vars["yaw"]),
            ("Pitch (deg)", self.euler_vars["pitch"]),
            ("Roll (deg)", self.euler_vars["roll"]),
            ("Yaw Drift", self.drift_vars["yaw"]),
            ("Pitch Drift", self.drift_vars["pitch"]),
            ("Roll Drift", self.drift_vars["roll"]),
            ("Validity", self.validity_var),
            ("Capture File", self.capture_file_var),
        ]

        for index, (label, variable) in enumerate(metrics):
            row = index // 2
            column = (index % 2) * 3
            ttk.Label(frame, text=label).grid(row=row, column=column, sticky="w", pady=2)
            ttk.Label(frame, textvariable=variable, wraplength=360).grid(
                row=row, column=column + 1, columnspan=2, sticky="w", pady=2, padx=(6, 10)
            )

        ttk.Label(frame, text="Summary").grid(row=4, column=0, sticky="nw", pady=(8, 0))
        ttk.Label(frame, textvariable=self.summary_var, wraplength=900, justify=tk.LEFT).grid(
            row=4, column=1, columnspan=5, sticky="w", pady=(8, 0)
        )
        ttk.Label(frame, text="Tuning Hint").grid(row=5, column=0, sticky="nw", pady=(8, 0))
        ttk.Label(frame, textvariable=self.tuning_var, wraplength=900, justify=tk.LEFT).grid(
            row=5, column=1, columnspan=5, sticky="w", pady=(8, 0)
        )

        ttk.Separator(frame, orient=tk.HORIZONTAL).grid(row=6, column=0, columnspan=6, sticky="ew", pady=(10, 8))
        ttk.Label(frame, text="IMU Drift Test").grid(row=7, column=0, sticky="w", pady=2)
        ttk.Combobox(
            frame,
            textvariable=self.imu_drift_duration_var,
            state="readonly",
            values=IMU_DRIFT_DURATION_PRESETS,
            width=12,
        ).grid(row=7, column=1, sticky="w", pady=2)
        ttk.Label(frame, text="Custom seconds").grid(row=7, column=2, sticky="e", pady=2)
        ttk.Entry(frame, textvariable=self.imu_drift_custom_duration_var, width=10).grid(
            row=7, column=3, sticky="w", pady=2
        )
        self.imu_drift_start_button = ttk.Button(
            frame,
            text="Start Drift Test",
            command=self.start_imu_drift_test,
        )
        self.imu_drift_start_button.grid(row=7, column=4, sticky="w", padx=(8, 0), pady=2)

        ttk.Label(frame, text="Test Status").grid(row=8, column=0, sticky="nw", pady=(8, 2))
        ttk.Label(frame, textvariable=self.imu_drift_status_var, wraplength=900, justify=tk.LEFT).grid(
            row=8, column=1, columnspan=5, sticky="w", pady=(8, 2)
        )
        ttk.Label(frame, text="Worst / Ranking").grid(row=9, column=0, sticky="nw", pady=(2, 0))
        ttk.Label(frame, textvariable=self.imu_drift_result_var, wraplength=900, justify=tk.LEFT).grid(
            row=9, column=1, columnspan=5, sticky="w", pady=(2, 0)
        )

    def _build_pose_test_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Guided Pose Test", padding=10)
        frame.grid(row=4, column=0, sticky="ew", pady=(0, 10))
        for column in range(6):
            frame.columnconfigure(column, weight=1)

        ttk.Label(frame, text="Pose Reference").grid(row=0, column=0, sticky="w", pady=2)
        ttk.Label(frame, textvariable=self.pose_test_node_var).grid(row=0, column=1, columnspan=5, sticky="w", pady=2)

        self.pose_test_start_button = ttk.Button(frame, text="Start Guided Test", command=self.start_pose_test)
        self.pose_test_start_button.grid(row=1, column=0, sticky="w", pady=(6, 2))
        self.pose_test_zero_button = ttk.Button(frame, text="Reset Current Pose To Zero", command=self.reset_pose_test_zero)
        self.pose_test_zero_button.grid(row=1, column=1, columnspan=2, sticky="w", padx=(8, 0), pady=(6, 2))
        self.pose_test_cancel_button = ttk.Button(frame, text="Cancel / Clear", command=self.cancel_pose_test)
        self.pose_test_cancel_button.grid(row=1, column=3, sticky="w", padx=(8, 0), pady=(6, 2))

        ttk.Label(frame, text="Status").grid(row=2, column=0, sticky="w", pady=(8, 2))
        ttk.Label(frame, textvariable=self.pose_test_status_var).grid(row=2, column=1, columnspan=5, sticky="w", pady=(8, 2))
        ttk.Label(frame, text="Instruction").grid(row=3, column=0, sticky="nw", pady=2)
        ttk.Label(frame, textvariable=self.pose_test_instruction_var, wraplength=900, justify=tk.LEFT).grid(
            row=3, column=1, columnspan=5, sticky="w", pady=2
        )
        ttk.Label(frame, text="Progress").grid(row=4, column=0, sticky="w", pady=2)
        ttk.Label(frame, textvariable=self.pose_test_progress_var).grid(row=4, column=1, columnspan=2, sticky="w", pady=2)
        ttk.Label(frame, text="Recording").grid(row=4, column=3, sticky="w", pady=2)
        ttk.Label(frame, textvariable=self.pose_test_recording_var).grid(row=4, column=4, columnspan=2, sticky="w", pady=2)

        ttk.Label(frame, text="Step Results").grid(row=5, column=0, sticky="nw", pady=(8, 2))
        results = ttk.Frame(frame)
        results.grid(row=5, column=1, columnspan=5, sticky="ew", pady=(8, 2))
        for index in range(3):
            results.columnconfigure(index * 2, weight=0)
            results.columnconfigure(index * 2 + 1, weight=1)
        for index, step in enumerate(POSE_TEST_STEPS):
            row = index // 3
            column = (index % 3) * 2
            ttk.Label(results, text=step.label).grid(row=row, column=column, sticky="w", pady=2)
            ttk.Label(results, textvariable=self.pose_test_step_vars[step.key]).grid(
                row=row, column=column + 1, sticky="w", padx=(6, 16), pady=2
            )

        ttk.Label(frame, text="Summary").grid(row=6, column=0, sticky="nw", pady=(8, 0))
        ttk.Label(frame, textvariable=self.pose_test_summary_var, wraplength=900, justify=tk.LEFT).grid(
            row=6, column=1, columnspan=5, sticky="w", pady=(8, 0)
        )

        ttk.Separator(frame, orient=tk.HORIZONTAL).grid(row=7, column=0, columnspan=6, sticky="ew", pady=(10, 8))
        ttk.Label(frame, text="Isolation Finger").grid(row=8, column=0, sticky="w", pady=2)
        ttk.Combobox(
            frame,
            textvariable=self.isolation_finger_var,
            state="readonly",
            values=("index", "middle", "ring"),
            width=12,
        ).grid(row=8, column=1, sticky="w", pady=2)
        ttk.Label(frame, text="Round").grid(row=8, column=2, sticky="w", pady=2)
        ttk.Combobox(
            frame,
            textvariable=self.isolation_round_var,
            state="readonly",
            values=("1", "2"),
            width=6,
        ).grid(row=8, column=3, sticky="w", pady=2)
        self.isolation_start_button = ttk.Button(
            frame,
            text="Start Isolation Capture",
            command=self.start_isolation_capture,
        )
        self.isolation_start_button.grid(row=8, column=4, sticky="w", padx=(8, 0), pady=2)
        self.isolation_stop_button = ttk.Button(
            frame,
            text="Stop Isolation Capture",
            command=self.stop_capture,
            state=tk.DISABLED,
        )
        self.isolation_stop_button.grid(row=8, column=5, sticky="w", padx=(8, 0), pady=2)
        self.isolation_analyze_button = ttk.Button(
            frame,
            text="Analyze Isolation CSVs",
            command=self.analyze_isolation_files,
        )
        self.isolation_analyze_button.grid(row=9, column=1, sticky="w", pady=(8, 2))

        ttk.Label(frame, text="Isolation Status").grid(row=10, column=0, sticky="nw", pady=(8, 2))
        ttk.Label(frame, textvariable=self.isolation_status_var, wraplength=900, justify=tk.LEFT).grid(
            row=10, column=1, columnspan=5, sticky="w", pady=(8, 2)
        )
        ttk.Label(frame, text="Isolation Summary").grid(row=11, column=0, sticky="nw", pady=(4, 0))
        ttk.Label(frame, textvariable=self.isolation_summary_var, wraplength=900, justify=tk.LEFT).grid(
            row=11, column=1, columnspan=5, sticky="w", pady=(4, 0)
        )

    def _build_counters_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Stream Stats", padding=10)
        frame.grid(row=1, column=0, sticky="ew", pady=(0, 10))
        for column in range(8):
            frame.columnconfigure(column, weight=1)

        pairs = [
            ("Packets", self.packet_var),
            ("Raw Frames", self.raw_packet_var),
            ("Fused Frames", self.fused_packet_var),
            ("Checksum Errors", self.checksum_var),
            ("Sequence Gaps", self.gap_var),
            ("Bytes Discarded", self.discarded_var),
            ("Protocol", self.protocol_var),
            ("Last Sequence", self.sequence_var),
            ("Node ID", self.node_id_var),
            ("Status Byte", self.status_hex_var),
            ("Status Bits", self.status_bits_var),
            ("Section Delta", self.delta_var),
        ]

        for index, (label, variable) in enumerate(pairs):
            row = index // 4
            column = (index % 4) * 2
            ttk.Label(frame, text=label).grid(row=row, column=column, sticky="w", pady=2)
            ttk.Label(frame, textvariable=variable).grid(row=row, column=column + 1, sticky="w", pady=2, padx=(6, 10))

    def _build_imu_panel(self, parent: ttk.Frame) -> None:
        wrapper = ttk.Frame(parent)
        wrapper.grid(row=2, column=0, sticky="ew", pady=(0, 10))
        wrapper.columnconfigure(0, weight=1)
        wrapper.columnconfigure(1, weight=1)
        wrapper.rowconfigure(1, weight=1)

        self._build_single_imu_panel(wrapper, 0, "IMU0")
        self._build_single_imu_panel(wrapper, 1, "IMU1")
        self._build_fused_panel(wrapper)

    def _build_single_imu_panel(self, parent: ttk.Frame, column: int, title: str) -> None:
        frame = ttk.LabelFrame(parent, text=title, padding=10)
        frame.grid(row=0, column=column, sticky="nsew", padx=(0, 5) if column == 0 else (5, 0))
        for idx in range(6):
            frame.columnconfigure(idx, weight=1)

        labels = [
            ("Accel X (mg)", "accel_x"),
            ("Accel Y (mg)", "accel_y"),
            ("Accel Z (mg)", "accel_z"),
            ("Gyro X (dps)", "gyro_x"),
            ("Gyro Y (dps)", "gyro_y"),
            ("Gyro Z (dps)", "gyro_z"),
        ]

        vars_for_imu: dict[str, tk.StringVar] = {}
        for index, (label, key) in enumerate(labels):
            row = index // 2
            col = (index % 2) * 3
            variable = tk.StringVar(value="-")
            vars_for_imu[key] = variable
            ttk.Label(frame, text=label).grid(row=row, column=col, sticky="w", pady=2)
            ttk.Label(frame, textvariable=variable).grid(row=row, column=col + 1, sticky="w", pady=2, padx=(6, 10))

        self.imu_vars[title.lower()] = vars_for_imu

    def _build_fused_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Fused Nodes", padding=10)
        frame.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(10, 0))
        for idx in range(8):
            frame.columnconfigure(idx, weight=1)

        ttk.Label(frame, text="Selected Node").grid(row=0, column=0, sticky="w", pady=2)
        self.node_combo = ttk.Combobox(frame, textvariable=self.selected_node_var, state="readonly", width=24)
        self.node_combo.grid(row=0, column=1, columnspan=2, sticky="w", pady=2)
        ttk.Label(frame, textvariable=self.selected_node_label_var).grid(
            row=0, column=3, columnspan=5, sticky="w", pady=2
        )

        labels = [
            ("Quat W", "w"),
            ("Quat X", "x"),
            ("Quat Y", "y"),
            ("Quat Z", "z"),
        ]

        for index, (label, key) in enumerate(labels):
            variable = tk.StringVar(value="-")
            self.quat_vars[key] = variable
            col = index * 2
            ttk.Label(frame, text=label).grid(row=1, column=col, sticky="w", pady=2)
            ttk.Label(frame, textvariable=variable).grid(row=1, column=col + 1, sticky="w", pady=2, padx=(6, 10))

        columns = ("node", "role", "packets", "last_ms", "yaw", "pitch", "roll", "vs_palm", "status")
        self.node_tree = ttk.Treeview(frame, columns=columns, show="headings", height=5)
        headings = {
            "node": "Node",
            "role": "Role",
            "packets": "Packets",
            "last_ms": "Last ms",
            "yaw": "Yaw",
            "pitch": "Pitch",
            "roll": "Roll",
            "vs_palm": "vs Palm",
            "status": "Status",
        }
        widths = {
            "node": 60,
            "role": 110,
            "packets": 70,
            "last_ms": 70,
            "yaw": 70,
            "pitch": 70,
            "roll": 70,
            "vs_palm": 80,
            "status": 260,
        }
        for column in columns:
            self.node_tree.heading(column, text=headings[column])
            self.node_tree.column(column, width=widths[column], anchor="w")
        self.node_tree.grid(row=2, column=0, columnspan=8, sticky="ew", pady=(8, 0))
        self.node_tree.bind("<<TreeviewSelect>>", self._on_node_tree_select)

        ttk.Label(
            frame,
            text=(
                "vs Palm = angular distance between this node's quaternion and the palm quaternion. "
                "Place palm and one fingertip flat in the same orientation; a correctly-mapped node reads near 0 deg."
            ),
            wraplength=900,
            justify=tk.LEFT,
            foreground="#555555",
        ).grid(row=3, column=0, columnspan=8, sticky="w", pady=(6, 0))

    def _build_phase0_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Phase 0 Frame Alignment Auto-Detect", padding=10)
        frame.grid(row=5, column=0, sticky="ew", pady=(0, 10))
        for column in range(6):
            frame.columnconfigure(column, weight=1 if column == 5 else 0)

        intro = (
            "Lay the palm board and every fingertip board flat in the same physical orientation. "
            "Auto-Detect captures ~3 s and writes a JSON report classifying each finger's mounting "
            "axes (one of 24 right-handed permutations). Send the JSON to an agent for review."
        )
        ttk.Label(frame, text=intro, wraplength=900, justify=tk.LEFT).grid(
            row=0, column=0, columnspan=6, sticky="w", pady=(0, 8)
        )

        self.phase0_autodetect_button = ttk.Button(
            frame,
            text=f"Phase 0 Auto-Detect ({int(PHASE0_AUTO_CAPTURE_S)} s)",
            command=self.start_phase0_autodetect,
        )
        self.phase0_autodetect_button.grid(row=1, column=0, sticky="w")

        self.phase0_analyze_button = ttk.Button(
            frame,
            text="Analyze Capture (Phase 0)",
            command=self.analyze_phase0_capture,
        )
        self.phase0_analyze_button.grid(row=1, column=1, sticky="w", padx=(8, 0))

        ttk.Label(frame, text="Status").grid(row=2, column=0, sticky="w", pady=(8, 2))
        ttk.Label(
            frame,
            textvariable=self.phase0_status_var,
            wraplength=900,
            justify=tk.LEFT,
        ).grid(row=2, column=1, columnspan=5, sticky="w", pady=(8, 2))

        ttk.Label(frame, text="Last Report").grid(row=3, column=0, sticky="nw", pady=(2, 0))
        ttk.Label(
            frame,
            textvariable=self.phase0_summary_var,
            wraplength=900,
            justify=tk.LEFT,
        ).grid(row=3, column=1, columnspan=5, sticky="w", pady=(2, 0))

    def _build_glove_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Phase 1 Glove Math", padding=10)
        frame.grid(row=6, column=0, sticky="ew", pady=(0, 10))
        for column in range(8):
            frame.columnconfigure(column, weight=1)

        self.glove_zero_button = ttk.Button(frame, text="Zero All", command=self.send_zero_all)
        self.glove_zero_button.grid(row=0, column=0, sticky="w")
        self.glove_clear_zero_button = ttk.Button(frame, text="Clear Zero", command=self.clear_zero_all)
        self.glove_clear_zero_button.grid(row=0, column=1, sticky="w", padx=(8, 0))
        self.glove_calibrate_button = ttk.Button(
            frame,
            text=f"Calibrate Flat ({GLOVE_CALIBRATION_CAPTURE_S:.0f} s)",
            command=self.start_glove_calibration_capture,
        )
        self.glove_calibrate_button.grid(row=0, column=2, sticky="w", padx=(8, 0))
        self.glove_reset_calibration_button = ttk.Button(
            frame,
            text="Reset Calibration",
            command=self.reset_glove_calibration,
        )
        self.glove_reset_calibration_button.grid(row=0, column=3, sticky="w", padx=(8, 0))

        ttk.Label(frame, text="Status").grid(row=1, column=0, sticky="w", pady=(8, 2))
        ttk.Label(frame, textvariable=self.glove_status_var, wraplength=900, justify=tk.LEFT).grid(
            row=1, column=1, columnspan=7, sticky="w", pady=(8, 2)
        )
        ttk.Label(frame, textvariable=self.glove_calibration_var, wraplength=900, justify=tk.LEFT).grid(
            row=2, column=0, columnspan=8, sticky="w", pady=(0, 8)
        )

        ttk.Label(frame, text="Palm YPR").grid(row=3, column=0, sticky="w", pady=2)
        ttk.Label(frame, text="Yaw").grid(row=3, column=1, sticky="e", pady=2)
        ttk.Label(frame, textvariable=self.glove_palm_ypr_vars["yaw"]).grid(row=3, column=2, sticky="w", pady=2)
        ttk.Label(frame, text="Pitch").grid(row=3, column=3, sticky="e", pady=2)
        ttk.Label(frame, textvariable=self.glove_palm_ypr_vars["pitch"]).grid(row=3, column=4, sticky="w", pady=2)
        ttk.Label(frame, text="Roll").grid(row=3, column=5, sticky="e", pady=2)
        ttk.Label(frame, textvariable=self.glove_palm_ypr_vars["roll"]).grid(row=3, column=6, sticky="w", pady=2)

        headers = ("Finger", "Present", "Valid", "Cal", "Flex", "Swing", "Twist")
        for column, header in enumerate(headers):
            ttk.Label(frame, text=header).grid(row=4, column=column, sticky="w", pady=(10, 2))

        for row_index, spec in enumerate(glove_pipeline.DEFAULT_FINGER_SPECS, start=5):
            vars_for_finger = self.glove_finger_vars[spec.node_id]
            ttk.Label(frame, text=spec.name).grid(row=row_index, column=0, sticky="w", pady=2)
            ttk.Label(frame, textvariable=vars_for_finger["present"]).grid(row=row_index, column=1, sticky="w", pady=2)
            ttk.Label(frame, textvariable=vars_for_finger["valid"]).grid(row=row_index, column=2, sticky="w", pady=2)
            ttk.Label(frame, textvariable=vars_for_finger["calibrated"]).grid(row=row_index, column=3, sticky="w", pady=2)
            ttk.Label(frame, textvariable=vars_for_finger["flex"]).grid(row=row_index, column=4, sticky="w", pady=2)
            ttk.Label(frame, textvariable=vars_for_finger["swing"]).grid(row=row_index, column=5, sticky="w", pady=2)
            ttk.Label(frame, textvariable=vars_for_finger["twist"]).grid(row=row_index, column=6, sticky="w", pady=2)

        ttk.Label(
            frame,
            text=(
                "Zero All sends {0xC0, 0x01}. Clear Zero sends {0xC0, 0x00}. "
                "Calibrate Flat stores host-side per-finger neutral offsets in calibration.json."
            ),
            wraplength=900,
            justify=tk.LEFT,
            foreground="#555555",
        ).grid(row=10, column=0, columnspan=8, sticky="w", pady=(8, 0))

    def _build_raw_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Last Accepted Frame", padding=10)
        frame.grid(row=7, column=0, sticky="nsew", pady=(0, 10))
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(1, weight=1)

        ttk.Label(
            frame,
            text="Raw: D6|01|seq|IMU0|IMU1|status|crc   Fused: B6|force(6B)|quat(8B)|status|node|crc",
        ).grid(row=0, column=0, sticky="w")

        self.raw_frame_text = tk.Text(frame, height=6, wrap=tk.WORD)
        self.raw_frame_text.grid(row=1, column=0, sticky="nsew", pady=(8, 0))
        self.raw_frame_text.insert("1.0", "No valid frame received yet.")
        self.raw_frame_text.configure(state=tk.DISABLED)

    def _build_plot_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Optional Live Plots", padding=10)
        frame.grid(row=8, column=0, sticky="nsew")
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(1, weight=1)

        controls = ttk.Frame(frame)
        controls.grid(row=0, column=0, sticky="ew")
        controls.columnconfigure(4, weight=1)

        ttk.Checkbutton(
            controls,
            text="Enable Plots",
            variable=self.plot_enabled_var,
            command=self.on_plot_toggle,
        ).grid(row=0, column=0, sticky="w")

        ttk.Label(controls, text="Plot Mode").grid(row=0, column=1, sticky="w", padx=(10, 4))
        ttk.Combobox(
            controls,
            textvariable=self.plot_mode_var,
            state="readonly",
            values=("auto", "accel", "gyro", "quat"),
            width=10,
        ).grid(row=0, column=2, sticky="w")

        ttk.Button(controls, text="Clear History", command=self.clear_plot_history).grid(row=0, column=3, sticky="w", padx=(10, 0))

        if not MATPLOTLIB_AVAILABLE:
            self.plot_status_label = ttk.Label(
                frame,
                text="Install matplotlib to enable embedded live plots.",
            )
            self.plot_status_label.grid(row=1, column=0, sticky="w", pady=(10, 0))
            return

        self.figure = Figure(figsize=(8.5, 3.8), dpi=100)
        axis0 = self.figure.add_subplot(211)
        axis1 = self.figure.add_subplot(212)
        self.axes = [axis0, axis1]
        self.figure.tight_layout(pad=2.0)
        self.plot_canvas = FigureCanvasTkAgg(self.figure, master=frame)
        self.plot_canvas.get_tk_widget().grid(row=1, column=0, sticky="nsew", pady=(10, 0))
        self.redraw_plots(force=True)

    def refresh_ports(self) -> None:
        ports = sorted(port.device for port in list_ports.comports())
        current = self.port_var.get()
        if self.port_combo is not None:
            self.port_combo["values"] = ports

        if current in ports:
            self.port_var.set(current)
        elif ports:
            self.port_var.set(ports[0])
        else:
            self.port_var.set("")

    def connect(self) -> None:
        if self.reader is not None:
            return

        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror(APP_TITLE, "Select a serial port before connecting.")
            return

        try:
            baudrate = int(self.baudrate_var.get())
        except ValueError:
            messagebox.showerror(APP_TITLE, "Baud rate must be an integer.")
            return

        self.clear_runtime_state()
        self.connection_var.set(f"Connecting to {port}...")
        self.connected_port = port
        self.reader = SerialReader(port, baudrate, self.reader_queue)
        self.reader.start()
        self._update_connection_buttons()

    def disconnect(self) -> None:
        reader = self.reader
        if reader is None:
            self.connection_var.set("Disconnected")
            self._update_connection_buttons()
            return

        self._reset_pose_test_state(clear_results=True)
        if self._phase0_auto_after_id is not None:
            try:
                self.root.after_cancel(self._phase0_auto_after_id)
            except (tk.TclError, ValueError):
                pass
            self._phase0_auto_after_id = None
        if self._imu_drift_test_after_id is not None:
            try:
                self.root.after_cancel(self._imu_drift_test_after_id)
            except (tk.TclError, ValueError):
                pass
            self._imu_drift_test_after_id = None
        if self._glove_calibration_after_id is not None:
            try:
                self.root.after_cancel(self._glove_calibration_after_id)
            except (tk.TclError, ValueError):
                pass
            self._glove_calibration_after_id = None
        self._glove_calibration_capture_in_progress = False
        self._phase0_auto_in_progress = False
        if self.capture_writer is not None:
            self.stop_capture()
        self._imu_drift_test_in_progress = False
        self._imu_drift_test_auto_stop = False
        self._imu_drift_test_started_monotonic = None
        self._imu_drift_test_duration_s = 0.0

        self.connection_var.set(f"Disconnecting from {self.connected_port or reader.port}...")
        reader.stop()
        self.reader = None
        self.connected_port = ""
        self._update_connection_buttons()

    def start_capture(
        self,
        suggested_filename: str = "drift_capture.csv",
        title: str = "Save drift capture CSV",
        capture_mode: str = "generic",
        explicit_path: str | None = None,
    ) -> bool:
        if self.capture_writer is not None:
            return True

        if explicit_path:
            capture_path = explicit_path
        else:
            capture_path = filedialog.asksaveasfilename(
                title=title,
                defaultextension=".csv",
                filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
                initialfile=suggested_filename,
            )
        if not capture_path:
            return False

        try:
            self.capture_file = open(capture_path, "w", newline="", encoding="utf-8")
        except OSError as exc:
            messagebox.showerror(APP_TITLE, f"Unable to create capture file:\n{exc}")
            return False

        try:
            self.capture_jsonl_file = open(self._glove_jsonl_path(capture_path), "w", encoding="utf-8")
        except OSError as exc:
            self.capture_file.close()
            self.capture_file = None
            messagebox.showerror(APP_TITLE, f"Unable to create glove JSONL sidecar:\n{exc}")
            return False

        self.capture_writer = csv.DictWriter(
            self.capture_file,
            fieldnames=[
                "time_s",
                "protocol",
                "sequence",
                "node_id",
                "status_hex",
                "status_bits",
                "quat_w",
                "quat_x",
                "quat_y",
                "quat_z",
                "yaw_deg",
                "yaw_unwrapped_deg",
                "pitch_deg",
                "roll_deg",
                "imu0_accel_x_mg",
                "imu0_accel_y_mg",
                "imu0_accel_z_mg",
                "imu0_gyro_x_dps",
                "imu0_gyro_y_dps",
                "imu0_gyro_z_dps",
                "imu1_accel_x_mg",
                "imu1_accel_y_mg",
                "imu1_accel_z_mg",
                "imu1_gyro_x_dps",
                "imu1_gyro_y_dps",
                "imu1_gyro_z_dps",
                "guided_test_active",
                "guided_test_step",
                "guided_test_step_label",
                "guided_test_step_status",
                "guided_test_elapsed_s",
                "guided_test_target_node",
                "guided_test_target_label",
                "guided_test_error_deg",
                "guided_test_reference_node",
                "guided_test_reference_label",
                "palm_ref_quat_w",
                "palm_ref_quat_x",
                "palm_ref_quat_y",
                "palm_ref_quat_z",
                "palm_live_quat_w",
                "palm_live_quat_x",
                "palm_live_quat_y",
                "palm_live_quat_z",
                "relative_to_palm_quat_w",
                "relative_to_palm_quat_x",
                "relative_to_palm_quat_y",
                "relative_to_palm_quat_z",
                "relative_to_palm_yaw_deg",
                "relative_to_palm_pitch_deg",
                "relative_to_palm_roll_deg",
                *self._glove_capture_fieldnames(),
                "raw_frame_hex",
            ],
        )
        self.capture_writer.writeheader()
        self.capture_path = capture_path
        self.capture_mode = capture_mode
        self.capture_start_monotonic = None
        self.capture_first_frame_kind = None
        self.capture_protocol_change_count = 0
        self.capture_samples_written = 0
        self.capture_status_samples = 0
        self.capture_status_good_samples = 0
        self.capture_status_issue_flags.clear()
        self.capture_last_summary = ""
        for state in self.fused_nodes.values():
            state.capture_samples.clear()
            state.capture_status_issue_flags.clear()
            state.capture_status_samples = 0
            state.capture_status_good_samples = 0
        self.capture_state_var.set("Recording")
        self.capture_file_var.set(capture_path)
        self.summary_var.set("Capture started. Hold still for 30-60 s or perform a quick motion-recovery test.")
        self.pose_test_recording_var.set("Capture: recording")
        self._update_connection_buttons()
        self._pose_test_dirty = True
        self._refresh_pose_test_ui(force=True)
        return True

    def stop_capture(self) -> None:
        writer = self.capture_writer
        capture_file = self.capture_file
        capture_jsonl_file = self.capture_jsonl_file
        if writer is None or capture_file is None:
            self.capture_state_var.set("Not recording")
            self._update_connection_buttons()
            return

        summary = self._build_capture_summary(final=True, node_id=self.selected_node_id)
        tuning = self._build_tuning_hint(final=True, node_id=self.selected_node_id)
        summary_path = ""
        saved_capture_path = self.capture_path
        capture_mode = self.capture_mode

        try:
            capture_file.flush()
            summary_path = self._write_capture_summary_file(summary, tuning)
        finally:
            capture_file.close()
            if capture_jsonl_file is not None:
                capture_jsonl_file.close()
            self.capture_writer = None
            self.capture_file = None
            self.capture_jsonl_file = None

        self.capture_state_var.set(
            f"Capture saved ({self.capture_samples_written} frames)"
            + (f" + summary {summary_path}" if summary_path else "")
        )
        self.capture_last_summary = summary
        self.summary_var.set(summary)
        self.tuning_var.set(tuning)
        self.capture_path = ""
        self.capture_mode = "generic"
        self.capture_file_var.set("-")
        self.pose_test_recording_var.set("Capture: idle")
        self._update_connection_buttons()
        self._pose_test_dirty = True
        self._refresh_pose_test_ui(force=True)
        if capture_mode == "isolation" and saved_capture_path:
            self._update_isolation_results([saved_capture_path], auto_open=False)
        if capture_mode == "imu_drift_test":
            self._on_imu_drift_capture_stopped()

    def _resolve_imu_drift_duration_s(self) -> float:
        selection = self.imu_drift_duration_var.get().strip()
        if selection and selection.lower() != "custom":
            try:
                duration_s = float(selection)
            except ValueError as exc:
                raise ValueError(f"Invalid preset duration '{selection}'.") from exc
        else:
            custom = self.imu_drift_custom_duration_var.get().strip()
            if not custom:
                raise ValueError("Enter a custom duration in seconds.")
            try:
                duration_s = float(custom)
            except ValueError as exc:
                raise ValueError("Custom duration must be a number.") from exc
        if duration_s <= 0.0:
            raise ValueError("Duration must be greater than 0 seconds.")
        return duration_s

    def _default_imu_drift_filename(self, duration_s: float) -> str:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        duration_label = f"{duration_s:.0f}" if duration_s.is_integer() else f"{duration_s:.1f}"
        return f"imu_drift_{duration_label}s_{timestamp}.csv"

    def start_imu_drift_test(self) -> None:
        if self.reader is None:
            messagebox.showerror(APP_TITLE, "Connect to the palm board before starting an IMU drift test.")
            return
        if self.capture_writer is not None:
            messagebox.showerror(APP_TITLE, "Stop the current capture before starting an IMU drift test.")
            return
        if self._imu_drift_test_in_progress:
            return
        try:
            duration_s = self._resolve_imu_drift_duration_s()
        except ValueError as exc:
            messagebox.showerror(APP_TITLE, str(exc))
            return
        if not self.start_capture(
            suggested_filename=self._default_imu_drift_filename(duration_s),
            title="Save IMU drift test CSV",
            capture_mode="imu_drift_test",
        ):
            return
        if self._imu_drift_test_after_id is not None:
            try:
                self.root.after_cancel(self._imu_drift_test_after_id)
            except (tk.TclError, ValueError):
                pass
            self._imu_drift_test_after_id = None
        self._imu_drift_test_in_progress = True
        self._imu_drift_test_duration_s = duration_s
        self._imu_drift_test_started_monotonic = time.monotonic()
        self._imu_drift_test_auto_stop = False
        self.imu_drift_status_var.set(
            f"Running IMU drift test for {duration_s:.1f} s. Keep the glove as still as possible."
        )
        self.imu_drift_result_var.set("Running... ranking will appear when the test finishes.")
        self._imu_drift_test_after_id = self.root.after(
            int(duration_s * 1000),
            self._finish_imu_drift_test,
        )
        self._update_connection_buttons()

    def _finish_imu_drift_test(self) -> None:
        self._imu_drift_test_after_id = None
        if not self._imu_drift_test_in_progress:
            return
        self._imu_drift_test_auto_stop = True
        if self.capture_writer is not None and self.capture_mode == "imu_drift_test":
            self.stop_capture()
            return
        self._on_imu_drift_capture_stopped()

    def _collect_imu_drift_rankings(self) -> list[ImuDriftRanking]:
        rankings: list[ImuDriftRanking] = []
        for node_id in sorted(self.fused_nodes):
            state = self.fused_nodes[node_id]
            samples = list(state.capture_samples)
            if len(samples) < 2:
                continue
            duration_s = samples[-1].time_s - samples[0].time_s
            if duration_s <= 0.0:
                continue
            yaw_rate = self._axis_rate("yaw_unwrapped_deg", samples) or 0.0
            pitch_rate = self._axis_rate("pitch_deg", samples) or 0.0
            roll_rate = self._axis_rate("roll_deg", samples) or 0.0
            rates = {
                "yaw": yaw_rate,
                "pitch": pitch_rate,
                "roll": roll_rate,
            }
            dominant_axis = max(rates, key=lambda axis: abs(rates[axis]))
            rankings.append(
                ImuDriftRanking(
                    node_id=node_id,
                    label=self._node_label(node_id),
                    duration_s=duration_s,
                    dominant_axis=dominant_axis,
                    dominant_rate=rates[dominant_axis],
                    yaw_rate=yaw_rate,
                    pitch_rate=pitch_rate,
                    roll_rate=roll_rate,
                    score=abs(rates[dominant_axis]),
                )
            )
        rankings.sort(key=lambda item: (-item.score, item.node_id))
        return rankings

    def _format_imu_drift_rankings(self, rankings: list[ImuDriftRanking]) -> str:
        if not rankings:
            return "No ranking available. Capture at least two fused samples per node."
        worst = rankings[0]
        lines = [
            (
                f"Worst: {worst.label} {worst.dominant_axis} {worst.dominant_rate:+.2f} deg/10s "
                f"(window {worst.duration_s:.1f}s)"
            ),
            "Ranking:",
        ]
        for index, item in enumerate(rankings, start=1):
            lines.append(
                (
                    f"{index}. {item.label}: {item.dominant_axis} {item.dominant_rate:+.2f} deg/10s "
                    f"[yaw {item.yaw_rate:+.2f}, pitch {item.pitch_rate:+.2f}, roll {item.roll_rate:+.2f}]"
                )
            )
        return "\n".join(lines)

    def _on_imu_drift_capture_stopped(self) -> None:
        if self._imu_drift_test_after_id is not None:
            try:
                self.root.after_cancel(self._imu_drift_test_after_id)
            except (tk.TclError, ValueError):
                pass
            self._imu_drift_test_after_id = None
        elapsed_s = 0.0
        if self._imu_drift_test_started_monotonic is not None:
            elapsed_s = max(0.0, time.monotonic() - self._imu_drift_test_started_monotonic)
        rankings = self._collect_imu_drift_rankings()
        self.imu_drift_result_var.set(self._format_imu_drift_rankings(rankings))
        if rankings:
            if self._imu_drift_test_auto_stop:
                self.imu_drift_status_var.set(
                    f"Completed IMU drift test ({self._imu_drift_test_duration_s:.1f} s target, {elapsed_s:.1f} s elapsed)."
                )
            else:
                self.imu_drift_status_var.set(
                    f"IMU drift test stopped early after {elapsed_s:.1f} s."
                )
        else:
            self.imu_drift_status_var.set(
                "IMU drift test ended, but there were not enough fused samples to rank drift."
            )
        self._imu_drift_test_auto_stop = False
        self._imu_drift_test_in_progress = False
        self._imu_drift_test_started_monotonic = None
        self._imu_drift_test_duration_s = 0.0
        self._update_connection_buttons()

    def _write_capture_summary_file(self, summary: str, tuning: str) -> str:
        if not self.capture_path:
            return ""

        if self.capture_path.lower().endswith(".csv"):
            summary_path = self.capture_path[:-4] + "_summary.txt"
        else:
            summary_path = self.capture_path + "_summary.txt"

        try:
            with open(summary_path, "w", encoding="utf-8") as handle:
                handle.write("STM32 USB CDC Drift Capture Summary\n\n")
                handle.write(summary + "\n\n")
                handle.write("Tuning Hint\n")
                handle.write(tuning + "\n")
        except OSError:
            return ""

        return summary_path

    def _isolation_capture_filename(self) -> str:
        return f"round{self.isolation_round_var.get()}_{self.isolation_finger_var.get()}_only.csv"

    def start_isolation_capture(self) -> None:
        if self.capture_writer is not None:
            messagebox.showerror(APP_TITLE, "Stop the current capture before starting an isolation capture.")
            return

        state = self._palm_reference_state()
        if state is None or state.last_frame is None or state.last_frame.orientation is None:
            messagebox.showerror(APP_TITLE, "Need palm-local fused data before starting an isolation capture.")
            return

        finger_label = self.isolation_finger_var.get()
        round_label = self.isolation_round_var.get()
        if not self.start_capture(
            suggested_filename=self._isolation_capture_filename(),
            title="Save isolation capture CSV",
            capture_mode="isolation",
        ):
            return

        self.isolation_status_var.set(
            f"Recording round {round_label} {finger_label} isolation capture."
        )
        self.isolation_summary_var.set(
            "Keep the palm as fixed as possible, move only the chosen finger, and stop after one clear curl/extend cycle."
        )

    def analyze_isolation_files(self) -> None:
        paths = filedialog.askopenfilenames(
            title="Select isolation capture CSVs",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not paths:
            return
        self._update_isolation_results(paths, auto_open=True)

    def _update_isolation_results(self, paths: list[str] | tuple[str, ...], auto_open: bool) -> None:
        try:
            results = [analyze_isolation_csv(Path(path)) for path in paths]
        except (OSError, ValueError, csv.Error) as exc:
            messagebox.showerror(APP_TITLE, f"Unable to analyze isolation capture(s):\n{exc}")
            return

        if not results:
            return

        latest = results[-1]
        self.isolation_status_var.set(
            f"{latest.path.name}: dominant node {latest.dominant_node_id if latest.dominant_node_id is not None else '?'} "
            f"(margin {latest.dominant_margin_deg:.1f} deg, palm {latest.palm_motion_deg:.1f} deg)"
        )
        self.isolation_summary_var.set(format_consensus_text(results))
        if auto_open:
            details = "\n\n".join(format_result_text(result) for result in results)
            messagebox.showinfo(APP_TITLE, format_consensus_text(results) + "\n\nDetails\n" + details)

    def start_phase0_autodetect(self) -> None:
        if self._phase0_auto_in_progress:
            return
        if self.reader is None:
            messagebox.showerror(
                APP_TITLE,
                "Connect to the palm board before running Phase 0 Auto-Detect.",
            )
            return
        if self.capture_writer is not None:
            messagebox.showerror(
                APP_TITLE,
                "Stop the current capture before running Phase 0 Auto-Detect.",
            )
            return

        palm_state = self._palm_reference_state()
        if palm_state is None or palm_state.last_frame is None or palm_state.last_frame.orientation is None:
            messagebox.showerror(
                APP_TITLE,
                "No palm fused frame seen yet. Wait a few seconds and try again.",
            )
            return

        fingertip_nodes_seen = sum(
            1
            for node_id, state in self.fused_nodes.items()
            if node_id != 0 and state.last_frame is not None
        )
        if fingertip_nodes_seen == 0:
            if not messagebox.askyesno(
                APP_TITLE,
                "No fingertip fused frames have been seen yet. Continue anyway?",
            ):
                return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"phase0_auto_{timestamp}.csv"
        capture_path = str(Path.cwd() / filename)

        if not self.start_capture(
            suggested_filename=filename,
            title="Phase 0 Auto-Detect Capture",
            capture_mode="phase0_auto",
            explicit_path=capture_path,
        ):
            return

        self._phase0_auto_in_progress = True
        self.phase0_status_var.set(
            f"Recording {PHASE0_AUTO_CAPTURE_S:.1f} s flat-pose capture. Hold everything still..."
        )
        self._update_connection_buttons()
        self._phase0_auto_after_id = self.root.after(
            int(PHASE0_AUTO_CAPTURE_S * 1000),
            self._phase0_finish_autodetect_capture,
        )

    def _phase0_finish_autodetect_capture(self) -> None:
        self._phase0_auto_after_id = None
        capture_path = self.capture_path
        try:
            if self.capture_writer is not None:
                self.stop_capture()
        finally:
            self._phase0_auto_in_progress = False
            self._update_connection_buttons()

        if not capture_path:
            self.phase0_status_var.set("Auto-Detect aborted: no capture path recorded.")
            return

        self._run_phase0_analysis_and_report(Path(capture_path))

    def analyze_phase0_capture(self) -> None:
        if self._phase0_auto_in_progress:
            return
        path_str = filedialog.askopenfilename(
            title="Choose a capture CSV to analyze",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not path_str:
            return
        self._run_phase0_analysis_and_report(Path(path_str))

    def _run_phase0_analysis_and_report(self, csv_path: Path) -> None:
        try:
            report = phase0_autodetect.run_autodetect(csv_path)
        except (OSError, ValueError, csv.Error) as exc:
            messagebox.showerror(APP_TITLE, f"Unable to analyze capture:\n{exc}")
            self.phase0_status_var.set("Analysis failed - see error dialog.")
            return

        out_path = phase0_autodetect.default_report_path(csv_path)
        try:
            phase0_autodetect.write_report_json(report, out_path)
        except OSError as exc:
            messagebox.showerror(APP_TITLE, f"Unable to write report JSON:\n{exc}")
            self.phase0_status_var.set("Report write failed - see error dialog.")
            return

        summary = report.get("summary_text", "(no summary)")
        self.phase0_status_var.set(f"Report written: {out_path}")
        self.phase0_summary_var.set(summary)

        warnings: list[str] = []
        palm_spread = report.get("palm", {}).get("spread_deg", 0.0)
        if palm_spread > phase0_autodetect.SPREAD_WARN_DEG:
            warnings.append(
                f"Palm moved during capture (spread {palm_spread:.2f} deg)."
            )
        for finger in report.get("fingers", []):
            tip_spread = finger.get("tip_spread_deg", 0.0)
            if tip_spread > phase0_autodetect.SPREAD_WARN_DEG:
                warnings.append(
                    f"{finger['label']} (node {finger['node']}) moved during capture "
                    f"(spread {tip_spread:.2f} deg)."
                )

        message_lines = [f"Report: {out_path}", "", summary]
        if warnings:
            message_lines.extend(["", "Warnings:", *warnings])
        if not report.get("fingers"):
            message_lines.extend([
                "",
                "No fingertip rows were found in the CSV. Check that fingertip "
                "boards were streaming fused frames during the capture.",
            ])
        messagebox.showinfo(APP_TITLE, "\n".join(message_lines))

    def _glove_calibration_path(self) -> Path:
        return Path.cwd() / GLOVE_CALIBRATION_FILENAME

    def _load_glove_calibration(self) -> None:
        path = self._glove_calibration_path()
        if not path.is_file():
            self.glove_calibration = None
            self.glove_calibration_var.set(f"Calibration: not loaded ({path.name} not found)")
            self._glove_ui_dirty = True
            return
        try:
            self.glove_calibration = glove_pipeline.load_calibration(path)
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
            self.glove_calibration = None
            self.glove_calibration_var.set(f"Calibration load failed: {exc}")
            self.glove_status_var.set("Calibration file exists but could not be parsed.")
            self._glove_ui_dirty = True
            return

        self.glove_calibration_var.set(
            f"Calibration: loaded {path.name} (id {self.glove_calibration.calibration_id})"
        )
        self.glove_status_var.set("Calibration loaded. Live flex/swing/twist now use host-side neutral offsets.")
        self._glove_ui_dirty = True

    def _save_glove_calibration(self) -> None:
        if self.glove_calibration is None:
            return
        path = self._glove_calibration_path()
        glove_pipeline.save_calibration(self.glove_calibration, path)
        self.glove_calibration_var.set(
            f"Calibration: loaded {path.name} (id {self.glove_calibration.calibration_id})"
        )

    def _send_usb_command(self, payload: bytes, success_message: str) -> None:
        reader = self.reader
        handle = None if reader is None else reader.serial_handle
        if reader is None or handle is None or not handle.is_open:
            messagebox.showerror(APP_TITLE, "Connect to the palm USB CDC port before sending a command.")
            return
        try:
            handle.write(payload)
            handle.flush()
        except SerialException as exc:
            messagebox.showerror(APP_TITLE, f"Unable to send command:\n{exc}")
            return
        self.glove_status_var.set(success_message)

    def send_zero_all(self) -> None:
        self._send_usb_command(bytes((0xC0, 0x01)), "Sent Zero All {0xC0, 0x01}; waiting for palm and ports to latch.")

    def clear_zero_all(self) -> None:
        self._send_usb_command(bytes((0xC0, 0x00)), "Sent Clear Zero {0xC0, 0x00}; raw mapped output resumes.")

    def reset_glove_calibration(self) -> None:
        if self._glove_calibration_after_id is not None:
            try:
                self.root.after_cancel(self._glove_calibration_after_id)
            except (tk.TclError, ValueError):
                pass
            self._glove_calibration_after_id = None
        self._glove_calibration_capture_in_progress = False
        self._glove_calibration_palm_samples.clear()
        for samples in self._glove_calibration_relative_samples.values():
            samples.clear()
        self.glove_calibration = None
        path = self._glove_calibration_path()
        try:
            if path.exists():
                path.unlink()
        except OSError:
            pass
        self.glove_calibration_var.set(f"Calibration: reset ({path.name} cleared)")
        self.glove_status_var.set("Host-side glove calibration cleared. Returning to raw palm-relative measurements.")
        self._refresh_glove_snapshot()
        self._glove_ui_dirty = True
        self._update_connection_buttons()

    def start_glove_calibration_capture(self) -> None:
        if self.reader is None:
            messagebox.showerror(APP_TITLE, "Connect to the palm board before running Calibrate Flat.")
            return
        if self._glove_calibration_capture_in_progress:
            return
        palm_state = self._palm_reference_state()
        if palm_state is None or palm_state.last_frame is None or palm_state.last_frame.orientation is None:
            messagebox.showerror(APP_TITLE, "Need a live palm fused quaternion before starting Calibrate Flat.")
            return
        self._glove_calibration_capture_in_progress = True
        self._glove_calibration_palm_samples.clear()
        for samples in self._glove_calibration_relative_samples.values():
            samples.clear()
        self.glove_status_var.set(
            f"Recording {GLOVE_CALIBRATION_CAPTURE_S:.1f} s flat pose for host-side calibration. Hold the glove neutral and still."
        )
        self._glove_ui_dirty = True
        self._update_connection_buttons()
        self._glove_calibration_after_id = self.root.after(
            int(GLOVE_CALIBRATION_CAPTURE_S * 1000),
            self._finish_glove_calibration_capture,
        )

    def _finish_glove_calibration_capture(self) -> None:
        self._glove_calibration_after_id = None
        self._glove_calibration_capture_in_progress = False
        try:
            self.glove_calibration = glove_pipeline.build_calibration(
                self._glove_calibration_palm_samples,
                self._glove_calibration_relative_samples,
            )
            self._save_glove_calibration()
        except (OSError, ValueError) as exc:
            self.glove_status_var.set(f"Calibration failed: {exc}")
            self._glove_ui_dirty = True
            self._update_connection_buttons()
            return
        finally:
            self._glove_calibration_palm_samples.clear()
            for samples in self._glove_calibration_relative_samples.values():
                samples.clear()

        self.glove_status_var.set(
            f"Calibration saved. Neutral pose offsets captured for {len(self.glove_calibration.fingers)} finger(s)."
        )
        self._refresh_glove_snapshot()
        self._glove_ui_dirty = True
        self._update_connection_buttons()

    def _collect_glove_calibration_sample(self, frame: ParsedFrame) -> None:
        if not self._glove_calibration_capture_in_progress or frame.kind != "fused" or frame.orientation is None:
            return
        if frame.node_id == 0:
            if (frame.status & 0x04) != 0:
                self._glove_calibration_palm_samples.append(
                    glove_pipeline.quaternion_normalize(frame.orientation.quaternion)
                )
            return

        node_id = frame.node_id or 0
        if node_id not in self._glove_calibration_relative_samples:
            return
        palm_state = self._palm_reference_state()
        node_state = self.fused_nodes.get(node_id)
        if (
            palm_state is None
            or palm_state.last_frame is None
            or palm_state.last_frame.orientation is None
            or node_state is None
            or node_state.last_frame is None
            or node_state.last_frame.orientation is None
        ):
            return
        if (palm_state.last_frame.status & 0x04) == 0 or (node_state.last_frame.status & 0x04) == 0:
            return
        palm_quat = glove_pipeline.quaternion_normalize(palm_state.last_frame.orientation.quaternion)
        tip_quat = glove_pipeline.quaternion_normalize(node_state.last_frame.orientation.quaternion)
        rel_quat = glove_pipeline.finger_rel(palm_quat, tip_quat)
        self._glove_calibration_relative_samples[node_id].append(rel_quat)

    def _refresh_glove_snapshot(self) -> None:
        palm_state = self._palm_reference_state()
        if palm_state is None or palm_state.last_frame is None or palm_state.last_frame.orientation is None:
            self.glove_snapshot = None
            self._glove_ui_dirty = True
            return

        finger_quats: dict[int, glove_pipeline.Quat] = {}
        finger_status: dict[int, int] = {}
        for spec in glove_pipeline.DEFAULT_FINGER_SPECS:
            state = self.fused_nodes.get(spec.node_id)
            if state is None or state.last_frame is None or state.last_frame.orientation is None:
                continue
            finger_quats[spec.node_id] = glove_pipeline.quaternion_normalize(state.last_frame.orientation.quaternion)
            finger_status[spec.node_id] = state.last_frame.status

        self.glove_snapshot = glove_pipeline.compute_snapshot(
            glove_pipeline.quaternion_normalize(palm_state.last_frame.orientation.quaternion),
            finger_quats,
            finger_status_by_node=finger_status,
            calibration=self.glove_calibration,
        )
        self._glove_ui_dirty = True

    def _refresh_glove_ui(self, force: bool = False) -> None:
        if not force and not self._glove_ui_dirty:
            return

        snapshot = self.glove_snapshot
        if snapshot is None:
            for variable in self.glove_palm_ypr_vars.values():
                variable.set("-")
            for vars_for_finger in self.glove_finger_vars.values():
                vars_for_finger["present"].set("no")
                vars_for_finger["valid"].set("-")
                vars_for_finger["calibrated"].set("no")
                vars_for_finger["flex"].set("-")
                vars_for_finger["swing"].set("-")
                vars_for_finger["twist"].set("-")
            self._glove_ui_dirty = False
            return

        yaw_deg, pitch_deg, roll_deg = snapshot.palm_ypr_deg
        self.glove_palm_ypr_vars["yaw"].set(f"{yaw_deg:+.2f}")
        self.glove_palm_ypr_vars["pitch"].set(f"{pitch_deg:+.2f}")
        self.glove_palm_ypr_vars["roll"].set(f"{roll_deg:+.2f}")
        for finger in snapshot.fingers:
            vars_for_finger = self.glove_finger_vars[finger.node_id]
            vars_for_finger["present"].set("yes" if finger.present else "no")
            vars_for_finger["valid"].set("yes" if finger.valid else "no")
            vars_for_finger["calibrated"].set("yes" if finger.calibrated else "no")
            vars_for_finger["flex"].set("-" if not finger.present else f"{finger.flex_deg:+.1f}")
            vars_for_finger["swing"].set("-" if not finger.present else f"{finger.swing_deg:+.1f}")
            vars_for_finger["twist"].set("-" if not finger.present else f"{finger.twist_deg:+.1f}")
        self._glove_ui_dirty = False

    def _glove_capture_fieldnames(self) -> list[str]:
        fieldnames = [
            "glove_calibrated",
            "glove_calibration_id",
            "glove_palm_yaw_deg",
            "glove_palm_pitch_deg",
            "glove_palm_roll_deg",
        ]
        for spec in glove_pipeline.DEFAULT_FINGER_SPECS:
            prefix = spec.name
            fieldnames.extend(
                [
                    f"{prefix}_present",
                    f"{prefix}_valid",
                    f"{prefix}_calibrated",
                    f"{prefix}_flex_deg",
                    f"{prefix}_swing_deg",
                    f"{prefix}_twist_deg",
                ]
            )
        return fieldnames

    def _glove_jsonl_path(self, capture_path: str) -> str:
        if capture_path.lower().endswith(".csv"):
            return capture_path[:-4] + GLOVE_CAPTURE_JSONL_SUFFIX
        return capture_path + GLOVE_CAPTURE_JSONL_SUFFIX

    def _apply_glove_capture_fields(self, row: dict[str, object]) -> None:
        snapshot = self.glove_snapshot
        row["glove_calibrated"] = "0"
        row["glove_calibration_id"] = ""
        row["glove_palm_yaw_deg"] = ""
        row["glove_palm_pitch_deg"] = ""
        row["glove_palm_roll_deg"] = ""
        for spec in glove_pipeline.DEFAULT_FINGER_SPECS:
            prefix = spec.name
            row[f"{prefix}_present"] = ""
            row[f"{prefix}_valid"] = ""
            row[f"{prefix}_calibrated"] = ""
            row[f"{prefix}_flex_deg"] = ""
            row[f"{prefix}_swing_deg"] = ""
            row[f"{prefix}_twist_deg"] = ""

        if snapshot is None:
            return

        row["glove_calibrated"] = "1" if snapshot.calibrated else "0"
        row["glove_calibration_id"] = "" if snapshot.calibration_id is None else snapshot.calibration_id
        yaw_deg, pitch_deg, roll_deg = snapshot.palm_ypr_deg
        row["glove_palm_yaw_deg"] = f"{yaw_deg:.3f}"
        row["glove_palm_pitch_deg"] = f"{pitch_deg:.3f}"
        row["glove_palm_roll_deg"] = f"{roll_deg:.3f}"
        for finger in snapshot.fingers:
            prefix = finger.name
            row[f"{prefix}_present"] = "1" if finger.present else "0"
            row[f"{prefix}_valid"] = "1" if finger.valid else "0"
            row[f"{prefix}_calibrated"] = "1" if finger.calibrated else "0"
            row[f"{prefix}_flex_deg"] = f"{finger.flex_deg:.3f}" if finger.present else ""
            row[f"{prefix}_swing_deg"] = f"{finger.swing_deg:.3f}" if finger.present else ""
            row[f"{prefix}_twist_deg"] = f"{finger.twist_deg:.3f}" if finger.present else ""

    def _write_glove_snapshot_jsonl(self, capture_time_s: float, frame: ParsedFrame) -> None:
        if (
            self.capture_jsonl_file is None
            or frame.kind != "fused"
            or frame.node_id != 0
            or self.glove_snapshot is None
        ):
            return

        snapshot = self.glove_snapshot
        record = {
            "schema_version": "glove-snapshot-1.0",
            "time_s": round(capture_time_s, 3),
            "calibrated": snapshot.calibrated,
            "calibration_id": snapshot.calibration_id,
            "palm": {
                "quat": list(snapshot.palm_quat),
                "ypr_deg": list(snapshot.palm_ypr_deg),
            },
            "fingers": [
                {
                    "name": finger.name,
                    "node_id": finger.node_id,
                    "present": finger.present,
                    "valid": finger.valid,
                    "calibrated": finger.calibrated,
                    "status": finger.status,
                    "quat_rel": list(finger.quat_rel),
                    "quat_rel_calibrated": list(finger.quat_rel_calibrated),
                    "flex_deg": finger.flex_deg,
                    "swing_deg": finger.swing_deg,
                    "twist_deg": finger.twist_deg,
                }
                for finger in snapshot.fingers
            ],
        }
        self.capture_jsonl_file.write(json.dumps(record) + "\n")

    def clear_runtime_state(self) -> None:
        self.last_frame = None
        self.last_stats = ParserStats()
        self.fused_nodes.clear()
        self.pose_test_baselines.clear()
        self.glove_snapshot = None
        self._glove_ui_dirty = True
        self._glove_calibration_palm_samples.clear()
        for samples in self._glove_calibration_relative_samples.values():
            samples.clear()
        self.selected_node_id = 0
        self.selected_node_var.set("palm-local (0)")
        self.selected_node_label_var.set("palm-local (0)")
        self.packet_var.set("0")
        self.raw_packet_var.set("0")
        self.fused_packet_var.set("0")
        self.checksum_var.set("0")
        self.gap_var.set("0")
        self.discarded_var.set("0")
        self.protocol_var.set("-")
        self.sequence_var.set("-")
        self.node_id_var.set("-")
        self.status_hex_var.set("0x00")
        self.status_bits_var.set("-")
        self.delta_var.set("Waiting for frames")
        self.validity_var.set("-")
        self.summary_var.set("No drift summary yet.")
        self.tuning_var.set("Capture a still test to get tuning advice.")
        self.isolation_status_var.set("No isolation capture yet.")
        self.isolation_summary_var.set(
            "Use two rounds of index/middle/ring single-finger captures to confirm which node dominates."
        )
        self.glove_status_var.set(
            "Phase 1 host math idle. Connect the palm and wait for fused fingertip frames."
        )
        if self._imu_drift_test_after_id is not None:
            try:
                self.root.after_cancel(self._imu_drift_test_after_id)
            except (tk.TclError, ValueError):
                pass
            self._imu_drift_test_after_id = None
        self.imu_drift_status_var.set("Idle. Choose a duration and run a drift test to rank nodes by drift.")
        self.imu_drift_result_var.set("No IMU drift test results yet.")
        self._imu_drift_test_in_progress = False
        self._imu_drift_test_auto_stop = False
        self._imu_drift_test_started_monotonic = None
        self._imu_drift_test_duration_s = 0.0
        self.clear_plot_history()
        self._set_raw_frame_text("No valid frame received yet.")
        self._refresh_node_controls()
        self._reset_pose_test_state(clear_results=True)
        for values in self.imu_vars.values():
            for variable in values.values():
                variable.set("-")
        for variable in self.quat_vars.values():
            variable.set("-")
        for variable in self.euler_vars.values():
            variable.set("-")
        for variable in self.drift_vars.values():
            variable.set("-")

    def _reset_pose_test_state(self, clear_results: bool) -> None:
        self.pose_test_active = False
        self.pose_test_node_id = None
        self.pose_test_step_index = 0
        self.pose_test_step_started_monotonic = None
        self.pose_test_started_monotonic = None
        self.pose_test_last_sample_monotonic = None
        self.pose_test_last_error_deg = None
        self.pose_test_started_capture = False
        if clear_results:
            self.pose_test_results = {step.key: "pending" for step in POSE_TEST_STEPS}
        self._pose_test_dirty = True
        self._refresh_pose_test_ui(force=True)

    def _current_pose_test_step(self) -> PoseTestStep | None:
        if not self.pose_test_active:
            return None
        if 0 <= self.pose_test_step_index < len(POSE_TEST_STEPS):
            return POSE_TEST_STEPS[self.pose_test_step_index]
        return None

    def _selected_fused_state(self, node_id: int | None = None) -> FusedNodeState | None:
        target_node = self.selected_node_id if node_id is None else node_id
        state = self.fused_nodes.get(target_node)
        if state is None or state.last_frame is None or state.last_frame.orientation is None:
            return None
        return state

    def _palm_reference_state(self) -> FusedNodeState | None:
        return self._selected_fused_state(0)

    def reset_pose_test_zero(self) -> None:
        state = self._palm_reference_state()
        if state is None or state.last_frame is None or state.last_frame.orientation is None:
            messagebox.showerror(APP_TITLE, "Need palm-local fused data before resetting the pose-test zero reference.")
            return

        baseline = quaternion_normalize(state.last_frame.orientation.quaternion)
        self.pose_test_baselines[state.node_id] = baseline
        if self.pose_test_active and self.pose_test_node_id == state.node_id:
            self.pose_test_step_index = 0
            self.pose_test_step_started_monotonic = None
            self.pose_test_started_monotonic = time.monotonic()
            self.pose_test_results = {step.key: "pending" for step in POSE_TEST_STEPS}
        self.pose_test_status_var.set("Zero baseline captured")
        self.pose_test_instruction_var.set(
            f"Current palm-local pose stored as the guided-test zero reference."
        )
        self.pose_test_summary_var.set(
            "Host-side zero baseline updated from palm-local orientation."
        )
        self._pose_test_dirty = True
        self._refresh_pose_test_ui(force=True)

    def start_pose_test(self) -> None:
        state = self._palm_reference_state()
        if state is None or state.last_frame is None or state.last_frame.orientation is None:
            messagebox.showerror(APP_TITLE, "Need palm-local fused data before starting the guided test.")
            return

        started_capture = False
        if self.capture_writer is None:
            if not self.start_capture("guided_pose_test.csv"):
                return
            started_capture = True

        self.pose_test_started_capture = started_capture
        self.pose_test_node_id = state.node_id
        self.pose_test_baselines[state.node_id] = quaternion_normalize(state.last_frame.orientation.quaternion)
        self.pose_test_active = True
        self.pose_test_step_index = 0
        self.pose_test_step_started_monotonic = None
        self.pose_test_started_monotonic = time.monotonic()
        self.pose_test_last_sample_monotonic = state.last_frame_monotonic
        self.pose_test_last_error_deg = None
        self.pose_test_results = {step.key: "pending" for step in POSE_TEST_STEPS}
        self.pose_test_status_var.set("Running")
        self.pose_test_summary_var.set(
            "Guided test started. Palm-local drives a timed checklist while all nodes are recorded for comparison."
        )
        self._pose_test_dirty = True
        self._refresh_pose_test_ui(force=True)

    def cancel_pose_test(self) -> None:
        target_node_id = self.pose_test_node_id
        should_stop_capture = self.pose_test_started_capture and self.capture_writer is not None
        self._reset_pose_test_state(clear_results=True)
        self.pose_test_status_var.set("Cancelled")
        self.pose_test_instruction_var.set("Guided test cleared.")
        self.pose_test_summary_var.set("Guided test cancelled.")
        if should_stop_capture:
            if target_node_id is not None:
                self._set_selected_node(target_node_id)
            self.stop_capture()
        else:
            self._refresh_pose_test_ui(force=True)

    def _complete_pose_test(self) -> None:
        target_node_id = self.pose_test_node_id
        summary = "Guided pose test complete using palm-local as reference."
        should_stop_capture = self.pose_test_started_capture and self.capture_writer is not None
        self.pose_test_active = False
        self.pose_test_step_started_monotonic = None
        self.pose_test_status_var.set("Completed")
        self.pose_test_instruction_var.set("All guided poses completed. Review the capture to compare each node against palm-local.")
        self.pose_test_summary_var.set(summary)
        self.pose_test_started_capture = False
        self._pose_test_dirty = True
        self._refresh_pose_test_ui(force=True)
        if should_stop_capture:
            if target_node_id is not None:
                self._set_selected_node(target_node_id)
            self.stop_capture()

    def _refresh_pose_test_ui(self, force: bool = False) -> None:
        if not force and not self._pose_test_dirty and not self.pose_test_active:
            return

        target_node_id = self.pose_test_node_id if self.pose_test_node_id is not None else 0
        self.pose_test_node_var.set(f"Reference: {self._node_label(target_node_id)}")
        self.pose_test_recording_var.set(
            f"Capture: {'recording' if self.capture_writer is not None else 'idle'}"
        )

        if not self.pose_test_active:
            for step in POSE_TEST_STEPS:
                self.pose_test_step_vars[step.key].set(self.pose_test_results.get(step.key, "pending"))
            if self.pose_test_status_var.get() == "Not running":
                self.pose_test_instruction_var.set("Press Start Guided Test to begin.")
            self.pose_test_progress_var.set("-")
            self._pose_test_dirty = False
            return

        now = time.monotonic()
        step = self._current_pose_test_step()
        stale = self.pose_test_last_sample_monotonic is None or (now - self.pose_test_last_sample_monotonic) > POSE_STATUS_STALE_S
        if step is None:
            self.pose_test_status_var.set("Completed")
            self.pose_test_instruction_var.set("All guided poses completed.")
            self.pose_test_progress_var.set("-")
        elif stale:
            self.pose_test_status_var.set("Waiting for data")
            self.pose_test_instruction_var.set("Waiting for fresh fused samples from palm-local reference.")
            self.pose_test_progress_var.set("0.0 / 5.0 s")
        else:
            elapsed = 0.0
            if self.pose_test_step_started_monotonic is not None:
                elapsed = max(0.0, now - self.pose_test_step_started_monotonic)
            self.pose_test_status_var.set(f"Running step {self.pose_test_step_index + 1}/{len(POSE_TEST_STEPS)}")
            self.pose_test_instruction_var.set(step.instruction)
            self.pose_test_progress_var.set(f"{elapsed:.1f} / {step.hold_seconds:.1f} s")

        for index, step in enumerate(POSE_TEST_STEPS):
            status = self.pose_test_results.get(step.key, "pending")
            if self.pose_test_active and index == self.pose_test_step_index and status == "pending":
                if self.pose_test_step_started_monotonic is not None:
                    elapsed = max(0.0, time.monotonic() - self.pose_test_step_started_monotonic)
                    status = f"holding {elapsed:.1f}/{step.hold_seconds:.1f}s"
                else:
                    status = "waiting for stable palm"
            self.pose_test_step_vars[step.key].set(status)

        self._pose_test_dirty = False

    def _flush_deferred_ui(self, force: bool = False) -> None:
        now = time.monotonic()
        if force or (self._pending_raw_frame is not None and now - self._last_raw_frame_refresh >= RAW_FRAME_REFRESH_S):
            if self._pending_raw_frame is not None:
                self._set_raw_frame_text(self._format_frame_hex(self._pending_raw_frame))
                self._pending_raw_frame = None
                self._last_raw_frame_refresh = now

        if force or now - self._last_heavy_ui_refresh >= HEAVY_UI_REFRESH_S:
            if self._node_controls_dirty:
                self._refresh_node_controls()
                self._node_controls_dirty = False
            if self._selected_details_dirty:
                self._render_selected_node_details()
                self._selected_details_dirty = False
            self._refresh_glove_ui(force=force)
            self._refresh_pose_test_ui(force=force)
            self._last_heavy_ui_refresh = now

    def _update_pose_test_for_frame(self, frame: ParsedFrame, now_monotonic: float) -> None:
        if (
            not self.pose_test_active
            or frame.kind != "fused"
            or frame.orientation is None
            or self.pose_test_node_id is None
            or frame.node_id != 0
        ):
            return

        step = self._current_pose_test_step()
        baseline = self.pose_test_baselines.get(self.pose_test_node_id)
        if step is None or baseline is None:
            return

        self.pose_test_last_sample_monotonic = now_monotonic
        self.pose_test_last_error_deg = None
        self._pose_test_dirty = True

        healthy = (frame.status & 0x04) != 0 and (frame.status & 0x20) == 0 and (frame.status & 0x40) == 0
        if not healthy:
            self.pose_test_step_started_monotonic = None
            self.pose_test_summary_var.set(
                "Waiting for stable palm-local fusion before counting dwell time."
            )
            return

        if self.pose_test_step_started_monotonic is None:
            self.pose_test_step_started_monotonic = now_monotonic
            return

        if now_monotonic - self.pose_test_step_started_monotonic >= step.hold_seconds:
            self.pose_test_results[step.key] = "pass (timed)"
            self.pose_test_step_index += 1
            self.pose_test_step_started_monotonic = None
            if self.pose_test_step_index >= len(POSE_TEST_STEPS):
                self._complete_pose_test()
            else:
                next_step = self._current_pose_test_step()
                if next_step is not None:
                    self.pose_test_summary_var.set(
                        f"{step.label} timed out successfully. Next: {next_step.label}. All nodes continue recording against palm-local."
                    )

    def process_reader_events(self) -> None:
        status_message: str | None = None
        saw_disconnect = False
        processed_events = 0

        while processed_events < UI_FRAME_BATCH_LIMIT:
            try:
                event = self.reader_queue.get_nowait()
            except queue.Empty:
                break
            processed_events += 1

            if event.kind == "connected":
                status_message = f"Connected to {event.message}"
            elif event.kind == "frame":
                if event.stats is not None:
                    self._apply_stats(event.stats)
                if event.frame is not None:
                    self._apply_frame(event.frame)
            elif event.kind == "stats":
                if event.stats is not None:
                    self._apply_stats(event.stats)
            elif event.kind == "error":
                status_message = f"Serial error: {event.message}"
            elif event.kind == "disconnected":
                saw_disconnect = True

        if status_message is not None:
            self.connection_var.set(status_message)
        if saw_disconnect:
            self.reader = None
            self.connected_port = ""
            if not status_message:
                self.connection_var.set("Disconnected")
            self._update_connection_buttons()

        self._flush_deferred_ui(force=False)
        next_delay_ms = 1 if processed_events >= UI_FRAME_BATCH_LIMIT else UI_POLL_MS
        self.root.after(next_delay_ms, self.process_reader_events)

    def _apply_stats(self, stats: ParserStats) -> None:
        self.last_stats = stats
        self.packet_var.set(str(stats.packets))
        self.raw_packet_var.set(str(stats.raw_packets))
        self.fused_packet_var.set(str(stats.fused_packets))
        self.checksum_var.set(str(stats.checksum_errors))
        self.gap_var.set(str(stats.sequence_gaps))
        self.discarded_var.set(str(stats.bytes_discarded))

    def _apply_frame(self, frame: ParsedFrame) -> None:
        previous = self.last_frame
        self.last_frame = frame
        now_monotonic = time.monotonic()

        self.protocol_var.set(frame.kind.upper())
        self.sequence_var.set("-" if frame.sequence is None else str(frame.sequence))
        self.node_id_var.set("-" if frame.node_id is None else str(frame.node_id))
        self.status_hex_var.set(f"0x{frame.status:02X}")
        self.status_bits_var.set(self._format_status_bits(frame.status))
        self.delta_var.set(self._format_section_delta(previous, frame))

        if frame.kind == "raw" and frame.imu0 is not None and frame.imu1 is not None:
            self._set_imu_values("imu0", frame.imu0.accel_mg, frame.imu0.gyro_dps_x10)
            self._set_imu_values("imu1", frame.imu1.accel_mg, frame.imu1.gyro_dps_x10)
            if not self.fused_nodes:
                self._set_quaternion_values(None)
                self._set_euler_values(None)
            self._selected_details_dirty = True
        elif frame.kind == "fused" and frame.orientation is not None:
            self._clear_imu_values()
            node_state = self._get_or_create_node_state(frame.node_id or 0)
            node_state.packet_count += 1
            node_state.last_frame = frame
            node_state.last_frame_monotonic = now_monotonic
            if node_state.node_id == self.selected_node_id or len(self.fused_nodes) == 1:
                self._set_selected_node(node_state.node_id)
            self._update_angle_metrics(node_state, frame, now_monotonic)
            self._update_pose_test_for_frame(frame, now_monotonic)
            self._collect_glove_calibration_sample(frame)
            self._refresh_glove_snapshot()
            self._node_controls_dirty = True
            self._selected_details_dirty = True

        self._pending_raw_frame = frame
        self._capture_frame(frame, now_monotonic)
        if frame.kind != "fused":
            self._selected_details_dirty = True
        self._update_plot_history(frame)

    def _set_imu_values(self, imu_key: str, accel: tuple[int, int, int], gyro_x10: tuple[int, int, int]) -> None:
        values = self.imu_vars[imu_key]
        values["accel_x"].set(str(accel[0]))
        values["accel_y"].set(str(accel[1]))
        values["accel_z"].set(str(accel[2]))
        values["gyro_x"].set(f"{gyro_x10[0] / 10.0:.1f}")
        values["gyro_y"].set(f"{gyro_x10[1] / 10.0:.1f}")
        values["gyro_z"].set(f"{gyro_x10[2] / 10.0:.1f}")

    def _clear_imu_values(self) -> None:
        for values in self.imu_vars.values():
            for variable in values.values():
                variable.set("-")

    def _set_quaternion_values(self, quaternion: tuple[float, float, float, float] | None) -> None:
        if quaternion is None:
            for variable in self.quat_vars.values():
                variable.set("-")
            return

        for key, value in zip(("w", "x", "y", "z"), quaternion):
            self.quat_vars[key].set(f"{value:.4f}")

    def _set_euler_values(self, angles_deg: tuple[float, float, float] | None) -> None:
        if angles_deg is None:
            for variable in self.euler_vars.values():
                variable.set("-")
            for variable in self.drift_vars.values():
                variable.set("-")
            self.validity_var.set("-")
            return

        yaw_deg, pitch_deg, roll_deg = angles_deg
        self.euler_vars["yaw"].set(f"{yaw_deg:+.2f}")
        self.euler_vars["pitch"].set(f"{pitch_deg:+.2f}")
        self.euler_vars["roll"].set(f"{roll_deg:+.2f}")

    def _get_or_create_node_state(self, node_id: int) -> FusedNodeState:
        state = self.fused_nodes.get(node_id)
        if state is None:
            state = FusedNodeState(node_id=node_id)
            self.fused_nodes[node_id] = state
        return state

    def _describe_host_node(self, node_id: int) -> tuple[str, str]:
        if node_id == 0:
            return "palm-local", "Palm local"

        uart_base = (node_id // 10) * 10
        local_index = node_id - uart_base
        finger_map = {
            20: ("thumb", "Thumb"),
            30: ("index", "Index"),
            40: ("middle", "Middle"),
            50: ("ring", "Ring"),
            60: ("pinky", "Pinky"),
        }
        uart_map = {
            20: "UART2",
            30: "UART3",
            40: "UART4",
            50: "UART5",
            60: "UART6",
            70: "UART7",
        }
        finger_info = finger_map.get(uart_base)
        if finger_info is not None:
            description, family = finger_info
            if local_index == 0:
                return description, family
            return f"{description} local {local_index}", family

        uart_name = uart_map.get(uart_base)
        if uart_name is None:
            return f"external node {node_id}", "External"

        return f"{uart_name} local {local_index}", uart_name

    def _node_label(self, node_id: int) -> str:
        description, _family = self._describe_host_node(node_id)
        return f"{description} ({node_id})"

    def _set_selected_node(self, node_id: int) -> None:
        self.selected_node_id = node_id
        label = self._node_label(node_id)
        if self.selected_node_var.get() != label:
            self.selected_node_var.set(label)
        self.selected_node_label_var.set(label)

    def _on_selected_node_changed(self, *_args: object) -> None:
        value = self.selected_node_var.get()
        if "(" not in value or ")" not in value:
            return
        try:
            node_id = int(value.rsplit("(", 1)[1].split(")", 1)[0])
        except ValueError:
            return
        self.selected_node_id = node_id
        self.selected_node_label_var.set(self._node_label(node_id))
        self._selected_details_dirty = True
        self._pose_test_dirty = True
        self._flush_deferred_ui(force=True)
        self.redraw_plots(force=True)

    def _on_node_tree_select(self, _event: object) -> None:
        if self.node_tree is None:
            return
        selection = self.node_tree.selection()
        if not selection:
            return
        values = self.node_tree.item(selection[0], "values")
        if not values:
            return
        try:
            node_id = int(values[0])
        except (TypeError, ValueError):
            return
        self._set_selected_node(node_id)
        self._selected_details_dirty = True
        self._pose_test_dirty = True
        self._flush_deferred_ui(force=True)
        self.redraw_plots(force=True)

    def _refresh_node_controls(self) -> None:
        labels = [self._node_label(node_id) for node_id in sorted(self.fused_nodes)]
        if self.node_combo is not None:
            self.node_combo["values"] = labels
        if self.selected_node_id not in self.fused_nodes and self.fused_nodes:
            self._set_selected_node(min(self.fused_nodes))
        elif not self.fused_nodes:
            self.selected_node_label_var.set("palm-local (0)")

        if self.node_tree is None:
            return

        for item in self.node_tree.get_children():
            self.node_tree.delete(item)

        now = time.monotonic()
        for node_id in sorted(self.fused_nodes):
            state = self.fused_nodes[node_id]
            frame = state.last_frame
            description, family = self._describe_host_node(node_id)
            if frame is None or frame.orientation is None:
                yaw_text = "-"
                pitch_text = "-"
                roll_text = "-"
                status_text = "-"
                age_ms = "-"
            else:
                latest = state.angle_history[-1] if state.angle_history else None
                yaw_text = "-" if latest is None else f"{latest.yaw_deg:+.2f}"
                pitch_text = "-" if latest is None else f"{latest.pitch_deg:+.2f}"
                roll_text = "-" if latest is None else f"{latest.roll_deg:+.2f}"
                status_text = self._format_status_bits(frame.status)
                age_ms = "-" if state.last_frame_monotonic is None else f"{(now - state.last_frame_monotonic) * 1000.0:.0f}"

            self.node_tree.insert(
                "",
                tk.END,
                values=(
                    node_id,
                    description if node_id == 0 else (family if node_id % 10 == 0 else f"{family} #{node_id % 10}"),
                    state.packet_count,
                    age_ms,
                    yaw_text,
                    pitch_text,
                    roll_text,
                    status_text,
                ),
            )

    def _render_selected_node_details(self) -> None:
        state = self.fused_nodes.get(self.selected_node_id)
        if state is None or state.last_frame is None or state.last_frame.orientation is None:
            self._set_quaternion_values(None)
            self._set_euler_values(None)
            self.validity_var.set("-")
            self.summary_var.set("No fused data for the selected node yet.")
            self.tuning_var.set("Select a node receiving fused frames to see drift guidance.")
            return

        frame = state.last_frame
        latest = state.angle_history[-1] if state.angle_history else None
        self._set_quaternion_values(frame.orientation.quaternion)
        if latest is None:
            self._set_euler_values(None)
            self.validity_var.set("-")
            return

        self._set_euler_values((latest.yaw_deg, latest.pitch_deg, latest.roll_deg))
        self.drift_vars["yaw"].set(format_rate_per_10s(self._axis_rate("yaw_unwrapped_deg", state.angle_history)))
        self.drift_vars["pitch"].set(format_rate_per_10s(self._axis_rate("pitch_deg", state.angle_history)))
        self.drift_vars["roll"].set(format_rate_per_10s(self._axis_rate("roll_deg", state.angle_history)))
        self.validity_var.set(self._validity_text(state.angle_history))
        self.summary_var.set(self._build_capture_summary(final=False, node_id=self.selected_node_id))
        self.tuning_var.set(self._build_tuning_hint(final=False, node_id=self.selected_node_id))

    def _update_angle_metrics(self, node_state: FusedNodeState, frame: ParsedFrame, now_monotonic: float) -> None:
        if frame.orientation is None:
            self._set_euler_values(None)
            return

        if node_state.session_start_monotonic is None:
            node_state.session_start_monotonic = now_monotonic

        relative_time_s = now_monotonic - node_state.session_start_monotonic
        yaw_deg, pitch_deg, roll_deg = quaternion_to_euler_deg(frame.orientation.quaternion)
        yaw_unwrapped = unwrap_angle_deg(node_state.last_yaw_unwrapped, yaw_deg)
        node_state.last_yaw_unwrapped = yaw_unwrapped

        sample = AngleSample(
            time_s=relative_time_s,
            yaw_deg=yaw_deg,
            yaw_unwrapped_deg=yaw_unwrapped,
            pitch_deg=pitch_deg,
            roll_deg=roll_deg,
            status=frame.status,
        )
        node_state.angle_history.append(sample)
        self._trim_angle_history(node_state)

    def _trim_angle_history(self, node_state: FusedNodeState) -> None:
        while len(node_state.angle_history) >= 2 and (
            node_state.angle_history[-1].time_s - node_state.angle_history[0].time_s > max(60.0, DRIFT_WINDOW_S * 3.0)
        ):
            node_state.angle_history.popleft()

    def _axis_rate(self, attr_name: str, samples: list[AngleSample] | deque[AngleSample] | None = None) -> float | None:
        if samples is None:
            node_state = self.fused_nodes.get(self.selected_node_id)
            source = list(node_state.angle_history) if node_state is not None else []
        else:
            source = list(samples)
        if len(source) < 2:
            return None

        end = source[-1]
        start = end
        for candidate in reversed(source):
            if end.time_s - candidate.time_s >= DRIFT_WINDOW_S:
                start = candidate
                break
            start = candidate

        dt = end.time_s - start.time_s
        if dt <= 0.0:
            return None
        return (getattr(end, attr_name) - getattr(start, attr_name)) / dt * 10.0

    def _validity_text(self, samples: list[AngleSample] | deque[AngleSample]) -> str:
        sample_list = list(samples)
        if not sample_list:
            return "-"

        valid_count = sum(1 for sample in sample_list if (sample.status & 0x03) == 0x03)
        ratio = valid_count / len(sample_list)
        if valid_count == len(sample_list):
            base = "Both IMUs valid throughout"
        else:
            base = f"Both IMUs valid {ratio * 100.0:.0f}%"

        flags = []
        if any(sample.status & 0x20 for sample in sample_list):
            flags.append("disagreement seen")
        if any(sample.status & 0x40 for sample in sample_list):
            flags.append("warmup seen")
        return base if not flags else base + " (" + ", ".join(flags) + ")"

    def _capture_frame(self, frame: ParsedFrame, now_monotonic: float) -> None:
        if self.capture_writer is None or self.capture_file is None:
            return

        if self.capture_start_monotonic is None:
            self.capture_start_monotonic = now_monotonic
        capture_time_s = now_monotonic - self.capture_start_monotonic

        if self.capture_first_frame_kind is None:
            self.capture_first_frame_kind = frame.kind
        elif frame.kind != self.capture_first_frame_kind:
            self.capture_protocol_change_count += 1
            self.capture_first_frame_kind = frame.kind

        angle_sample: AngleSample | None = None
        node_state: FusedNodeState | None = None
        if frame.kind == "fused":
            node_state = self.fused_nodes.get(frame.node_id or 0)
            if node_state is not None and node_state.angle_history:
                latest = node_state.angle_history[-1]
                frame_time_s = now_monotonic - (node_state.session_start_monotonic or now_monotonic)
                if abs(latest.time_s - frame_time_s) < 0.2:
                    angle_sample = latest
                    node_state.capture_samples.append(
                        AngleSample(
                            time_s=capture_time_s,
                            yaw_deg=latest.yaw_deg,
                            yaw_unwrapped_deg=latest.yaw_unwrapped_deg,
                            pitch_deg=latest.pitch_deg,
                            roll_deg=latest.roll_deg,
                            status=frame.status,
                        )
                    )

            self.capture_status_samples += 1
            if (frame.status & 0x03) == 0x03:
                self.capture_status_good_samples += 1
            if node_state is not None:
                node_state.capture_status_samples += 1
                if (frame.status & 0x03) == 0x03:
                    node_state.capture_status_good_samples += 1
                if frame.status & 0x20:
                    self.capture_status_issue_flags.add("calibrating")
                    node_state.capture_status_issue_flags.add("calibrating")
                if frame.status & 0x40:
                    self.capture_status_issue_flags.add("filter warmup")
                    node_state.capture_status_issue_flags.add("filter warmup")
                if frame.status & 0x08:
                    self.capture_status_issue_flags.add("single IMU mode")
                    node_state.capture_status_issue_flags.add("single IMU mode")

        row = {
            "time_s": f"{capture_time_s:.3f}",
            "protocol": frame.kind,
            "sequence": "" if frame.sequence is None else frame.sequence,
            "node_id": "" if frame.node_id is None else frame.node_id,
            "status_hex": f"0x{frame.status:02X}",
            "status_bits": self._format_status_bits(frame.status),
            "quat_w": "",
            "quat_x": "",
            "quat_y": "",
            "quat_z": "",
            "yaw_deg": "",
            "yaw_unwrapped_deg": "",
            "pitch_deg": "",
            "roll_deg": "",
            "imu0_accel_x_mg": "",
            "imu0_accel_y_mg": "",
            "imu0_accel_z_mg": "",
            "imu0_gyro_x_dps": "",
            "imu0_gyro_y_dps": "",
            "imu0_gyro_z_dps": "",
            "imu1_accel_x_mg": "",
            "imu1_accel_y_mg": "",
            "imu1_accel_z_mg": "",
            "imu1_gyro_x_dps": "",
            "imu1_gyro_y_dps": "",
            "imu1_gyro_z_dps": "",
            "guided_test_active": "1" if self.pose_test_active else "0",
            "guided_test_step": "",
            "guided_test_step_label": "",
            "guided_test_step_status": self.pose_test_status_var.get(),
            "guided_test_elapsed_s": "",
            "guided_test_target_node": "",
            "guided_test_target_label": "",
            "guided_test_error_deg": "",
            "guided_test_reference_node": "",
            "guided_test_reference_label": "",
            "palm_ref_quat_w": "",
            "palm_ref_quat_x": "",
            "palm_ref_quat_y": "",
            "palm_ref_quat_z": "",
            "palm_live_quat_w": "",
            "palm_live_quat_x": "",
            "palm_live_quat_y": "",
            "palm_live_quat_z": "",
            "relative_to_palm_quat_w": "",
            "relative_to_palm_quat_x": "",
            "relative_to_palm_quat_y": "",
            "relative_to_palm_quat_z": "",
            "relative_to_palm_yaw_deg": "",
            "relative_to_palm_pitch_deg": "",
            "relative_to_palm_roll_deg": "",
            "raw_frame_hex": frame.raw_frame.hex(" "),
        }

        active_step = self._current_pose_test_step()
        if self.pose_test_node_id is not None:
            row["guided_test_target_node"] = self.pose_test_node_id
            row["guided_test_target_label"] = self._node_label(self.pose_test_node_id)
            row["guided_test_reference_node"] = self.pose_test_node_id
            row["guided_test_reference_label"] = self._node_label(self.pose_test_node_id)
        if self.pose_test_started_monotonic is not None:
            row["guided_test_elapsed_s"] = f"{max(0.0, now_monotonic - self.pose_test_started_monotonic):.3f}"
        if active_step is not None:
            row["guided_test_step"] = active_step.key
            row["guided_test_step_label"] = active_step.label
        if (
            self.pose_test_active
            and frame.kind == "fused"
            and self.pose_test_node_id is not None
            and frame.node_id == self.pose_test_node_id
            and self.pose_test_last_error_deg is not None
        ):
            row["guided_test_error_deg"] = f"{self.pose_test_last_error_deg:.3f}"

        baseline = self.pose_test_baselines.get(0)
        palm_state = self.fused_nodes.get(0)
        palm_frame = palm_state.last_frame if palm_state is not None else None
        palm_quaternion = palm_frame.orientation.quaternion if palm_frame is not None and palm_frame.orientation is not None else None
        if baseline is not None:
            row["palm_ref_quat_w"] = f"{baseline[0]:.6f}"
            row["palm_ref_quat_x"] = f"{baseline[1]:.6f}"
            row["palm_ref_quat_y"] = f"{baseline[2]:.6f}"
            row["palm_ref_quat_z"] = f"{baseline[3]:.6f}"
        if palm_quaternion is not None:
            row["palm_live_quat_w"] = f"{palm_quaternion[0]:.6f}"
            row["palm_live_quat_x"] = f"{palm_quaternion[1]:.6f}"
            row["palm_live_quat_y"] = f"{palm_quaternion[2]:.6f}"
            row["palm_live_quat_z"] = f"{palm_quaternion[3]:.6f}"

        if frame.orientation is not None:
            quat = frame.orientation.quaternion
            row["quat_w"] = f"{quat[0]:.6f}"
            row["quat_x"] = f"{quat[1]:.6f}"
            row["quat_y"] = f"{quat[2]:.6f}"
            row["quat_z"] = f"{quat[3]:.6f}"
            if palm_quaternion is not None:
                relative_to_palm = quaternion_relative_to(
                    quaternion_normalize(palm_quaternion),
                    quaternion_normalize(quat),
                )
                relative_yaw, relative_pitch, relative_roll = quaternion_to_euler_deg(relative_to_palm)
                row["relative_to_palm_quat_w"] = f"{relative_to_palm[0]:.6f}"
                row["relative_to_palm_quat_x"] = f"{relative_to_palm[1]:.6f}"
                row["relative_to_palm_quat_y"] = f"{relative_to_palm[2]:.6f}"
                row["relative_to_palm_quat_z"] = f"{relative_to_palm[3]:.6f}"
                row["relative_to_palm_yaw_deg"] = f"{relative_yaw:.3f}"
                row["relative_to_palm_pitch_deg"] = f"{relative_pitch:.3f}"
                row["relative_to_palm_roll_deg"] = f"{relative_roll:.3f}"
        self._apply_glove_capture_fields(row)
        if angle_sample is not None:
            row["yaw_deg"] = f"{angle_sample.yaw_deg:.3f}"
            row["yaw_unwrapped_deg"] = f"{angle_sample.yaw_unwrapped_deg:.3f}"
            row["pitch_deg"] = f"{angle_sample.pitch_deg:.3f}"
            row["roll_deg"] = f"{angle_sample.roll_deg:.3f}"
        if frame.imu0 is not None and frame.imu1 is not None:
            row["imu0_accel_x_mg"], row["imu0_accel_y_mg"], row["imu0_accel_z_mg"] = frame.imu0.accel_mg
            row["imu0_gyro_x_dps"] = f"{frame.imu0.gyro_dps_x10[0] / 10.0:.3f}"
            row["imu0_gyro_y_dps"] = f"{frame.imu0.gyro_dps_x10[1] / 10.0:.3f}"
            row["imu0_gyro_z_dps"] = f"{frame.imu0.gyro_dps_x10[2] / 10.0:.3f}"
            row["imu1_accel_x_mg"], row["imu1_accel_y_mg"], row["imu1_accel_z_mg"] = frame.imu1.accel_mg
            row["imu1_gyro_x_dps"] = f"{frame.imu1.gyro_dps_x10[0] / 10.0:.3f}"
            row["imu1_gyro_y_dps"] = f"{frame.imu1.gyro_dps_x10[1] / 10.0:.3f}"
            row["imu1_gyro_z_dps"] = f"{frame.imu1.gyro_dps_x10[2] / 10.0:.3f}"

        self.capture_writer.writerow(row)
        self._write_glove_snapshot_jsonl(capture_time_s, frame)
        self.capture_samples_written += 1

    def _build_capture_summary(self, final: bool, node_id: int | None = None) -> str:
        target_node_id = self.selected_node_id if node_id is None else node_id
        node_state = self.fused_nodes.get(target_node_id)
        samples = node_state.capture_samples if final and node_state is not None else (
            list(node_state.angle_history) if node_state is not None else []
        )
        if len(samples) < 2:
            return f"Need more fused quaternion samples to estimate drift for node {target_node_id}."
        description, _family = self._describe_host_node(target_node_id)

        duration_s = samples[-1].time_s - samples[0].time_s
        if duration_s <= 0.0:
            return "Need more elapsed time to estimate drift."

        yaw_rate = self._axis_rate("yaw_unwrapped_deg", samples)
        pitch_rate = self._axis_rate("pitch_deg", samples)
        roll_rate = self._axis_rate("roll_deg", samples)
        rates = {
            "yaw": yaw_rate or 0.0,
            "pitch": pitch_rate or 0.0,
            "roll": roll_rate or 0.0,
        }
        dominant_axis = max(rates, key=lambda key: abs(rates[key]))
        stable_axes = [axis for axis in ("yaw", "pitch", "roll") if axis != dominant_axis and abs(rates[axis]) < 0.5]

        motion_seen = (
            max(sample.yaw_unwrapped_deg for sample in samples) - min(sample.yaw_unwrapped_deg for sample in samples) > MOTION_EVENT_DEG
            or max(sample.pitch_deg for sample in samples) - min(sample.pitch_deg for sample in samples) > MOTION_EVENT_DEG
            or max(sample.roll_deg for sample in samples) - min(sample.roll_deg for sample in samples) > MOTION_EVENT_DEG
        )

        validity = self._validity_text(samples)
        summary = (
            f"{'capture' if final else 'live'} {description} ({target_node_id}): {dominant_axis} drifts {rates[dominant_axis]:+.2f} deg/10s "
            f"over {duration_s:.1f}s; "
            f"yaw {rates['yaw']:+.2f}, pitch {rates['pitch']:+.2f}, roll {rates['roll']:+.2f} deg/10s; "
            f"{'motion seen; ' if motion_seen else ''}{validity.lower()}."
        )
        if stable_axes:
            summary += " Stable axes: " + ", ".join(stable_axes) + "."
        if final and self.capture_protocol_change_count:
            summary += f" Protocol changed {self.capture_protocol_change_count} time(s)."
        return summary

    def _build_tuning_hint(self, final: bool, node_id: int | None = None) -> str:
        target_node_id = self.selected_node_id if node_id is None else node_id
        node_state = self.fused_nodes.get(target_node_id)
        samples = node_state.capture_samples if final and node_state is not None else (
            list(node_state.angle_history) if node_state is not None else []
        )
        if len(samples) < 2:
            return "Run a 30-60 s still capture first. Then tune `PALM_MAHONY_KP`, `PALM_MAHONY_KI`, and the gyro-bias calibration constants in `Core/Inc/imu/imu_config.h`."

        yaw_rate = abs(self._axis_rate("yaw_unwrapped_deg", samples) or 0.0)
        pitch_rate = abs(self._axis_rate("pitch_deg", samples) or 0.0)
        roll_rate = abs(self._axis_rate("roll_deg", samples) or 0.0)
        issues = set(node_state.capture_status_issue_flags) if final and node_state is not None else set()
        if not final:
            issues.update(
                flag
                for flag, mask in (
                    ("calibrating", 0x20),
                    ("filter warmup", 0x40),
                    ("single IMU mode", 0x08),
                )
                if any(sample.status & mask for sample in samples)
            )

        if yaw_rate > max(pitch_rate, roll_rate) and pitch_rate < 0.5 and roll_rate < 0.5:
            hint = (
                "Yaw is drifting more than pitch/roll. In this 6DOF path some yaw walk is normal; "
                "try a small increase in `PALM_MAHONY_KI` first to improve gyro bias trim. "
                "`PALM_MAHONY_KP` will mainly affect tilt correction, not long-term heading lock."
            )
        elif pitch_rate > 0.5 or roll_rate > 0.5:
            hint = (
                "Pitch/roll are drifting while still. Try a modest increase in `PALM_MAHONY_KP`, "
                "then a smaller increase in `PALM_MAHONY_KI` if bias remains. If motion causes noisy correction, back `PALM_MAHONY_KP` down."
            )
        else:
            hint = (
                "Current drift is modest. Keep `PALM_MAHONY_KP` near its present value and only fine-tune "
                "`PALM_MAHONY_KI` in small steps if you still want less slow bias walk."
            )

        if "single IMU mode" in issues:
            hint += " Single-IMU mode appeared, so one sensor may be missing samples or not contributing to the averaged filter path."
        if "calibrating" in issues:
            hint += " Calibration was still active during the window, so ignore the earliest samples or shorten the calibration window only after bias quality looks acceptable."
        if "filter warmup" in issues:
            hint += " Warmup stayed active during the window, so ignore the earliest samples or raise `PALM_IMU_WARMUP_SAMPLES` slightly."
        return hint

    def _set_raw_frame_text(self, text: str) -> None:
        if self.raw_frame_text is None:
            return
        self.raw_frame_text.configure(state=tk.NORMAL)
        self.raw_frame_text.delete("1.0", tk.END)
        self.raw_frame_text.insert("1.0", text)
        self.raw_frame_text.configure(state=tk.DISABLED)

    def _format_frame_hex(self, frame: ParsedFrame) -> str:
        raw = frame.raw_frame
        if frame.kind == "raw":
            sections = [
                ("Header", raw[0:1]),
                ("Type", raw[1:2]),
                ("Seq", raw[2:3]),
                ("IMU0", raw[3:15]),
                ("IMU1", raw[15:27]),
                ("Status", raw[27:28]),
                ("Checksum", raw[28:29]),
            ]
        else:
            sections = [
                ("Header", raw[0:1]),
                ("Force", raw[1:7]),
                ("Quat", raw[7:15]),
                ("Status", raw[15:16]),
                ("Node", raw[16:17]),
                ("Checksum", raw[17:18]),
            ]
        lines = []
        for label, chunk in sections:
            lines.append(f"{label:8}: {' '.join(f'{value:02X}' for value in chunk)}")
        return "\n".join(lines)

    def _format_status_bits(self, status: int) -> str:
        labels = [
            ("IMU0_OK", 0x01),
            ("IMU1_OK", 0x02),
            ("FUSION_READY", 0x04),
            ("SINGLE_IMU", 0x08),
            ("USB_BUSY", 0x10),
            ("CALIBRATING", 0x20),
            ("WARMUP", 0x40),
        ]
        active = [label for label, mask in labels if status & mask]
        return ", ".join(active) if active else "none"

    def _format_section_delta(self, previous: ParsedFrame | None, current: ParsedFrame) -> str:
        if previous is None:
            return "Captured first valid frame"

        if previous.kind != current.kind:
            return f"Protocol changed: {previous.kind.upper()} -> {current.kind.upper()}"

        if current.kind == "raw":
            previous_raw = previous.raw_frame
            current_raw = current.raw_frame
            imu0_changed = previous_raw[3:15] != current_raw[3:15]
            imu1_changed = previous_raw[15:27] != current_raw[15:27]
            status_changed = previous_raw[27] != current_raw[27]
            return (
                f"IMU0 {'changed' if imu0_changed else 'steady'} | "
                f"IMU1 {'changed' if imu1_changed else 'steady'} | "
                f"Status {'changed' if status_changed else 'steady'}"
            )

        previous_raw = previous.raw_frame
        current_raw = current.raw_frame
        previous_node = previous.node_id if previous.node_id is not None else -1
        current_node = current.node_id if current.node_id is not None else -1
        if previous_node != current_node:
            return f"Fused node changed: {previous_node} -> {current_node}"
        quat_changed = previous_raw[7:15] != current_raw[7:15]
        status_changed = previous_raw[15] != current_raw[15]
        return (
            f"Quat {'changed' if quat_changed else 'steady'} | "
            f"Status {'changed' if status_changed else 'steady'} | "
            f"Node {current_node}"
        )

    def on_plot_toggle(self) -> None:
        if self.plot_enabled_var.get() and not MATPLOTLIB_AVAILABLE:
            self.plot_enabled_var.set(False)
            messagebox.showinfo(APP_TITLE, "Install matplotlib to enable live plots.")
            return
        self.redraw_plots(force=True)

    def _on_plot_mode_changed(self, *_args: object) -> None:
        self.clear_plot_history()

    def clear_plot_history(self) -> None:
        for channel_group in self.plot_history.values():
            for series in channel_group:
                series.clear()
        self.quat_plot_history.clear()
        self.redraw_plots(force=True)

    def _update_plot_history(self, frame: ParsedFrame) -> None:
        mode = self._effective_plot_mode(frame)

        if mode == "gyro" and frame.imu0 is not None and frame.imu1 is not None:
            imu0_values = [value / 10.0 for value in frame.imu0.gyro_dps_x10]
            imu1_values = [value / 10.0 for value in frame.imu1.gyro_dps_x10]
            for index, value in enumerate(imu0_values):
                self.plot_history["imu0"][index].append(value)
            for index, value in enumerate(imu1_values):
                self.plot_history["imu1"][index].append(value)
        elif mode == "accel" and frame.imu0 is not None and frame.imu1 is not None:
            for index, value in enumerate(frame.imu0.accel_mg):
                self.plot_history["imu0"][index].append(value)
            for index, value in enumerate(frame.imu1.accel_mg):
                self.plot_history["imu1"][index].append(value)
        elif mode == "quat" and frame.orientation is not None:
            history = self.quat_plot_history.setdefault(
                frame.node_id or 0,
                [deque(maxlen=PLOT_HISTORY) for _ in range(4)],
            )
            for index, value in enumerate(frame.orientation.quaternion):
                history[index].append(value)

        self.redraw_plots()

    def redraw_plots(self, force: bool = False) -> None:
        if not MATPLOTLIB_AVAILABLE or self.figure is None or self.plot_canvas is None:
            return

        if not force and not self.plot_enabled_var.get():
            return

        now = time.monotonic()
        if not force and now - self.last_plot_update < 0.1:
            return
        self.last_plot_update = now

        mode = self._effective_plot_mode(self.last_frame)

        for axis in self.axes:
            axis.clear()

        if not self.plot_enabled_var.get():
            for axis in self.axes:
                axis.text(0.5, 0.5, "Plots disabled", ha="center", va="center", transform=axis.transAxes)
        elif mode in ("accel", "gyro"):
            labels = ("X", "Y", "Z")
            unit = "dps" if mode == "gyro" else "mg"
            for axis, imu_name, history_key in zip(self.axes, ("IMU0", "IMU1"), ("imu0", "imu1")):
                axis.set_title(f"{imu_name} {mode.capitalize()} ({unit})")
                axis.set_xlabel("Samples")
                axis.set_ylabel(unit)
                for label, values in zip(labels, self.plot_history[history_key]):
                    axis.plot(range(len(values)), list(values), label=label)
                axis.legend(loc="upper right")
        elif mode == "quat":
            selected_history = self.quat_plot_history.get(self.selected_node_id, [deque() for _ in range(4)])
            description, _family = self._describe_host_node(self.selected_node_id)
            self.axes[0].set_title(f"{description} ({self.selected_node_id}) Quaternion W/X")
            self.axes[0].set_xlabel("Samples")
            self.axes[0].set_ylabel("unitless")
            for label, values in zip(("W", "X"), selected_history[:2]):
                self.axes[0].plot(range(len(values)), list(values), label=label)
            self.axes[0].legend(loc="upper right")

            self.axes[1].set_title(f"{description} ({self.selected_node_id}) Quaternion Y/Z")
            self.axes[1].set_xlabel("Samples")
            self.axes[1].set_ylabel("unitless")
            for label, values in zip(("Y", "Z"), selected_history[2:]):
                self.axes[1].plot(range(len(values)), list(values), label=label)
            self.axes[1].legend(loc="upper right")
        else:
            for axis in self.axes:
                axis.text(0.5, 0.5, "No compatible frame received yet", ha="center", va="center", transform=axis.transAxes)

        self.figure.tight_layout(pad=2.0)
        self.plot_canvas.draw_idle()

    def _effective_plot_mode(self, frame: ParsedFrame | None) -> str:
        selected = self.plot_mode_var.get()
        if selected != "auto":
            return selected
        if frame is None:
            return "accel"
        return "quat" if frame.kind == "fused" else "accel"

    def _update_connection_buttons(self) -> None:
        connected = self.reader is not None
        if self.connect_button is not None:
            self.connect_button.configure(state=tk.DISABLED if connected else tk.NORMAL)
        if self.disconnect_button is not None:
            self.disconnect_button.configure(state=tk.NORMAL if connected else tk.DISABLED)
        if self.capture_button is not None:
            self.capture_button.configure(
                state=tk.NORMAL if connected and self.capture_writer is None else tk.DISABLED
            )
        if self.stop_capture_button is not None:
            self.stop_capture_button.configure(
                state=tk.NORMAL if self.capture_writer is not None else tk.DISABLED
            )
        if self.port_combo is not None:
            self.port_combo.configure(state=tk.DISABLED if connected else "readonly")
        if self.pose_test_start_button is not None:
            self.pose_test_start_button.configure(state=tk.NORMAL if connected else tk.DISABLED)
        if self.pose_test_zero_button is not None:
            self.pose_test_zero_button.configure(state=tk.NORMAL if connected else tk.DISABLED)
        if self.pose_test_cancel_button is not None:
            self.pose_test_cancel_button.configure(state=tk.NORMAL if connected else tk.DISABLED)
        if self.isolation_start_button is not None:
            self.isolation_start_button.configure(
                state=tk.NORMAL if connected and self.capture_writer is None else tk.DISABLED
            )
        if self.isolation_stop_button is not None:
            self.isolation_stop_button.configure(
                state=tk.NORMAL if self.capture_writer is not None and self.capture_mode == "isolation" else tk.DISABLED
            )
        if self.isolation_analyze_button is not None:
            self.isolation_analyze_button.configure(state=tk.NORMAL)
        if self.phase0_autodetect_button is not None:
            self.phase0_autodetect_button.configure(
                state=tk.NORMAL
                if connected
                and self.capture_writer is None
                and not self._phase0_auto_in_progress
                else tk.DISABLED
            )
        if self.phase0_analyze_button is not None:
            self.phase0_analyze_button.configure(
                state=tk.DISABLED if self._phase0_auto_in_progress else tk.NORMAL
            )
        if self.imu_drift_start_button is not None:
            self.imu_drift_start_button.configure(
                state=tk.NORMAL
                if connected and self.capture_writer is None and not self._imu_drift_test_in_progress
                else tk.DISABLED
            )
        if self.glove_zero_button is not None:
            self.glove_zero_button.configure(state=tk.NORMAL if connected else tk.DISABLED)
        if self.glove_clear_zero_button is not None:
            self.glove_clear_zero_button.configure(state=tk.NORMAL if connected else tk.DISABLED)
        if self.glove_calibrate_button is not None:
            self.glove_calibrate_button.configure(
                state=tk.NORMAL
                if connected and not self._glove_calibration_capture_in_progress
                else tk.DISABLED
            )
        if self.glove_reset_calibration_button is not None:
            self.glove_reset_calibration_button.configure(state=tk.NORMAL)

    def on_close(self) -> None:
        self.disconnect()
        self.root.after(150, self.root.destroy)


def main() -> None:
    root = tk.Tk()
    style = ttk.Style(root)
    if "clam" in style.theme_names():
        style.theme_use("clam")
    app = MonitorApp(root)
    app.refresh_ports()
    root.mainloop()


if __name__ == "__main__":
    main()
