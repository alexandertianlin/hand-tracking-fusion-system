from __future__ import annotations

from dataclasses import dataclass
import struct


RAW_FRAME_HEADER = 0xD6
RAW_FRAME_TYPE = 0x01
RAW_FRAME_SIZE = 29
RAW_IMU_SECTION_SIZE = 12

FUSED_FRAME_HEADER = 0xB6
FUSED_FRAME_SIZE = 18
FUSED_QUAT_SCALE = 10000.0

MAX_FRAME_SIZE = max(RAW_FRAME_SIZE, FUSED_FRAME_SIZE)


@dataclass(frozen=True)
class ImuSample:
    accel_mg: tuple[int, int, int]
    gyro_dps_x10: tuple[int, int, int]


@dataclass(frozen=True)
class OrientationSample:
    quaternion: tuple[float, float, float, float]
    raw_i16: tuple[int, int, int, int]


@dataclass(frozen=True)
class ParsedFrame:
    kind: str
    raw_frame: bytes
    status: int
    sequence: int | None = None
    node_id: int | None = None
    imu0: ImuSample | None = None
    imu1: ImuSample | None = None
    orientation: OrientationSample | None = None


@dataclass
class ParserStats:
    packets: int = 0
    raw_packets: int = 0
    fused_packets: int = 0
    checksum_errors: int = 0
    sequence_gaps: int = 0
    bytes_discarded: int = 0


def xor_checksum(data: bytes) -> int:
    checksum = 0
    for value in data:
        checksum ^= value
    return checksum


def decode_raw_frame(frame: bytes) -> ParsedFrame:
    if len(frame) != RAW_FRAME_SIZE:
        raise ValueError(f"expected {RAW_FRAME_SIZE} bytes, got {len(frame)}")
    if frame[0] != RAW_FRAME_HEADER:
        raise ValueError("invalid raw frame header")
    if frame[1] != RAW_FRAME_TYPE:
        raise ValueError("invalid raw frame type")

    expected_checksum = xor_checksum(frame[1:28])
    if expected_checksum != frame[28]:
        raise ValueError("invalid raw frame checksum")

    imu0_values = struct.unpack_from("<6h", frame, 3)
    imu1_values = struct.unpack_from("<6h", frame, 15)

    return ParsedFrame(
        kind="raw",
        raw_frame=bytes(frame),
        sequence=frame[2],
        imu0=ImuSample(
            accel_mg=imu0_values[0:3],
            gyro_dps_x10=imu0_values[3:6],
        ),
        imu1=ImuSample(
            accel_mg=imu1_values[0:3],
            gyro_dps_x10=imu1_values[3:6],
        ),
        status=frame[27],
    )


def decode_fused_frame(frame: bytes) -> ParsedFrame:
    if len(frame) != FUSED_FRAME_SIZE:
        raise ValueError(f"expected {FUSED_FRAME_SIZE} bytes, got {len(frame)}")
    if frame[0] != FUSED_FRAME_HEADER:
        raise ValueError("invalid fused frame header")

    expected_checksum = xor_checksum(frame[1:17])
    if expected_checksum != frame[17]:
        raise ValueError("invalid fused frame checksum")

    raw_quat = struct.unpack_from("<4h", frame, 7)
    quaternion = tuple(value / FUSED_QUAT_SCALE for value in raw_quat)

    return ParsedFrame(
        kind="fused",
        raw_frame=bytes(frame),
        status=frame[15],
        node_id=frame[16],
        orientation=OrientationSample(
            quaternion=quaternion,
            raw_i16=raw_quat,
        ),
    )


def decode_frame(frame: bytes) -> ParsedFrame:
    if not frame:
        raise ValueError("frame is empty")
    if frame[0] == RAW_FRAME_HEADER:
        return decode_raw_frame(frame)
    if frame[0] == FUSED_FRAME_HEADER:
        return decode_fused_frame(frame)
    raise ValueError("unknown frame header")


class FrameParser:
    def __init__(self) -> None:
        self._buffer = bytearray()
        self._last_sequence: int | None = None
        self.stats = ParserStats()

    def reset(self) -> None:
        self._buffer.clear()
        self._last_sequence = None
        self.stats = ParserStats()

    def feed(self, data: bytes) -> list[ParsedFrame]:
        if not data:
            return []

        self._buffer.extend(data)
        parsed_frames: list[ParsedFrame] = []

        while True:
            header_index = self._find_next_header()
            if header_index < 0:
                self._trim_unsynced_tail()
                break

            if header_index > 0:
                del self._buffer[:header_index]
                self.stats.bytes_discarded += header_index

            if not self._buffer:
                break

            header = self._buffer[0]
            if header == RAW_FRAME_HEADER:
                frame = self._try_parse_raw()
            elif header == FUSED_FRAME_HEADER:
                frame = self._try_parse_fused()
            else:
                del self._buffer[0]
                self.stats.bytes_discarded += 1
                continue

            if frame is None:
                break

            parsed_frames.append(frame)

        return parsed_frames

    def _find_next_header(self) -> int:
        raw_index = self._buffer.find(bytes((RAW_FRAME_HEADER,)))
        fused_index = self._buffer.find(bytes((FUSED_FRAME_HEADER,)))

        candidates = [index for index in (raw_index, fused_index) if index >= 0]
        if not candidates:
            return -1
        return min(candidates)

    def _try_parse_raw(self) -> ParsedFrame | None:
        if len(self._buffer) < RAW_FRAME_SIZE:
            return None

        frame = bytes(self._buffer[:RAW_FRAME_SIZE])
        if frame[1] != RAW_FRAME_TYPE:
            del self._buffer[0]
            self.stats.bytes_discarded += 1
            return self._consume_next_if_available()

        expected_checksum = xor_checksum(frame[1:28])
        if expected_checksum != frame[28]:
            del self._buffer[0]
            self.stats.checksum_errors += 1
            self.stats.bytes_discarded += 1
            return self._consume_next_if_available()

        parsed = decode_raw_frame(frame)
        self._update_sequence_stats(parsed.sequence)
        self.stats.packets += 1
        self.stats.raw_packets += 1
        del self._buffer[:RAW_FRAME_SIZE]
        return parsed

    def _try_parse_fused(self) -> ParsedFrame | None:
        if len(self._buffer) < FUSED_FRAME_SIZE:
            return None

        frame = bytes(self._buffer[:FUSED_FRAME_SIZE])
        expected_checksum = xor_checksum(frame[1:17])
        if expected_checksum != frame[17]:
            del self._buffer[0]
            self.stats.checksum_errors += 1
            self.stats.bytes_discarded += 1
            return self._consume_next_if_available()

        parsed = decode_fused_frame(frame)
        self.stats.packets += 1
        self.stats.fused_packets += 1
        del self._buffer[:FUSED_FRAME_SIZE]
        return parsed

    def _consume_next_if_available(self) -> ParsedFrame | None:
        return None

    def _trim_unsynced_tail(self) -> None:
        if len(self._buffer) < MAX_FRAME_SIZE:
            return

        discard_count = len(self._buffer) - (MAX_FRAME_SIZE - 1)
        del self._buffer[:discard_count]
        self.stats.bytes_discarded += discard_count

    def _update_sequence_stats(self, sequence: int | None) -> None:
        if sequence is None:
            return
        if self._last_sequence is None:
            self._last_sequence = sequence
            return

        expected = (self._last_sequence + 1) & 0xFF
        if sequence != expected:
            self.stats.sequence_gaps += (sequence - expected) & 0xFF

        self._last_sequence = sequence
