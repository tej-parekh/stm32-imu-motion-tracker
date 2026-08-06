#!/usr/bin/env python3
"""Live 3D comparison of four STM32 IMU attitude estimates.

The script consumes the project's current 30-field UART CSV stream and displays:
    1. accelerometer-only tilt
    2. pure gyro quaternion integration performed in Python
    3. embedded adaptive roll/pitch Kalman estimates
    4. embedded Mahony quaternion estimate

Current UART field order:
    0  sample_index
    1  timestamp_ms
    2  accel_x_raw
    3  accel_y_raw
    4  accel_z_raw
    5  gyro_x_raw
    6  gyro_y_raw
    7  gyro_z_raw
    8  accel_x_g
    9  accel_y_g
    10 accel_z_g
    11 gyro_x_dps
    12 gyro_y_dps
    13 gyro_z_dps
    14 roll_acc_deg
    15 pitch_acc_deg
    16 accel_magnitude_g
    17 roll_kf_deg
    18 pitch_kf_deg
    19 roll_bias_dps
    20 pitch_bias_dps
    21 roll_k_angle
    22 pitch_k_angle
    23 qw
    24 qx
    25 qy
    26 qz
    27 quaternion_roll_deg
    28 quaternion_pitch_deg
    29 quaternion_yaw_deg
"""

from __future__ import annotations

import argparse
import glob
import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np
import serial
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


FIELD_COUNT = 30
DEFAULT_BAUD = 460800
DISPLAY_INTERVAL_MS = 40  # 25 FPS keeps four Matplotlib 3D panels responsive.


# -----------------------------------------------------------------------------
# Quaternion math
# -----------------------------------------------------------------------------

def normalize_quaternion(q: np.ndarray) -> np.ndarray:
    """Return a unit quaternion, or identity if the input is invalid."""
    norm = float(np.linalg.norm(q))
    if not np.isfinite(norm) or norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return q / norm


def euler_deg_to_quaternion(
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
) -> np.ndarray:
    """Convert ZYX yaw-pitch-roll Euler angles to [qw, qx, qy, qz]."""
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    return normalize_quaternion(
        np.array(
            [
                cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
            ],
            dtype=float,
        )
    )


def propagate_gyro_quaternion(
    q: np.ndarray,
    gx_dps: float,
    gy_dps: float,
    gz_dps: float,
    dt_s: float,
) -> np.ndarray:
    """Euler-integrate q_dot = 0.5 * q ⊗ [0, omega]."""
    qw, qx, qy, qz = q
    wx, wy, wz = np.radians([gx_dps, gy_dps, gz_dps])

    q_dot = 0.5 * np.array(
        [
            -qx * wx - qy * wy - qz * wz,
            qw * wx + qy * wz - qz * wy,
            qw * wy - qx * wz + qz * wx,
            qw * wz + qx * wy - qy * wx,
        ],
        dtype=float,
    )

    return normalize_quaternion(q + q_dot * dt_s)


def quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    """Convert [qw, qx, qy, qz] to a body-to-world rotation matrix."""
    qw, qx, qy, qz = normalize_quaternion(q)

    return np.array(
        [
            [
                1.0 - 2.0 * (qy * qy + qz * qz),
                2.0 * (qx * qy - qw * qz),
                2.0 * (qx * qz + qw * qy),
            ],
            [
                2.0 * (qx * qy + qw * qz),
                1.0 - 2.0 * (qx * qx + qz * qz),
                2.0 * (qy * qz - qw * qx),
            ],
            [
                2.0 * (qx * qz - qw * qy),
                2.0 * (qy * qz + qw * qx),
                1.0 - 2.0 * (qx * qx + qy * qy),
            ],
        ],
        dtype=float,
    )


def quaternion_to_euler_deg(q: np.ndarray) -> tuple[float, float, float]:
    """Convert [qw, qx, qy, qz] to ZYX roll, pitch, and yaw in degrees."""
    qw, qx, qy, qz = normalize_quaternion(q)

    roll = math.atan2(
        2.0 * (qw * qx + qy * qz),
        1.0 - 2.0 * (qx * qx + qy * qy),
    )

    pitch_input = 2.0 * (qw * qy - qz * qx)
    pitch = math.asin(max(-1.0, min(1.0, pitch_input)))

    yaw = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )

    return tuple(float(value) for value in np.degrees([roll, pitch, yaw]))


# -----------------------------------------------------------------------------
# Serial parsing and estimator state
# -----------------------------------------------------------------------------

@dataclass(frozen=True)
class OrientationFrame:
    sample_index: int
    timestamp_ms: int
    accel_magnitude_g: float
    q_accel: np.ndarray
    q_gyro: np.ndarray
    q_kalman: np.ndarray
    q_mahony: np.ndarray
    input_rate_hz: float
    bad_lines: int


class LiveOrientationStream:
    """Read every UART sample, integrate gyro at full rate, expose latest frame."""

    def __init__(self, port: str, baud: int) -> None:
        self.port = port
        self.baud = baud
        self.serial = serial.Serial(port, baudrate=baud, timeout=0.20)
        self.serial.reset_input_buffer()

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)

        self._latest: Optional[OrientationFrame] = None
        self._exception: Optional[BaseException] = None
        self._bad_lines = 0
        self._timestamp_history: deque[int] = deque(maxlen=200)

        self._q_gyro: Optional[np.ndarray] = None
        self._previous_timestamp_ms: Optional[int] = None
        self._reset_gyro_requested = False

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=1.0)
        if self.serial.is_open:
            self.serial.close()

    def request_gyro_reset(self) -> None:
        """Reinitialize pure gyro orientation from the current accel tilt."""
        with self._lock:
            self._reset_gyro_requested = True

    def latest(self) -> Optional[OrientationFrame]:
        with self._lock:
            return self._latest

    def exception(self) -> Optional[BaseException]:
        with self._lock:
            return self._exception

    def _record_bad_line(self) -> None:
        with self._lock:
            self._bad_lines += 1

    def _reader_loop(self) -> None:
        try:
            while not self._stop_event.is_set():
                raw_line = self.serial.readline()
                if not raw_line:
                    continue

                try:
                    line = raw_line.decode("ascii", errors="strict").strip()
                    parts = line.split(",")
                    if len(parts) != FIELD_COUNT:
                        self._record_bad_line()
                        continue

                    frame = self._parse_sample(parts)
                    with self._lock:
                        self._latest = frame

                except (UnicodeDecodeError, ValueError, OverflowError):
                    self._record_bad_line()

        except BaseException as exc:  # Preserve background-thread errors for UI.
            with self._lock:
                self._exception = exc

    def _parse_sample(self, parts: list[str]) -> OrientationFrame:
        sample_index = int(parts[0])
        timestamp_ms = int(parts[1])

        gx_dps = float(parts[11])
        gy_dps = float(parts[12])
        gz_dps = float(parts[13])

        roll_acc_deg = float(parts[14])
        pitch_acc_deg = float(parts[15])
        accel_magnitude_g = float(parts[16])

        roll_kf_deg = float(parts[17])
        pitch_kf_deg = float(parts[18])

        q_mahony = normalize_quaternion(
            np.array(
                [
                    float(parts[23]),
                    float(parts[24]),
                    float(parts[25]),
                    float(parts[26]),
                ],
                dtype=float,
            )
        )

        # Accel and the two independent Kalman filters have no yaw estimate.
        q_accel = euler_deg_to_quaternion(roll_acc_deg, pitch_acc_deg, 0.0)
        q_kalman = euler_deg_to_quaternion(roll_kf_deg, pitch_kf_deg, 0.0)

        with self._lock:
            reset_requested = self._reset_gyro_requested
            self._reset_gyro_requested = False

        timestamp_reset = (
            self._previous_timestamp_ms is not None
            and timestamp_ms <= self._previous_timestamp_ms
        )

        if self._q_gyro is None or reset_requested or timestamp_reset:
            # Give pure gyro integration a fair starting tilt, then let it drift.
            self._q_gyro = q_accel.copy()
        elif self._previous_timestamp_ms is not None:
            dt_s = (timestamp_ms - self._previous_timestamp_ms) / 1000.0

            # Normal operation is 0.010 s. Ignore clearly invalid discontinuities.
            if 0.0 < dt_s <= 0.100:
                self._q_gyro = propagate_gyro_quaternion(
                    self._q_gyro,
                    gx_dps,
                    gy_dps,
                    gz_dps,
                    dt_s,
                )

        self._previous_timestamp_ms = timestamp_ms
        self._timestamp_history.append(timestamp_ms)

        input_rate_hz = 0.0
        if len(self._timestamp_history) >= 2:
            elapsed_ms = self._timestamp_history[-1] - self._timestamp_history[0]
            if elapsed_ms > 0:
                input_rate_hz = (
                    1000.0 * (len(self._timestamp_history) - 1) / elapsed_ms
                )

        with self._lock:
            bad_lines = self._bad_lines

        return OrientationFrame(
            sample_index=sample_index,
            timestamp_ms=timestamp_ms,
            accel_magnitude_g=accel_magnitude_g,
            q_accel=q_accel,
            q_gyro=self._q_gyro.copy(),
            q_kalman=q_kalman,
            q_mahony=q_mahony,
            input_rate_hz=input_rate_hz,
            bad_lines=bad_lines,
        )


# -----------------------------------------------------------------------------
# 3D drawing
# -----------------------------------------------------------------------------

def board_geometry() -> tuple[np.ndarray, list[list[int]]]:
    """Return a board-like rectangular prism and its six faces."""
    half_length = 0.82
    half_width = 0.50
    half_height = 0.10

    vertices = np.array(
        [
            [-half_length, -half_width, -half_height],
            [ half_length, -half_width, -half_height],
            [ half_length,  half_width, -half_height],
            [-half_length,  half_width, -half_height],
            [-half_length, -half_width,  half_height],
            [ half_length, -half_width,  half_height],
            [ half_length,  half_width,  half_height],
            [-half_length,  half_width,  half_height],
        ],
        dtype=float,
    )

    faces = [
        [0, 1, 2, 3],
        [4, 5, 6, 7],
        [0, 1, 5, 4],
        [1, 2, 6, 5],
        [2, 3, 7, 6],
        [3, 0, 4, 7],
    ]
    return vertices, faces


class OrientationPanel:
    """One persistent Matplotlib 3D panel updated without clearing the axis."""

    def __init__(
        self,
        axis,
        title: str,
        subtitle: str,
        body_color,
        axis_colors: list,
    ) -> None:
        self.axis = axis
        self.title = title
        self.base_vertices, self.faces = board_geometry()

        axis.set_xlim(-1.15, 1.15)
        axis.set_ylim(-1.15, 1.15)
        axis.set_zlim(-1.15, 1.15)
        axis.set_box_aspect((1.0, 1.0, 1.0))
        axis.view_init(elev=24, azim=38)
        axis.set_xlabel("World X")
        axis.set_ylabel("World Y")
        axis.set_zlabel("World Z")
        axis.set_title(f"{title}\n{subtitle}", pad=12)

        # Static world reference axes.
        world_axis_length = 0.95
        for direction, color in zip(np.eye(3), axis_colors):
            axis.plot(
                [0.0, world_axis_length * direction[0]],
                [0.0, world_axis_length * direction[1]],
                [0.0, world_axis_length * direction[2]],
                linestyle=":",
                linewidth=1.0,
                color=color,
                alpha=0.55,
            )

        initial_faces = [self.base_vertices[face] for face in self.faces]
        self.body = Poly3DCollection(
            initial_faces,
            facecolor=body_color,
            edgecolor=body_color,
            linewidth=1.0,
            alpha=0.78,
        )
        axis.add_collection3d(self.body)

        # A line across the board's front edge makes heading visually obvious.
        self.front_edge, = axis.plot([], [], [], linewidth=4.0, color=body_color)

        # Rotating body-frame axes.
        self.body_axes = []
        for color in axis_colors:
            line, = axis.plot([], [], [], linewidth=2.3, color=color)
            self.body_axes.append(line)

        self.readout = axis.text2D(
            0.03,
            0.03,
            "waiting for data…",
            transform=axis.transAxes,
            family="monospace",
            fontsize=9,
        )

    def update(self, q: np.ndarray) -> None:
        rotation = quaternion_to_rotation_matrix(q)
        rotated_vertices = (rotation @ self.base_vertices.T).T

        self.body.set_verts([rotated_vertices[face] for face in self.faces])

        # The +X face is the front of the virtual board.
        front_indices = [1, 2, 6, 5, 1]
        front = rotated_vertices[front_indices]
        self.front_edge.set_data_3d(front[:, 0], front[:, 1], front[:, 2])

        body_axis_length = 0.88
        for index, line in enumerate(self.body_axes):
            endpoint = rotation[:, index] * body_axis_length
            line.set_data_3d(
                [0.0, endpoint[0]],
                [0.0, endpoint[1]],
                [0.0, endpoint[2]],
            )

        roll_deg, pitch_deg, yaw_deg = quaternion_to_euler_deg(q)
        self.readout.set_text(
            f"roll  {roll_deg:8.2f}°\n"
            f"pitch {pitch_deg:8.2f}°\n"
            f"yaw   {yaw_deg:8.2f}°"
        )


# -----------------------------------------------------------------------------
# Application
# -----------------------------------------------------------------------------

def autodetect_serial_port() -> str:
    candidates: list[str] = []
    for pattern in (
        "/dev/cu.usbmodem*",
        "/dev/ttyACM*",
        "/dev/ttyUSB*",
        "COM*",
    ):
        candidates.extend(glob.glob(pattern))

    candidates = sorted(set(candidates))
    if not candidates:
        raise RuntimeError(
            "No STM32 serial port found. Pass it explicitly with --port."
        )

    if len(candidates) > 1:
        print("Multiple serial ports found:")
        for candidate in candidates:
            print(f"  {candidate}")
        print(f"Using: {candidates[0]}")

    return candidates[0]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Live 3D comparison: accelerometer, gyro integration, "
            "adaptive Kalman, and Mahony"
        )
    )
    parser.add_argument(
        "--port",
        help="Serial device, e.g. /dev/cu.usbmodem1303. Auto-detected if omitted.",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD,
        help=f"UART baud rate (default: {DEFAULT_BAUD}).",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    port = args.port or autodetect_serial_port()

    print(f"Opening {port} at {args.baud} baud")
    print("Controls: R = reset pure gyro from current tilt, Q/Esc = close")

    stream = LiveOrientationStream(port, args.baud)
    stream.start()

    figure = plt.figure(figsize=(14.5, 9.2))
    figure.suptitle(
        "STM32 IMU — Live Attitude Estimator Comparison",
        fontsize=17,
        fontweight="semibold",
    )

    axes = [
        figure.add_subplot(2, 2, index + 1, projection="3d")
        for index in range(4)
    ]

    default_colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    body_colors = [default_colors[index % len(default_colors)] for index in range(4)]
    axis_colors = [default_colors[index % len(default_colors)] for index in (4, 1, 2)]

    panels = [
        OrientationPanel(
            axes[0],
            "Accelerometer Only",
            "gravity reference · noisy · yaw unavailable",
            body_colors[0],
            axis_colors,
        ),
        OrientationPanel(
            axes[1],
            "Pure Gyro Integration",
            "smooth short-term motion · unconstrained drift",
            body_colors[1],
            axis_colors,
        ),
        OrientationPanel(
            axes[2],
            "Adaptive Kalman",
            "stable roll/pitch · acceleration rejection · no yaw",
            body_colors[2],
            axis_colors,
        ),
        OrientationPanel(
            axes[3],
            "Mahony Quaternion",
            "coherent 3D attitude · gravity-corrected tilt",
            body_colors[3],
            axis_colors,
        ),
    ]

    status_text = figure.text(
        0.5,
        0.018,
        "Connecting…",
        ha="center",
        family="monospace",
        fontsize=10,
    )

    figure.subplots_adjust(
        left=0.02,
        right=0.98,
        bottom=0.07,
        top=0.91,
        wspace=0.04,
        hspace=0.17,
    )

    start_wall_time = time.monotonic()
    first_timestamp_ms: Optional[int] = None

    def update(_frame_number: int):
        nonlocal first_timestamp_ms

        background_error = stream.exception()
        if background_error is not None:
            status_text.set_text(f"Serial error: {background_error}")
            return []

        frame = stream.latest()
        if frame is None:
            elapsed = time.monotonic() - start_wall_time
            status_text.set_text(
                f"Waiting for a valid {FIELD_COUNT}-field UART line…  "
                f"{elapsed:4.1f} s"
            )
            return []

        if first_timestamp_ms is None:
            first_timestamp_ms = frame.timestamp_ms

        panels[0].update(frame.q_accel)
        panels[1].update(frame.q_gyro)
        panels[2].update(frame.q_kalman)
        panels[3].update(frame.q_mahony)

        elapsed_s = (frame.timestamp_ms - first_timestamp_ms) / 1000.0
        mahony_norm = float(np.linalg.norm(frame.q_mahony))

        status_text.set_text(
            f"sample {frame.sample_index:,}   "
            f"t = {elapsed_s:7.2f} s   "
            f"input = {frame.input_rate_hz:6.1f} Hz   "
            f"|a| = {frame.accel_magnitude_g:5.3f} g   "
            f"|q_M| = {mahony_norm:7.5f}   "
            f"rejected lines = {frame.bad_lines}"
        )
        return []

    def on_key(event) -> None:
        if event.key in ("q", "escape"):
            plt.close(figure)
        elif event.key == "r":
            stream.request_gyro_reset()
            print("Pure gyro orientation reset from current accelerometer tilt")

    figure.canvas.mpl_connect("key_press_event", on_key)

    # Keep a live reference; otherwise Matplotlib may garbage-collect the animation.
    animation = FuncAnimation(
        figure,
        update,
        interval=DISPLAY_INTERVAL_MS,
        blit=False,
        cache_frame_data=False,
    )
    _ = animation

    try:
        plt.show()
    finally:
        stream.close()


if __name__ == "__main__":
    main()
