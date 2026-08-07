#!/usr/bin/env python3

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
DISPLAY_INTERVAL_MS = 40
VIEW_ORDER = ("front", "iso")


def normalize_quaternion(q: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(q))
    if not np.isfinite(norm) or norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return q / norm


def euler_to_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    # ZYX Euler convention: body -> world rotation.
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def matrix_to_euler_deg(rotation: np.ndarray) -> tuple[float, float, float]:
    pitch = math.asin(max(-1.0, min(1.0, -float(rotation[2, 0]))))

    if abs(math.cos(pitch)) > 1e-7:
        roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
        yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    else:
        roll = math.atan2(-float(rotation[1, 2]), float(rotation[1, 1]))
        yaw = 0.0

    return tuple(float(v) for v in np.degrees([roll, pitch, yaw]))


def matrix_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    trace = float(np.trace(rotation))

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (rotation[2, 1] - rotation[1, 2]) / s
        qy = (rotation[0, 2] - rotation[2, 0]) / s
        qz = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        qw = (rotation[2, 1] - rotation[1, 2]) / s
        qx = 0.25 * s
        qy = (rotation[0, 1] + rotation[1, 0]) / s
        qz = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        qw = (rotation[0, 2] - rotation[2, 0]) / s
        qx = (rotation[0, 1] + rotation[1, 0]) / s
        qy = 0.25 * s
        qz = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        qw = (rotation[1, 0] - rotation[0, 1]) / s
        qx = (rotation[0, 2] + rotation[2, 0]) / s
        qy = (rotation[1, 2] + rotation[2, 1]) / s
        qz = 0.25 * s

    return normalize_quaternion(np.array([qw, qx, qy, qz], dtype=float))


def quaternion_to_matrix(q: np.ndarray) -> np.ndarray:
    qw, qx, qy, qz = normalize_quaternion(q)

    return np.array(
        [
            [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qw * qz), 2.0 * (qx * qz + qw * qy)],
            [2.0 * (qx * qy + qw * qz), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qw * qx)],
            [2.0 * (qx * qz - qw * qy), 2.0 * (qy * qz + qw * qx), 1.0 - 2.0 * (qx * qx + qy * qy)],
        ],
        dtype=float,
    )


def propagate_gyro_quaternion(
    q: np.ndarray,
    gx_dps: float,
    gy_dps: float,
    gz_dps: float,
    dt_s: float,
) -> np.ndarray:
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


@dataclass(frozen=True)
class Frame:
    sample_index: int
    timestamp_ms: int
    gx_dps: float
    gy_dps: float
    gz_dps: float
    accel_magnitude_g: float
    roll_acc_deg: float
    pitch_acc_deg: float
    roll_kf_deg: float
    pitch_kf_deg: float
    q_mahony: np.ndarray
    mahony_roll_deg: float
    mahony_pitch_deg: float
    mahony_yaw_deg: float
    input_rate_hz: float
    bad_lines: int


class SerialStream:
    def __init__(self, port: str, baud: int) -> None:
        self.serial = serial.Serial(port, baudrate=baud, timeout=0.20)
        self.serial.reset_input_buffer()

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._latest: Optional[Frame] = None
        self._exception: Optional[BaseException] = None
        self._bad_lines = 0
        self._timestamps: deque[int] = deque(maxlen=200)
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        if self.serial.is_open:
            self.serial.close()

    def latest(self) -> Optional[Frame]:
        with self._lock:
            return self._latest

    def exception(self) -> Optional[BaseException]:
        with self._lock:
            return self._exception

    def _reader_loop(self) -> None:
        try:
            while not self._stop.is_set():
                raw = self.serial.readline()
                if not raw:
                    continue

                try:
                    parts = raw.decode("ascii", errors="strict").strip().split(",")
                    if len(parts) != FIELD_COUNT:
                        with self._lock:
                            self._bad_lines += 1
                        continue

                    frame = self._parse(parts)
                    with self._lock:
                        self._latest = frame

                except (UnicodeDecodeError, ValueError, OverflowError):
                    with self._lock:
                        self._bad_lines += 1

        except BaseException as exc:
            with self._lock:
                self._exception = exc

    def _parse(self, parts: list[str]) -> Frame:
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
                [float(parts[23]), float(parts[24]), float(parts[25]), float(parts[26])],
                dtype=float,
            )
        )

        mahony_roll_deg = float(parts[27])
        mahony_pitch_deg = float(parts[28])
        mahony_yaw_deg = float(parts[29])

        self._timestamps.append(timestamp_ms)
        input_rate_hz = 0.0

        if len(self._timestamps) >= 2:
            elapsed_ms = self._timestamps[-1] - self._timestamps[0]
            if elapsed_ms > 0:
                input_rate_hz = 1000.0 * (len(self._timestamps) - 1) / elapsed_ms

        with self._lock:
            bad_lines = self._bad_lines

        return Frame(
            sample_index=sample_index,
            timestamp_ms=timestamp_ms,
            gx_dps=gx_dps,
            gy_dps=gy_dps,
            gz_dps=gz_dps,
            accel_magnitude_g=accel_magnitude_g,
            roll_acc_deg=roll_acc_deg,
            pitch_acc_deg=pitch_acc_deg,
            roll_kf_deg=roll_kf_deg,
            pitch_kf_deg=pitch_kf_deg,
            q_mahony=q_mahony,
            mahony_roll_deg=mahony_roll_deg,
            mahony_pitch_deg=mahony_pitch_deg,
            mahony_yaw_deg=mahony_yaw_deg,
            input_rate_hz=input_rate_hz,
            bad_lines=bad_lines,
        )


def board_geometry():
    """Thin Nucleo-style rectangular board, scaled to a 70 x 82.5 mm footprint."""
    half_x = 0.68
    half_y = 0.80
    half_z = 0.055

    vertices = np.array(
        [
            [-half_x, -half_y, -half_z],  # 0
            [ half_x, -half_y, -half_z],  # 1
            [ half_x,  half_y, -half_z],  # 2
            [-half_x,  half_y, -half_z],  # 3
            [-half_x, -half_y,  half_z],  # 4
            [ half_x, -half_y,  half_z],  # 5
            [ half_x,  half_y,  half_z],  # 6
            [-half_x,  half_y,  half_z],  # 7
        ],
        dtype=float,
    )

    faces = {
        "bottom": [0, 1, 2, 3],
        "top": [4, 5, 6, 7],
        "minus_y": [0, 1, 5, 4],
        "plus_x": [1, 2, 6, 5],
        "plus_y": [2, 3, 7, 6],
        "minus_x": [3, 0, 4, 7],
    }

    edges = [
        [0, 1], [1, 2], [2, 3], [3, 0],
        [4, 5], [5, 6], [6, 7], [7, 4],
        [0, 4], [1, 5], [2, 6], [3, 7],
    ]

    return vertices, faces, edges, half_x, half_y, half_z


class Panel:
    def __init__(self, axis, title: str, body_color) -> None:
        self.axis = axis
        (
            self.base_vertices,
            self.faces,
            self.edge_indices,
            self.half_x,
            self.half_y,
            self.half_z,
        ) = board_geometry()

        axis.set_xlim(-1.14, 1.14)
        axis.set_ylim(-1.14, 1.14)
        axis.set_zlim(-0.88, 1.08)
        axis.set_box_aspect((1.0, 1.0, 0.84))
        axis.set_axis_off()
        axis.set_facecolor("white")

        if hasattr(axis, "disable_mouse_rotation"):
            axis.disable_mouse_rotation()

        self.title_text = axis.text2D(
            0.5,
            0.965,
            title,
            transform=axis.transAxes,
            ha="center",
            va="top",
            fontsize=12.5,
            fontweight="semibold",
        )

        self.angle_text = axis.text2D(
            0.5,
            0.018,
            "R    0.0°   P    0.0°   Y    0.0°",
            transform=axis.transAxes,
            ha="center",
            va="bottom",
            family="monospace",
            fontsize=9.1,
            bbox=dict(
                boxstyle="round,pad=0.28",
                facecolor="white",
                edgecolor="0.88",
                alpha=0.94,
            ),
        )

        floor_z = -0.70
        self.floor_lines = []

        for value in np.linspace(-1.0, 1.0, 5):
            line_a, = axis.plot(
                [-1.0, 1.0],
                [value, value],
                [floor_z, floor_z],
                color="0.76",
                alpha=0.20,
                linewidth=0.72,
            )

            line_b, = axis.plot(
                [value, value],
                [-1.0, 1.0],
                [floor_z, floor_z],
                color="0.76",
                alpha=0.20,
                linewidth=0.72,
            )

            self.floor_lines.extend(
                [line_a, line_b]
            )

        # World-frame axis triad. In FRONT mode only Y/Z are shown because +X
        # points directly toward the viewer and would collapse to a point.
        triad_origin = np.array(
            [0.0, -0.90, -0.68],
            dtype=float,
        )

        tx, ty, tz = triad_origin
        triad_len = 0.38

        self.world_x_arrow = axis.quiver(
            tx, ty, tz,
            triad_len, 0.0, 0.0,
            color="tab:red",
            alpha=0.94,
            arrow_length_ratio=0.20,
            linewidth=2.0,
        )

        self.world_y_arrow = axis.quiver(
            tx, ty, tz,
            0.0, triad_len, 0.0,
            color="tab:green",
            alpha=0.94,
            arrow_length_ratio=0.20,
            linewidth=2.0,
        )

        self.world_z_arrow = axis.quiver(
            tx, ty, tz,
            0.0, 0.0, triad_len,
            color="tab:blue",
            alpha=0.94,
            arrow_length_ratio=0.20,
            linewidth=2.0,
        )

        self.world_x_label = axis.text(
            tx + triad_len + 0.05,
            ty,
            tz,
            "X",
            color="tab:red",
            fontsize=10,
            fontweight="bold",
        )

        self.world_y_label = axis.text(
            tx,
            ty + triad_len + 0.05,
            tz,
            "Y",
            color="tab:green",
            fontsize=10,
            fontweight="bold",
        )

        self.world_z_label = axis.text(
            tx,
            ty,
            tz + triad_len + 0.05,
            "Z",
            color="tab:blue",
            fontsize=10,
            fontweight="bold",
        )

        # Main board: semi-transparent body, with only the +Y end face
        # strongly distinguished. This is a common rigid-body visualization
        # trick: one "front" face removes the 180-degree ambiguity without
        # adding symbols to the body.
        base_rgba = plt.matplotlib.colors.to_rgba(
            body_color,
            0.42,
        )

        side_rgba = plt.matplotlib.colors.to_rgba(
            body_color,
            0.30,
        )

        top_rgba = plt.matplotlib.colors.to_rgba(
            body_color,
            0.50,
        )

        # Warm, high-contrast +Y end cap.
        front_rgba = (0.0, 0.0, 0.0, 0.88)

        self.face_artists = {}

        for name, face in self.faces.items():
            if name == "plus_y":
                face_color = front_rgba
            elif name == "top":
                face_color = top_rgba
            elif name == "bottom":
                face_color = base_rgba
            else:
                face_color = side_rgba

            collection = Poly3DCollection(
                [self.base_vertices[face]],
                facecolor=face_color,
                edgecolor="none",
                linewidth=0.0,
            )

            axis.add_collection3d(collection)
            self.face_artists[name] = collection

        # A crisp wireframe keeps the dimensions readable through the
        # translucent faces.
        self.edge_artists = []

        for _ in self.edge_indices:
            line, = axis.plot(
                [],
                [],
                [],
                color="0.28",
                linewidth=1.25,
                alpha=0.88,
            )

            self.edge_artists.append(line)

    def set_view(self, mode: str) -> None:
        if mode == "front":
            # +X toward viewer, +Y screen-right, +Z screen-up.
            self.axis.view_init(
                elev=0,
                azim=0,
                roll=0,
            )

            self.axis.set_proj_type("ortho")
            show_floor = False

            # X is exactly along the camera direction, so only the screen-plane
            # Y/Z axis pair is drawn here.
            self.world_x_arrow.set_visible(False)
            self.world_x_label.set_visible(False)

        else:
            self.axis.view_init(
                elev=24,
                azim=-55,
                roll=0,
            )

            self.axis.set_proj_type("persp")
            show_floor = True

            self.world_x_arrow.set_visible(True)
            self.world_x_label.set_visible(True)

        self.world_y_arrow.set_visible(True)
        self.world_z_arrow.set_visible(True)
        self.world_y_label.set_visible(True)
        self.world_z_label.set_visible(True)

        for line in self.floor_lines:
            line.set_visible(show_floor)

    def update(self, rotation: np.ndarray) -> None:
        rotated = (
            rotation
            @ self.base_vertices.T
        ).T

        for name, face in self.faces.items():
            self.face_artists[name].set_verts(
                [rotated[face]]
            )

        for line, edge in zip(
            self.edge_artists,
            self.edge_indices,
        ):
            points = rotated[edge]

            line.set_data_3d(
                points[:, 0],
                points[:, 1],
                points[:, 2],
            )

        roll, pitch, yaw = (
            matrix_to_euler_deg(rotation)
        )

        self.angle_text.set_text(
            f"R {roll:7.1f}°   "
            f"P {pitch:7.1f}°   "
            f"Y {yaw:7.1f}°"
        )


class DashboardState:
    def __init__(self) -> None:
        self.q_gyro: Optional[np.ndarray] = None
        self.previous_timestamp_ms: Optional[int] = None
        self.references: Optional[list[np.ndarray]] = None
        self.last_raw_rotations: Optional[list[np.ndarray]] = None

    def raw_rotations(self, frame: Frame) -> list[np.ndarray]:
        dt_s = 0.0

        if self.previous_timestamp_ms is not None:
            delta_ms = frame.timestamp_ms - self.previous_timestamp_ms
            if 0 < delta_ms <= 100:
                dt_s = delta_ms / 1000.0

        self.previous_timestamp_ms = frame.timestamp_ms

        if self.q_gyro is None:
            self.q_gyro = matrix_to_quaternion(
                euler_to_matrix(frame.roll_acc_deg, frame.pitch_acc_deg, 0.0)
            )
        elif dt_s > 0.0:
            self.q_gyro = propagate_gyro_quaternion(
                self.q_gyro,
                frame.gx_dps,
                frame.gy_dps,
                frame.gz_dps,
                dt_s,
            )

        gyro_rotation = quaternion_to_matrix(self.q_gyro)
        _, _, gyro_yaw_deg = matrix_to_euler_deg(gyro_rotation)

        # The STM32 supplies accelerometer tilt, Kalman roll/pitch, and Mahony
        # attitude. Python only integrates a gyro-only attitude because that
        # quantity is not streamed separately by the firmware.
        accel_rotation = euler_to_matrix(frame.roll_acc_deg, frame.pitch_acc_deg, 0.0)
        kf_rotation = euler_to_matrix(frame.roll_kf_deg, frame.pitch_kf_deg, gyro_yaw_deg)
        mahony_rotation = euler_to_matrix(
            frame.mahony_roll_deg,
            frame.mahony_pitch_deg,
            frame.mahony_yaw_deg,
        )

        raw = [accel_rotation, gyro_rotation, kf_rotation, mahony_rotation]
        self.last_raw_rotations = raw
        return raw

    def display_rotations(self, frame: Frame) -> list[np.ndarray]:
        raw = self.raw_rotations(frame)

        if self.references is None:
            self.references = [rotation.copy() for rotation in raw]

        return [reference.T @ rotation for reference, rotation in zip(self.references, raw)]

    def set_room_zero(self) -> None:
        if self.last_raw_rotations is None:
            return
        self.references = [rotation.copy() for rotation in self.last_raw_rotations]


def autodetect_port() -> str:
    candidates: list[str] = []
    for pattern in ("/dev/cu.usbmodem*", "/dev/ttyACM*", "/dev/ttyUSB*", "COM*"):
        candidates.extend(glob.glob(pattern))

    candidates = sorted(set(candidates))
    if not candidates:
        raise RuntimeError("No serial port found. Use --port.")
    return candidates[0]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--view", choices=VIEW_ORDER, default="front")
    return parser.parse_args()


def view_description(mode: str) -> str:
    return "FRONT" if mode == "front" else "ISO"


def build_figure(initial_view: str):
    figure = plt.figure(figsize=(14.2, 8.8), facecolor="0.96")
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]

    grid = figure.add_gridspec(
        4,
        2,
        height_ratios=[0.13, 1.0, 1.0, 0.13],
        left=0.025,
        right=0.975,
        bottom=0.025,
        top=0.985,
        wspace=0.035,
        hspace=0.08,
    )

    header = figure.add_subplot(grid[0, :])
    header.set_axis_off()
    header.set_facecolor("0.96")

    header.text(
        0.0,
        0.52,
        "STM32 IMU Attitude Estimation",
        ha="left",
        va="center",
        fontsize=16,
        fontweight="semibold",
    )

    view_text = header.text(
        0.67,
        0.52,
        view_description(initial_view),
        ha="center",
        va="center",
        family="monospace",
        fontsize=9.5,
        fontweight="semibold",
        bbox=dict(boxstyle="round,pad=0.35", facecolor="white", edgecolor="0.86"),
    )

    header.text(
        1.0,
        0.52,
        "R zero   V view   Q quit",
        ha="right",
        va="center",
        family="monospace",
        fontsize=9.2,
        color="0.32",
    )

    axes = [
        figure.add_subplot(grid[1, 0], projection="3d"),
        figure.add_subplot(grid[1, 1], projection="3d"),
        figure.add_subplot(grid[2, 0], projection="3d"),
        figure.add_subplot(grid[2, 1], projection="3d"),
    ]

    titles = [
        "Accelerometer",
        "Gyro Integration",
        "Adaptive KF + Gyro Yaw",
        "Mahony",
    ]

    panels = [
        Panel(axis, title, colors[i % len(colors)])
        for i, (axis, title) in enumerate(zip(axes, titles))
    ]

    for panel in panels:
        panel.set_view(initial_view)

    footer = figure.add_subplot(grid[3, :])
    footer.set_axis_off()
    footer.set_facecolor("0.96")

    status_left = footer.text(
        0.0,
        0.5,
        "Waiting for data",
        ha="left",
        va="center",
        family="monospace",
        fontsize=9.5,
    )

    status_right = footer.text(
        1.0,
        0.5,
        "",
        ha="right",
        va="center",
        family="monospace",
        fontsize=9.5,
        color="0.32",
    )

    return figure, panels, view_text, status_left, status_right


def main() -> None:
    args = parse_args()
    port = args.port or autodetect_port()

    stream = SerialStream(port, args.baud)
    stream.start()
    state = DashboardState()

    current_view = args.view
    notice_text = ""
    notice_until = 0.0

    figure, panels, view_text, status_left, status_right = build_figure(current_view)

    def set_view(mode: str) -> None:
        nonlocal current_view
        current_view = mode
        for panel in panels:
            panel.set_view(mode)
        view_text.set_text(view_description(mode))
        figure.canvas.draw_idle()

    def cycle_view() -> None:
        index = VIEW_ORDER.index(current_view)
        set_view(VIEW_ORDER[(index + 1) % len(VIEW_ORDER)])

    def update(_):
        nonlocal notice_text, notice_until

        error = stream.exception()
        if error is not None:
            status_left.set_text(f"Serial error: {error}")
            status_right.set_text("")
            return []

        frame = stream.latest()
        if frame is None:
            return []

        rotations = state.display_rotations(frame)
        for panel, rotation in zip(panels, rotations):
            panel.update(rotation)

        q_norm = float(np.linalg.norm(frame.q_mahony))
        status_left.set_text(
            f"{frame.input_rate_hz:5.1f} Hz    |a| {frame.accel_magnitude_g:5.3f} g    "
            f"|q| {q_norm:7.5f}    bad {frame.bad_lines}"
        )

        if time.monotonic() < notice_until:
            status_right.set_text(notice_text)
        else:
            status_right.set_text(f"sample {frame.sample_index:,}")

        return []

    def on_key(event) -> None:
        nonlocal notice_text, notice_until

        if event.key in ("q", "escape"):
            plt.close(figure)
        elif event.key == "r":
            state.set_room_zero()
            notice_text = "ROOM ZERO SET"
            notice_until = time.monotonic() + 1.2
        elif event.key == "v":
            cycle_view()
        elif event.key == "1":
            set_view("front")
        elif event.key == "2":
            set_view("iso")

    figure.canvas.mpl_connect("key_press_event", on_key)
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