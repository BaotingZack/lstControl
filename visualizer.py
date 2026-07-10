"""Visualization helpers for the bridge-crane target control simulation."""

from __future__ import annotations

import os
from typing import Any

import matplotlib

if not os.environ.get("DISPLAY") and os.name != "nt":
    matplotlib.use("Agg")

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
import numpy as np


AXIS_COLORS = {"x": "#2196F3", "y": "#4CAF50", "z": "#FF9800"}
VEL_AXIS_COLORS = {"vx": "#2196F3", "vy": "#4CAF50", "vz": "#FF9800"}
AXIS_TITLES = {
    "x": "X Bridge",
    "y": "Y Trolley",
    "z": "Z Hoist",
}


class CraneVisualizer:
    """Create static plots and optional matplotlib replay for one target command."""

    def __init__(self, config: Any):
        self.config = config

    def plot(
        self,
        history: list[dict],
        phase_boundaries: list[tuple[float, str]],
        deadband_events: list[tuple[float, str, object]] | None = None,
        v_change_events: list[tuple[float, str, object]] | None = None,
        save_path: str = "crane_simulation.png",
    ):
        """Save a control-focused plot for the single-target PD simulation."""
        if not history:
            raise ValueError("history must not be empty")

        t = self._series(history, "t")
        x = self._series(history, "x")
        y = self._series(history, "y")
        z = self._series(history, "z")
        vx = self._series(history, "vx")
        vy = self._series(history, "vy")
        vz = self._series(history, "vz")
        vx_cmd = self._series(history, "vx_cmd")
        vy_cmd = self._series(history, "vy_cmd")
        vz_cmd = self._series(history, "vz_cmd")
        vx_raw = self._series(history, "vx_raw")
        vy_raw = self._series(history, "vy_raw")
        vz_raw = self._series(history, "vz_raw")
        vx_filtered = self._series(history, "vx_filtered")
        vy_filtered = self._series(history, "vy_filtered")
        vz_filtered = self._series(history, "vz_filtered")

        fig = plt.figure(figsize=(15, 10), constrained_layout=True)
        gs = fig.add_gridspec(2, 2, hspace=0.24, wspace=0.18)
        ax_pos = fig.add_subplot(gs[0, 0])
        ax_vel = fig.add_subplot(gs[0, 1])
        ax_dist = fig.add_subplot(gs[1, 0])
        ax_xy = fig.add_subplot(gs[1, 1])

        self._plot_positions(ax_pos, history, t, x, y, z, phase_boundaries)
        self._plot_velocities(
            ax_vel,
            t,
            (vx, vy, vz),
            (vx_cmd, vy_cmd, vz_cmd),
            (vx_raw, vy_raw, vz_raw),
            (vx_filtered, vy_filtered, vz_filtered),
            phase_boundaries,
        )
        self._plot_disturbances(ax_dist, history, t, phase_boundaries)
        self._plot_xy_trajectory(ax_xy, history, x, y)

        fig.suptitle(
            "Bridge Crane PD Velocity Control",
            fontsize=14,
            fontweight="bold",
        )
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        plt.close(fig)
        print(f"Chart saved to: {save_path}")

    def plot_operation_diagram(
        self,
        history: list[dict],
        phase_boundaries: list[tuple[float, str]],
        target_pos: tuple[float, float, float],
        initial_pos: tuple[float, float, float],
        save_path: str = "crane_operation_diagram.png",
    ):
        """Save an operation diagram for one dispatch target."""
        if not history:
            raise ValueError("history must not be empty")

        t = self._series(history, "t")
        x = self._series(history, "x")
        y = self._series(history, "y")
        z = self._series(history, "z")

        fig = plt.figure(figsize=(14, 8), constrained_layout=True)
        gs = fig.add_gridspec(
            2,
            2,
            width_ratios=[1.25, 1.0],
            height_ratios=[1.0, 0.6],
        )
        ax_plan = fig.add_subplot(gs[:, 0])
        ax_z = fig.add_subplot(gs[0, 1])
        ax_seq = fig.add_subplot(gs[1, 1])

        self._plot_operation_plan(ax_plan, x, y, initial_pos, target_pos)
        self._plot_hoist_height(ax_z, t, z, initial_pos, target_pos, phase_boundaries)
        self._plot_arrival_sequence(ax_seq, phase_boundaries, float(t[-1]))

        fig.suptitle("Crane Single-Target Operation Diagram", fontsize=14, fontweight="bold")
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        plt.close(fig)
        print(f"Operation diagram saved to: {save_path}")

    def live_frame_indices(self, history: list[dict], update_hz: float = 10.0) -> list[int]:
        """Sample simulation points at the live-display refresh rate."""
        if update_hz <= 0:
            raise ValueError("update_hz must be positive")
        if not history:
            raise ValueError("history must not be empty")

        period = 1.0 / update_hz
        frame_indices = [0]
        next_t = history[0]["t"] + period

        for i, item in enumerate(history[1:], start=1):
            if item["t"] + 1e-9 >= next_t:
                frame_indices.append(i)
                next_t += period

        if frame_indices[-1] != len(history) - 1:
            frame_indices.append(len(history) - 1)
        return frame_indices

    def show_live(
        self,
        history: list[dict],
        target_pos: tuple[float, float, float],
        initial_pos: tuple[float, float, float],
        update_hz: float = 10.0,
        speed: float = 1.0,
    ):
        """Replay movement with matplotlib when an interactive backend exists.

        The CLI uses the browser-based live view, which also works on headless
        machines. This method is kept for desktop debugging.
        """
        if speed <= 0:
            raise ValueError("speed must be positive")
        if not history:
            raise ValueError("history must not be empty")
        backend = matplotlib.get_backend().lower()
        if "agg" in backend:
            raise RuntimeError(
                "Live display requires an interactive matplotlib backend. "
                "Use main.py --live for the browser replay on headless machines."
            )

        frame_indices = self.live_frame_indices(history, update_hz)
        t = self._series(history, "t")
        x = self._series(history, "x")
        y = self._series(history, "y")
        z = self._series(history, "z")

        margin = 0.8
        x_min, x_max = self._bounds([float(np.min(x)), float(np.max(x)), initial_pos[0], target_pos[0]], margin)
        y_min, y_max = self._bounds([float(np.min(y)), float(np.max(y)), initial_pos[1], target_pos[1]], margin)

        fig = plt.figure(figsize=(12, 6), constrained_layout=True)
        gs = fig.add_gridspec(1, 2, width_ratios=[1.2, 1.0])
        ax_plan = fig.add_subplot(gs[0])
        ax_z = fig.add_subplot(gs[1])

        ax_plan.set_title(f"Live Crane Movement ({update_hz:g} Hz)")
        ax_plan.set_xlabel("X - Bridge (m)")
        ax_plan.set_ylabel("Y - Trolley (m)")
        ax_plan.set_xlim(x_min, x_max)
        ax_plan.set_ylim(y_min, y_max)
        ax_plan.set_aspect("equal", adjustable="box")
        ax_plan.grid(True, alpha=0.25)
        ax_plan.plot(x, y, color="#b0bec5", linewidth=1.0, linestyle="--", label="Full path")
        self._mark_point(ax_plan, initial_pos[0], initial_pos[1], "Initial", "#424242", "o", (7, 7))
        self._mark_point(ax_plan, target_pos[0], target_pos[1], "Target", "#ef6c00", "s", (10, -16))
        traveled_line, = ax_plan.plot([], [], color="#2e7d32", linewidth=2.4, label="Traveled")
        current_dot, = ax_plan.plot(
            [],
            [],
            marker="o",
            markersize=9,
            color="#c62828",
            markeredgecolor="white",
            linestyle="None",
            label="Current",
        )
        status_text = ax_plan.text(
            0.02,
            0.98,
            "",
            transform=ax_plan.transAxes,
            va="top",
            ha="left",
            fontsize=9,
            bbox=dict(facecolor="white", alpha=0.8, edgecolor="#cccccc"),
        )
        ax_plan.legend(fontsize=8, loc="best")

        ax_z.set_title("Live Hoist Height")
        ax_z.set_xlabel("Time (s)")
        ax_z.set_ylabel("Z (m)")
        ax_z.set_xlim(t[0], t[-1])
        z_min, z_max = self._bounds([float(np.min(z)), float(np.max(z)), initial_pos[2], target_pos[2]], 0.2)
        ax_z.set_ylim(z_min, z_max)
        ax_z.grid(True, alpha=0.25)
        ax_z.axhline(target_pos[2], color="#2e7d32", linestyle="--", linewidth=0.9, label="Target Z")
        ax_z.plot(t, z, color="#ffcc80", linewidth=1.0, linestyle="--", label="Full Z profile")
        live_z_line, = ax_z.plot([], [], color=AXIS_COLORS["z"], linewidth=2.4, label="Current Z")
        z_cursor = ax_z.axvline(t[0], color="#c62828", linewidth=1.1, alpha=0.8)
        ax_z.legend(fontsize=8, loc="best")

        def update(frame_no: int):
            idx = frame_indices[frame_no]
            traveled_line.set_data(x[: idx + 1], y[: idx + 1])
            current_dot.set_data([x[idx]], [y[idx]])
            live_z_line.set_data(t[: idx + 1], z[: idx + 1])
            z_cursor.set_xdata([t[idx], t[idx]])
            status_text.set_text(
                f"t={t[idx]:.1f}s\n"
                f"Move to target\n"
                f"X={x[idx]:.2f}  Y={y[idx]:.2f}  Z={z[idx]:.2f}"
            )
            return traveled_line, current_dot, live_z_line, z_cursor, status_text

        animation = FuncAnimation(
            fig,
            update,
            frames=len(frame_indices),
            interval=1000.0 / update_hz / speed,
            blit=False,
            repeat=False,
        )
        plt.show()
        return animation

    def _plot_positions(self, ax, history, t, x, y, z, arrival_events):
        targets = {
            "x": self._series(history, "p_ref_x"),
            "y": self._series(history, "p_ref_y"),
            "z": self._series(history, "p_ref_z"),
        }
        actuals = {"x": x, "y": y, "z": z}

        for axis, values in actuals.items():
            ax.plot(
                t,
                values,
                color=AXIS_COLORS[axis],
                linewidth=1.35,
                label=f"{AXIS_TITLES[axis]} actual",
            )
            ax.plot(
                t,
                targets[axis],
                color=AXIS_COLORS[axis],
                linestyle=":",
                linewidth=1.0,
                alpha=0.75,
                label=f"{AXIS_TITLES[axis]} target",
            )

        self._add_arrival_lines(ax, arrival_events)
        ax.set_title("Position Feedback vs Target")
        ax.set_ylabel("Position (m)")
        ax.set_xlim(t[0], t[-1])
        ax.grid(True, alpha=0.28)
        ax.legend(loc="upper right", fontsize=8, ncol=3)

    def _plot_velocities(self, ax, t, actual, commands, raw, filtered, arrival_events):
        actual_names = ("vx", "vy", "vz")
        for name, values in zip(actual_names, raw):
            ax.plot(
                t,
                values,
                color=VEL_AXIS_COLORS[name],
                linewidth=0.55,
                linestyle=":",
                alpha=0.25,
                label="_nolegend_",
            )
        for name, values in zip(actual_names, actual):
            axis = name[-1]
            ax.plot(
                t,
                values,
                color=VEL_AXIS_COLORS[name],
                linewidth=1.2,
                label=f"{AXIS_TITLES[axis]} actual",
            )
        for name, values in zip(actual_names, commands):
            axis = name[-1]
            ax.plot(
                t,
                values,
                color=VEL_AXIS_COLORS[name],
                linewidth=0.95,
                linestyle="--",
                alpha=0.65,
                label=f"{AXIS_TITLES[axis]} cmd",
            )
        for name, values in zip(actual_names, filtered):
            axis = name[-1]
            ax.plot(
                t,
                values,
                color=VEL_AXIS_COLORS[name],
                linewidth=0.8,
                linestyle="-.",
                alpha=0.55,
                label=f"{AXIS_TITLES[axis]} filtered D",
            )

        vmax_xy = self.config.max_velocity_xy
        vmax_z = self.config.max_velocity_z
        ax.axhline(+vmax_xy, color="#607d8b", linestyle=":", linewidth=0.8, alpha=0.6)
        ax.axhline(-vmax_xy, color="#607d8b", linestyle=":", linewidth=0.8, alpha=0.6)
        ax.axhline(+vmax_z, color=AXIS_COLORS["z"], linestyle=":", linewidth=0.8, alpha=0.5)
        ax.axhline(-vmax_z, color=AXIS_COLORS["z"], linestyle=":", linewidth=0.8, alpha=0.5)

        self._add_arrival_lines(ax, arrival_events)
        ax.set_title("Velocity: cmd / servo / filtered-D, faint dotted = raw diff")
        ax.set_ylabel("Velocity (m/s)")
        ax.set_xlim(t[0], t[-1])
        velocity_window = max(vmax_xy, vmax_z) * 1.75
        ax.set_ylim(-velocity_window, velocity_window)
        ax.grid(True, alpha=0.28)
        ax.legend(loc="upper right", fontsize=6.5, ncol=3)

    def _plot_disturbances(self, ax, history, t, arrival_events):
        for key, color, label in [
            ("disturbance_x", AXIS_COLORS["x"], "X disturbance"),
            ("disturbance_y", AXIS_COLORS["y"], "Y disturbance"),
            ("disturbance_z", AXIS_COLORS["z"], "Z disturbance"),
        ]:
            if key in history[0]:
                ax.plot(t, self._series(history, key), color=color, linewidth=1.0, label=label)

        self._add_arrival_lines(ax, arrival_events)
        ax.set_title("Equivalent Velocity Disturbance")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Disturbance (m/s)")
        ax.set_xlim(t[0], t[-1])
        ax.grid(True, alpha=0.28)
        ax.legend(loc="upper right", fontsize=8)

    def _plot_xy_trajectory(self, ax, history, x, y):
        initial = (x[0], y[0])
        target = (history[-1].get("p_ref_x", x[-1]), history[-1].get("p_ref_y", y[-1]))

        ax.plot(x, y, color="#455a64", linewidth=2.0, label="Trajectory")
        ax.scatter(x, y, s=6, color="#90a4ae", alpha=0.28, zorder=2)
        self._mark_point(ax, initial[0], initial[1], "Initial", "#424242", "o", (7, 7))
        self._mark_point(ax, target[0], target[1], "Target", "#ef6c00", "s", (10, -16))
        self._mark_point(ax, x[-1], y[-1], "Final", "#c62828", "X", (10, 8))
        ax.annotate(
            "",
            xy=target,
            xytext=initial,
            arrowprops=dict(arrowstyle="->", color="#78909c", linewidth=1.2, alpha=0.75),
        )

        ax.set_title("XY Plane Trajectory")
        ax.set_xlabel("X - Bridge (m)")
        ax.set_ylabel("Y - Trolley (m)")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.28)
        ax.legend(fontsize=8, loc="best")

    def _plot_operation_plan(self, ax, x, y, initial_pos, target_pos):
        margin = 0.8
        x_min, x_max = self._bounds(
            [float(np.min(x)), float(np.max(x)), initial_pos[0], target_pos[0]],
            margin,
        )
        y_min, y_max = self._bounds(
            [float(np.min(y)), float(np.max(y)), initial_pos[1], target_pos[1]],
            margin,
        )

        ax.add_patch(
            mpatches.Rectangle(
                (x_min, y_min),
                x_max - x_min,
                y_max - y_min,
                facecolor="#fafafa",
                edgecolor="#bdbdbd",
                linewidth=1.0,
                zorder=0,
            )
        )
        rail_y0 = y_min + 0.18 * (y_max - y_min)
        rail_y1 = y_max - 0.18 * (y_max - y_min)
        ax.plot([x_min, x_max], [rail_y0, rail_y0], color="#9e9e9e", linewidth=2.0, alpha=0.75)
        ax.plot([x_min, x_max], [rail_y1, rail_y1], color="#9e9e9e", linewidth=2.0, alpha=0.75)

        ax.plot(x, y, color="#2e7d32", linewidth=2.3, alpha=0.9, label="Controlled path")
        self._mark_point(ax, initial_pos[0], initial_pos[1], "Initial", "#424242", "o", (7, 7))
        self._mark_point(ax, target_pos[0], target_pos[1], "Target", "#ef6c00", "s", (10, -16))
        self._mark_point(ax, x[-1], y[-1], "Final", "#c62828", "X", (10, 8))
        ax.annotate(
            "",
            xy=(target_pos[0], target_pos[1]),
            xytext=(initial_pos[0], initial_pos[1]),
            arrowprops=dict(arrowstyle="->", color="#616161", linewidth=1.5, alpha=0.72),
        )

        ax.set_title("XY Plan View: X Bridge + Y Trolley")
        ax.set_xlabel("X - Bridge (m)")
        ax.set_ylabel("Y - Trolley (m)")
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.25)
        ax.legend(fontsize=8, loc="best")

    def _plot_hoist_height(self, ax, t, z, initial_pos, target_pos, arrival_events):
        ax.plot(t, z, color=AXIS_COLORS["z"], linewidth=2.0, label="Hoist Z")
        for value, label, color in [
            (initial_pos[2], "Initial Z", "#424242"),
            (target_pos[2], "Target Z", "#ef6c00"),
        ]:
            ax.axhline(value, color=color, linestyle="--", linewidth=0.9, alpha=0.75)
            ax.text(t[-1], value, f" {label}", va="center", ha="left", fontsize=8, color=color)
        self._add_arrival_lines(ax, arrival_events)
        ax.set_title("Hoist Height Profile")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Z (m)")
        ax.set_xlim(t[0], t[-1] * 1.08)
        ax.grid(True, alpha=0.25)
        ax.legend(fontsize=8, loc="best")

    def _plot_arrival_sequence(self, ax, arrival_events, t_max):
        ax.set_title("Axis Arrival Sequence")
        ax.barh(0, t_max, left=0.0, height=0.38, color="#e8f5e9", edgecolor="#81c784")
        ax.text(t_max / 2.0, 0, "Move to target", ha="center", va="center", fontsize=8, color="#2e7d32")

        colors = {"x": AXIS_COLORS["x"], "y": AXIS_COLORS["y"], "z": AXIS_COLORS["z"]}
        label_offsets = {"z": 0.34, "y": 0.58, "x": 0.34}
        for event_t, axis in arrival_events:
            color = colors.get(axis, "#757575")
            ax.axvline(event_t, color=color, linestyle="--", linewidth=1.1, alpha=0.9)
            ax.text(
                event_t,
                label_offsets.get(axis, 0.34),
                f"{axis.upper()} arrived\n{event_t:.1f}s",
                ha="center",
                va="bottom",
                fontsize=7,
                color=color,
            )

        ax.set_xlim(0, max(t_max, 1e-6))
        ax.set_ylim(-0.55, 0.78)
        ax.set_yticks([])
        ax.set_xlabel("Time (s)")
        ax.grid(True, alpha=0.25, axis="x")

    def _add_arrival_lines(self, ax, arrival_events):
        colors = {"x": AXIS_COLORS["x"], "y": AXIS_COLORS["y"], "z": AXIS_COLORS["z"]}
        for event_t, axis in arrival_events:
            ax.axvline(
                event_t,
                color=colors.get(axis, "#757575"),
                linestyle="--",
                linewidth=0.85,
                alpha=0.7,
            )

    @staticmethod
    def _series(history: list[dict], key: str) -> np.ndarray:
        return np.array([item[key] for item in history], dtype=float)

    @staticmethod
    def _bounds(values: list[float], margin: float) -> tuple[float, float]:
        low = min(values) - margin
        high = max(values) + margin
        if abs(high - low) < 1e-9:
            low -= margin
            high += margin
        return low, high

    @staticmethod
    def _mark_point(ax, x, y, label, color, marker, offset):
        ax.scatter(
            x,
            y,
            s=90,
            marker=marker,
            color=color,
            edgecolors="white",
            linewidth=1.0,
            zorder=5,
        )
        ax.annotate(
            label,
            (x, y),
            textcoords="offset points",
            xytext=offset,
            fontsize=8,
            color=color,
            fontweight="bold",
        )
