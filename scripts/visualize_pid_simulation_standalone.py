#!/usr/bin/env python3
"""
Visualize heading-hold CSV logs without any kinematic simulation model.
Creates a clean time-series plot from logged detector/controller outputs only.

Usage:
    python scripts/visualize_pid_simulation_standalone.py <csv_file> [--output-plot <path>]

A/B comparison examples (comment-only, run manually):
    # OLD algorithm run
    # python process_video.py videos/your_video.mp4 --output logs/csv/ab_old.csv --video-output result_videos/ab_old.mp4

    # NEW algorithm run
    # python process_video.py videos/your_video.mp4 --output logs/csv/ab_new.csv --video-output result_videos/ab_new.mp4

    # Plot each run (no kinematic traces)
    # python scripts/visualize_pid_simulation_standalone.py logs/csv/ab_old.csv --output-plot logs/csv/ab_old_plot.png
    # python scripts/visualize_pid_simulation_standalone.py logs/csv/ab_new.csv --output-plot logs/csv/ab_new_plot.png
"""

import argparse
import csv
import sys
from pathlib import Path
from typing import Optional

import matplotlib.pyplot as plt


def load_csv_data(
    csv_path: str,
) -> tuple[list[Optional[float]], list[float], list[int], list[str], float]:
    """Load detector/controller values from CSV log."""
    theta_list: list[Optional[float]] = []
    servo_angle_list: list[float] = []
    frame_nums: list[int] = []
    fsm_states: list[str] = []

    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            frame_nums.append(int(row["frame_num"]))
            fsm_states.append(row.get("fsm_state", ""))

            theta_str = row["theta"].strip()
            if theta_str:
                theta_list.append(float(theta_str))
            else:
                theta_list.append(None)

            servo_angle_list.append(float(row["servo_angle"]))

    if not frame_nums:
        raise ValueError("CSV has no rows.")

    servo_center = 90.0
    return theta_list, servo_angle_list, frame_nums, fsm_states, servo_center


def plot_logged_signals(
    frame_nums: list[int],
    theta_list: list[Optional[float]],
    servo_angle_list: list[float],
    fsm_states: list[str],
    servo_center: float,
    output_path: str,
) -> None:
    """Create a 3-panel plot using only logged values (no simulation)."""
    fig, axes = plt.subplots(3, 1, figsize=(16, 10), sharex=True)

    valid_idx = [i for i, t in enumerate(theta_list) if t is not None]
    valid_frames = [frame_nums[i] for i in valid_idx]
    valid_theta = [theta_list[i] for i in valid_idx]

    # Panel 1: Detector theta only.
    ax = axes[0]
    ax.plot(valid_frames, valid_theta, label="Detected theta", color="deepskyblue", linewidth=2.0)
    ax.axhline(y=90.0, color="red", linestyle="--", linewidth=1.5, label="90 deg reference", alpha=0.8)
    ax.set_ylabel("Angle (deg)", fontsize=12, fontweight="bold")
    ax.set_title("Panel 1: Detected Heading", fontsize=14, fontweight="bold")
    ax.grid(True, alpha=0.3)
    ax.set_ylim(0, 180)
    ax.legend(loc="best", fontsize=10)

    # Panel 2: Servo command only.
    ax = axes[1]
    ax.plot(frame_nums, servo_angle_list, label="Servo command", color="limegreen", linewidth=1.8)
    ax.axhline(y=servo_center, color="red", linestyle="--", linewidth=1.5, label="Servo center", alpha=0.8)
    ax.set_ylabel("Angle (deg)", fontsize=12, fontweight="bold")
    ax.set_title("Panel 2: Servo Command", fontsize=14, fontweight="bold")
    ax.grid(True, alpha=0.3)
    ax.set_ylim(0, 180)
    ax.legend(loc="best", fontsize=10)

    # Panel 3: Error and steering offset (derived from logged values only).
    ax = axes[2]
    heading_error: list[Optional[float]] = []
    for theta in theta_list:
        if theta is None:
            heading_error.append(None)
        else:
            heading_error.append(theta - 90.0)
    valid_err_idx = [i for i, e in enumerate(heading_error) if e is not None]
    err_frames = [frame_nums[i] for i in valid_err_idx]
    err_values = [heading_error[i] for i in valid_err_idx]

    servo_offset = [angle - servo_center for angle in servo_angle_list]

    ax.plot(err_frames, err_values, label="Heading error (theta-90)", color="orange", linewidth=1.8)
    ax.plot(frame_nums, servo_offset, label="Servo offset (cmd-center)", color="gold", linewidth=1.2, alpha=0.9)
    ax.axhline(y=0.0, color="gray", linestyle="--", linewidth=1.2, alpha=0.8)
    ax.set_xlabel("Frame Number", fontsize=12, fontweight="bold")
    ax.set_ylabel("Degrees", fontsize=12, fontweight="bold")
    ax.set_title("Panel 3: Logged Error and Offset", fontsize=14, fontweight="bold")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=10)

    # Mark GAPPING spans for easy diagnosis.
    in_gap = False
    gap_start = 0
    for i, state in enumerate(fsm_states):
        if state == "GAPPING" and not in_gap:
            in_gap = True
            gap_start = frame_nums[i]
        elif state != "GAPPING" and in_gap:
            in_gap = False
            gap_end = frame_nums[i]
            for panel_ax in axes:
                panel_ax.axvspan(gap_start, gap_end, color="crimson", alpha=0.08)
    if in_gap:
        gap_end = frame_nums[-1]
        for panel_ax in axes:
            panel_ax.axvspan(gap_start, gap_end, color="crimson", alpha=0.08)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    print(f"Saved plot: {output_path}")
    plt.close()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Visualize heading-hold CSV logs (no kinematic model)."
    )
    parser.add_argument("csv_file", help="Path to CSV log file")
    parser.add_argument(
        "--output-plot",
        type=str,
        required=False,
        default=None,
        help="Output image path (default: <csv_stem>_plot.png)",
    )
    args = parser.parse_args()

    if not Path(args.csv_file).exists():
        print(f"Error: CSV file not found: {args.csv_file}")
        sys.exit(1)

    theta_list, servo_angle_list, frame_nums, fsm_states, servo_center = load_csv_data(args.csv_file)

    output_path = args.output_plot
    if output_path is None:
        output_path = f"{Path(args.csv_file).stem}_plot.png"

    plot_logged_signals(
        frame_nums=frame_nums,
        theta_list=theta_list,
        servo_angle_list=servo_angle_list,
        fsm_states=fsm_states,
        servo_center=servo_center,
        output_path=output_path,
    )


if __name__ == "__main__":
    main()
