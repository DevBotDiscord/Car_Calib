#!/usr/bin/env python3
"""
Visualize PID simulation states from CSV logs.
Creates both a time-series plot and a standalone simulation video.

Usage:
    python scripts/visualize_pid_simulation.py <csv_file> [--output-plot <path>] [--output-video <path>] [--sim-speed <deg/s>] [--panel-height <px>]
"""

import argparse
import csv
import sys
from pathlib import Path
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np

# cv2 is optional - only needed for video output
try:
    import cv2
    HAS_CV2 = True
except (ImportError, AttributeError):
    HAS_CV2 = False

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from visualization.pid_simulation import PIDSimulationVisualizer


def load_csv_data(csv_path: str) -> tuple[list[float], list[float], list[int]]:
    """Load theta and servo_angle from CSV."""
    theta_list = []
    servo_angle_list = []
    frame_nums = []
    
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            frame_nums.append(int(row['frame_num']))
            theta_list.append(float(row['theta']))
            servo_angle_list.append(float(row['servo_angle']))
    
    return theta_list, servo_angle_list, frame_nums


def regenerate_simulation_states(
    theta_list: list[float],
    servo_angle_list: list[float],
    sim_speed_deg_per_sec: float = 120.0,
) -> dict:
    """Regenerate PID simulation states from CSV data."""
    visualizer = PIDSimulationVisualizer(speed_deg_per_sec=sim_speed_deg_per_sec)
    
    states = {
        'predicted_theta': [],
        'measured_heading': [],
        'servo_heading': [],
        'servo_rate': [],
    }
    
    # Assume 30 Hz frame rate (roughly 33ms per frame)
    dt = 1.0 / 30.0
    
    for theta, servo_angle in zip(theta_list, servo_angle_list):
        state = visualizer.update(theta, servo_angle, dt)
        if state is not None:
            states['predicted_theta'].append(state.predicted_theta)
            states['measured_heading'].append(state.measured_heading)
            states['servo_heading'].append(state.servo_heading)
            states['servo_rate'].append(state.servo_rate)
        else:
            # If state is None, append None to maintain alignment
            states['predicted_theta'].append(None)
            states['measured_heading'].append(None)
            states['servo_heading'].append(None)
            states['servo_rate'].append(None)
    
    return states


def plot_simulation(
    frame_nums: list[int],
    theta_list: list[float],
    servo_angle_list: list[float],
    states: dict,
    output_path: Optional[str] = None,
) -> None:
    """Create a matplotlib plot of simulation states over time."""
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    
    # Plot 1: Detected vs Simulated Heading (Kinematic Speed Model)
    ax = axes[0]
    ax.plot(frame_nums, theta_list, label='Detected θ', color='yellow', linewidth=2, alpha=0.8)
    
    # Filter out None values for plotting
    valid_indices = [i for i, v in enumerate(states['predicted_theta']) if v is not None]
    valid_frames = [frame_nums[i] for i in valid_indices]
    valid_predicted = [states['predicted_theta'][i] for i in valid_indices]
    valid_measured = [states['measured_heading'][i] for i in valid_indices]
    
    ax.plot(valid_frames, valid_predicted, label='Predicted θ (target)', color='cyan', linewidth=1.5, alpha=0.7)
    ax.plot(valid_frames, valid_measured, label='Measured Heading (kinematic)', color='blue', linewidth=2)
    ax.set_ylabel('Angle (degrees)', fontsize=12)
    ax.set_title('Detected + Kinematic Speed Model Panel', fontsize=14, fontweight='bold')
    ax.legend(loc='best')
    ax.grid(True, alpha=0.3)
    ax.set_ylim(0, 180)
    
    # Plot 2: Servo Integrated Response
    ax = axes[1]
    valid_servo = [states['servo_heading'][i] for i in valid_indices]
    ax.plot(valid_frames, valid_servo, label='Servo Integrated Heading', color='green', linewidth=2)
    ax.axhline(y=90, color='white', linestyle='--', label='90° Reference', alpha=0.7)
    ax.set_ylabel('Angle (degrees)', fontsize=12)
    ax.set_title('Servo/PID Integrated Response Panel', fontsize=14, fontweight='bold')
    ax.legend(loc='best')
    ax.grid(True, alpha=0.3)
    ax.set_ylim(0, 180)
    
    # Plot 3: Servo Rate (steering command)
    ax = axes[2]
    valid_servo_rate = [states['servo_rate'][i] for i in valid_indices]
    ax.plot(valid_frames, valid_servo_rate, label='Servo Rate (deg/s)', color='orange', linewidth=2)
    ax.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
    ax.set_xlabel('Frame Number', fontsize=12)
    ax.set_ylabel('Rate (deg/s)', fontsize=12)
    ax.set_title('Servo Rate Over Time', fontsize=14, fontweight='bold')
    ax.legend(loc='best')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"✓ Plot saved to: {output_path}")
    else:
        plt.show()
    
    plt.close()


def create_simulation_video(
    frame_nums: list[int],
    theta_list: list[float],
    servo_angle_list: list[float],
    states: dict,
    output_path: str,
    panel_height: int = 240,
) -> None:
    """Create a video of just the simulation panels."""
    if not HAS_CV2:
        print("Warning: cv2 not available. Skipping video generation.")
        print("  (NumPy 2.x compatibility issue with current cv2 version)")
        return
    
    visualizer = PIDSimulationVisualizer(speed_deg_per_sec=120.0)
    panel_width = 600  # Width for each panel (side-by-side)
    
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    fps = 30.0
    out_size = (panel_width * 2, panel_height)
    
    writer = cv2.VideoWriter(output_path, fourcc, fps, out_size)
    
    if not writer.isOpened():
        print(f"Error: Could not open video writer for {output_path}")
        return
    
    dt = 1.0 / 30.0
    
    for i, (theta, servo_angle) in enumerate(zip(theta_list, servo_angle_list)):
        state = visualizer.update(theta, servo_angle, dt)
        
        if state is not None:
            panel = visualizer.render(
                width=panel_width * 2,
                height=panel_height,
                theta_detected=theta,
                servo_angle=servo_angle,
                state=state,
            )
            
            # Ensure BGR format for OpenCV
            if len(panel.shape) == 3 and panel.shape[2] == 3:
                panel = cv2.cvtColor(panel, cv2.COLOR_RGB2BGR)
            elif len(panel.shape) == 2:
                panel = cv2.cvtColor(panel, cv2.COLOR_GRAY2BGR)
            
            # Resize to output size if needed
            if panel.shape[:2] != out_size:
                panel = cv2.resize(panel, out_size)
            
            writer.write(panel)
        
        if (i + 1) % 100 == 0:
            print(f"  Frame {i + 1}/{len(theta_list)}")
    
    writer.release()
    print(f"✓ Simulation video saved to: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description='Visualize PID simulation states from CSV logs.'
    )
    parser.add_argument('csv_file', help='Path to CSV log file')
    parser.add_argument(
        '--output-plot',
        type=str,
        default=None,
        help='Output path for matplotlib plot (e.g., plot.png). Default: show in window',
    )
    parser.add_argument(
        '--output-video',
        type=str,
        default=None,
        help='Output path for simulation video (e.g., sim.mp4)',
    )
    parser.add_argument(
        '--sim-speed',
        type=float,
        default=120.0,
        help='Kinematic speed for simulation (degrees/second). Default: 120',
    )
    parser.add_argument(
        '--panel-height',
        type=int,
        default=240,
        help='Height of simulation panels in video (pixels). Default: 240',
    )
    
    args = parser.parse_args()
    
    # Validate CSV file exists
    if not Path(args.csv_file).exists():
        print(f"Error: CSV file not found: {args.csv_file}")
        sys.exit(1)
    
    print(f"Loading CSV: {args.csv_file}")
    theta_list, servo_angle_list, frame_nums = load_csv_data(args.csv_file)
    print(f"  Loaded {len(theta_list)} frames")
    
    print(f"Regenerating simulation states (speed={args.sim_speed} deg/s)...")
    states = regenerate_simulation_states(theta_list, servo_angle_list, args.sim_speed)
    print(f"  Regenerated {len(theta_list)} simulation states")
    
    # Create plot
    if args.output_plot or not args.output_video:
        print("\nGenerating matplotlib plot...")
        plot_path = args.output_plot or 'pid_simulation_plot.png'
        plot_simulation(frame_nums, theta_list, servo_angle_list, states, plot_path)
    
    # Create video
    if args.output_video:
        print(f"\nGenerating simulation video (panel_height={args.panel_height}px)...")
        create_simulation_video(
            frame_nums, theta_list, servo_angle_list, states,
            args.output_video,
            panel_height=args.panel_height,
        )
    
    print("\n✓ Done!")


if __name__ == '__main__':
    main()
