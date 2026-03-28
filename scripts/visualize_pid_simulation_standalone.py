#!/usr/bin/env python3
"""
Visualize PID simulation states from CSV logs (standalone - no cv2 dependency).
Creates a time-series plot of simulation states.

Usage:
    python scripts/visualize_pid_simulation_standalone.py <csv_file> [--output-plot <path>] [--sim-speed <deg/s>]
"""

import argparse
import csv
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


@dataclass
class SimulationState:
    """Dataclass holding PID simulation state at a single timestep."""
    predicted_theta: float
    measured_heading: float
    measured_velocity: float
    servo_heading: float
    servo_rate: float


class SimplePIDSimulator:
    """Minimal PID simulation (no rendering, just state math)."""
    
    def __init__(self, speed_deg_per_sec: float = 120.0):
        self.speed_deg_per_sec = speed_deg_per_sec
        self.last_detected_theta: Optional[float] = None
        self.last_detected_velocity: float = 0.0
        self.measured_heading: float = 0.0
        self.servo_heading: float = 0.0
        self._initialized = False
    
    def _wrap_180(self, angle: float) -> float:
        """Wrap angle to [-180, 180]."""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle
    
    def _shortest_delta(self, target: float, current: float) -> float:
        """Compute shortest angular delta from current to target."""
        delta = target - current
        return self._wrap_180(delta)
    
    def _clamp(self, value: float, min_val: float, max_val: float) -> float:
        """Clamp value to range."""
        return max(min_val, min(value, max_val))
    
    def update(
        self,
        theta_detected: Optional[float],
        servo_angle: float,
        dt: float,
    ) -> SimulationState:
        """Update simulation state."""
        
        # Initialize on first call
        if not self._initialized:
            if theta_detected is not None:
                self.measured_heading = theta_detected
                self.last_detected_theta = theta_detected
            else:
                self.measured_heading = 0.0
            self._initialized = True
        
        # On detection: update predicted theta and measure velocity
        if theta_detected is not None:
            self.measured_velocity = self._shortest_delta(
                theta_detected, self.last_detected_theta
            ) / max(dt, 0.001)
            self.last_detected_theta = theta_detected
            predicted_theta = theta_detected
        else:
            # On miss: continue prediction with last velocity
            predicted_theta = self.last_detected_theta or self.measured_heading
            if self.last_detected_theta is not None:
                predicted_theta += self.last_detected_velocity * dt
        
        # Update measured heading toward predicted (with kinematic speed limit)
        max_step = self.speed_deg_per_sec * dt
        delta = self._shortest_delta(predicted_theta, self.measured_heading)
        step = self._clamp(delta, -max_step, max_step)
        self.measured_heading = self._wrap_180(self.measured_heading + step)
        
        # Compute servo rate from steering offset
        max_steering_offset = 45.0  # Assumes max ±45° steering
        steering_offset = self._shortest_delta(servo_angle, 90.0)
        normalized_offset = steering_offset / max_steering_offset
        self.servo_rate = normalized_offset * self.speed_deg_per_sec
        
        # Integrate servo heading
        self.servo_heading = self._wrap_180(
            self.servo_heading + self.servo_rate * dt
        )
        
        # Store last velocity for prediction
        if theta_detected is not None:
            self.last_detected_velocity = self.measured_velocity
        
        return SimulationState(
            predicted_theta=predicted_theta,
            measured_heading=self.measured_heading,
            measured_velocity=self.measured_velocity,
            servo_heading=self.servo_heading,
            servo_rate=self.servo_rate,
        )


def load_csv_data(csv_path: str) -> tuple[list[Optional[float]], list[float], list[int]]:
    """Load theta and servo_angle from CSV. Handles missing theta values."""
    theta_list = []
    servo_angle_list = []
    frame_nums = []
    
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            frame_nums.append(int(row['frame_num']))
            
            # Handle missing theta (when detection fails)
            theta_str = row['theta'].strip()
            if theta_str:
                theta_list.append(float(theta_str))
            else:
                theta_list.append(None)
            
            servo_angle_list.append(float(row['servo_angle']))
    
    return theta_list, servo_angle_list, frame_nums


def regenerate_simulation_states(
    theta_list: list[Optional[float]],
    servo_angle_list: list[float],
    sim_speed_deg_per_sec: float = 120.0,
) -> dict:
    """Regenerate PID simulation states from CSV data."""
    simulator = SimplePIDSimulator(speed_deg_per_sec=sim_speed_deg_per_sec)
    
    states = {
        'predicted_theta': [],
        'measured_heading': [],
        'servo_heading': [],
        'servo_rate': [],
        'measured_velocity': [],
    }
    
    # Assume 30 Hz frame rate (33ms per frame)
    dt = 1.0 / 30.0
    
    for theta, servo_angle in zip(theta_list, servo_angle_list):
        state = simulator.update(theta, servo_angle, dt)
        states['predicted_theta'].append(state.predicted_theta)
        states['measured_heading'].append(state.measured_heading)
        states['servo_heading'].append(state.servo_heading)
        states['servo_rate'].append(state.servo_rate)
        states['measured_velocity'].append(state.measured_velocity)
    
    return states


def plot_simulation(
    frame_nums: list[int],
    theta_list: list[Optional[float]],
    servo_angle_list: list[float],
    states: dict,
    output_path: str,
) -> None:
    """Create a matplotlib plot of simulation states over time."""
    fig, axes = plt.subplots(3, 1, figsize=(16, 10))
    
    # Plot 1: Detected vs Simulated Heading (Kinematic Speed Model)
    ax = axes[0]
    
    # Plot theta, handling None values
    valid_theta_indices = [i for i, t in enumerate(theta_list) if t is not None]
    valid_theta_frames = [frame_nums[i] for i in valid_theta_indices]
    valid_theta_values = [theta_list[i] for i in valid_theta_indices]
    ax.plot(valid_theta_frames, valid_theta_values, label='Detected θ', color='yellow', linewidth=2.5, alpha=0.9, marker='o', markersize=2)
    
    ax.plot(frame_nums, states['predicted_theta'], label='Predicted θ (target)', color='cyan', linewidth=1.5, alpha=0.8, linestyle='--')
    ax.plot(frame_nums, states['measured_heading'], label='Measured Heading (kinematic)', color='blue', linewidth=2.5, alpha=0.9)
    ax.set_ylabel('Angle (degrees)', fontsize=13, fontweight='bold')
    ax.set_title('Panel 1: Detected + Kinematic Speed Model (31 deg/s model)', fontsize=15, fontweight='bold')
    ax.legend(loc='best', fontsize=11)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(0, 180)
    
    # Plot 2: Servo Integrated Response
    ax = axes[1]
    ax.plot(frame_nums, states['servo_heading'], label='Servo Integrated Heading', color='green', linewidth=2.5, alpha=0.9)
    ax.axhline(y=90, color='red', linestyle='--', linewidth=2, label='90° Reference', alpha=0.8)
    ax.set_ylabel('Angle (degrees)', fontsize=13, fontweight='bold')
    ax.set_title('Panel 2: Servo/PID Integrated Response', fontsize=15, fontweight='bold')
    ax.legend(loc='best', fontsize=11)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(0, 180)
    
    # Plot 3: Servo Rate (steering command magnitude)
    ax = axes[2]
    ax.plot(frame_nums, states['servo_rate'], label='Servo Rate (deg/s)', color='orange', linewidth=2, alpha=0.9)
    ax.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
    ax.fill_between(frame_nums, 0, states['servo_rate'], alpha=0.3, color='orange')
    ax.set_xlabel('Frame Number', fontsize=13, fontweight='bold')
    ax.set_ylabel('Rate (deg/s)', fontsize=13, fontweight='bold')
    ax.set_title('Panel 3: Servo Rate (steering command)', fontsize=15, fontweight='bold')
    ax.legend(loc='best', fontsize=11)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"✓ Plot saved to: {output_path}")
    plt.close()


def main():
    parser = argparse.ArgumentParser(
        description='Visualize PID simulation states from CSV logs (standalone).'
    )
    parser.add_argument('csv_file', help='Path to CSV log file')
    parser.add_argument(
        '--output-plot',
        type=str,
        required=False,
        default=None,
        help='Output path for matplotlib plot (e.g., plot.png)',
    )
    parser.add_argument(
        '--sim-speed',
        type=float,
        default=120.0,
        help='Kinematic speed for simulation (degrees/second). Default: 120',
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
    print(f"  ✓ Regenerated {len(theta_list)} simulation states")
    
    # Determine output path
    if args.output_plot:
        output_path = args.output_plot
    else:
        # Generate default output name based on CSV input
        csv_name = Path(args.csv_file).stem
        output_path = f"{csv_name}_simulation_plot.png"
    
    print(f"\nGenerating matplotlib plot...")
    print(f"  Output: {output_path}")
    plot_simulation(frame_nums, theta_list, servo_angle_list, states, output_path)
    
    print("\n✓ Done!")


if __name__ == '__main__':
    main()
