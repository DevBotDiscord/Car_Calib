# PID Simulation Visualization Guide

## Overview

I've created a visualization tool that shows the **PID simulation movement over time** by extracting simulation states from your video processing logs. Two example visualizations are provided below.

## What the Plots Show

Each plot has **3 panels** showing different aspects of the PID control system:

### **Panel 1: Detected + Kinematic Speed Model**
- **Yellow dots/line** (`Detected θ`): Raw detected heading angle from vision system
- **Cyan dashed line** (`Predicted θ`): Target angle for kinematic model
- **Blue solid line** (`Measured Heading`): Simulated heading response with constant acceleration model (120 deg/s default)

**What it shows**: How well the kinematic speed model tracks the detected angle. If the blue line lags behind yellow, it shows the system needs more "speed" to keep up.

### **Panel 2: Servo/PID Integrated Response**
- **Green solid line** (`Servo Integrated Heading`): Heading response from servo/PID controller
- **White dashed line** (`90° Reference`): Center/neutral position

**What it shows**: How the actual servo responds to steering commands. Spikes and oscillations indicate the control effort needed.

### **Panel 3: Servo Rate (Steering Command)**
- **Orange bars** (`Servo Rate`): Rate of change commanded by the PID controller (degrees/second)
- **Shaded area**: Magnitude of steering command

**What it shows**: The control effort over time. Large values = aggressive steering, near-zero = minimal steering.

---

## Generated Files

### Video 1: `log_11_21.csv` (502 frames, ~17 seconds)
**File**: `./logs/pid_plot_11_21.png`

**Key Observations**:
- Early frames (0-100): Kinematic model tracking detected angle smoothly
- Middle frames (150-250): Multiple detection dropouts (gaps in yellow line), blue model continues predicting
- Late frames (300-500): Model settles to steady state around 90° (heading-hold mode)
- **Servo behavior**: Shows sharp steering commands during transitions, then minimal effort in steady state

---

### Video 2: `log_sim_smoke.csv` (4,984 frames, ~2.8 minutes)
**File**: `./logs/pid_plot_sim_smoke.png`

**Key Observations**:
- **Highly dynamic**: Frequent detection changes (noisy yellow dots) across entire sequence
- **Servo oscillation**: Green servo response shows several peaks (frames 500-1500, 2000-3000), indicating active steering
- **Detection dropouts**: Cyan dashes show where detection was lost; blue line continues predicting
- **Steady-state period** (frames 1000-2500): Longer section where servo settles before resuming active control
- **Dense servo commands**: Orange bars show nearly continuous steering activity, suggesting the heading is drifting

---

## How to Use These Visualizations

### Option 1: Generate plots for your own videos
```bash
python scripts/visualize_pid_simulation_standalone.py <csv_file> --output-plot <output.png> --sim-speed 120
```

### Option 2: Experiment with different kinematic speeds
```bash
# Slower model (robot response is sluggish)
python scripts/visualize_pid_simulation_standalone.py ./logs/csv/28_3/log_11_21.csv --output-plot slow.png --sim-speed 80

# Faster model (robot responds quickly)
python scripts/visualize_pid_simulation_standalone.py ./logs/csv/28_3/log_11_21.csv --output-plot fast.png --sim-speed 160
```

If the **blue line lags behind yellow**, increase `--sim-speed`.  
If the **blue line overshoots ahead**, decrease `--sim-speed`.

---

## Interpreting Performance

**Good PID tuning** shows:
- Blue line (measured) stays close to yellow line (detected)
- Green line (servo) shows smooth, predictable response
- Orange rate commands are moderate, not oscillatory

**Poor tuning** shows:
- Blue line lags significantly behind yellow (model too slow)
- Green line oscillates wildly (servo overshoot/hunting)
- Orange bars are erratic or have high-frequency noise (poor dampening)

---

## Technical Details

- **Frame rate**: 30 Hz (assumed from CSV timestamps)
- **Kinematic model**: Simple constant-acceleration model with max bounded step per frame
- **Servo model**: Treats servo angle as steering command; output is integrated heading
- **Missing detections**: When vision fails, the system uses last-known velocity to predict ahead

---

## Files Generated

| File | Content |
|------|---------|
| `./logs/pid_plot_11_21.png` | Video 1 visualization (502 frames) |
| `./logs/pid_plot_sim_smoke.png` | Video 2 visualization (4,984 frames) |
| `scripts/visualize_pid_simulation_standalone.py` | Script to generate plots from any CSV |

Use the standalone script to analyze other videos without cv2 dependency issues!
