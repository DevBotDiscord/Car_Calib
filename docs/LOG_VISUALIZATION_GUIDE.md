# Log Visualization Guide

## Overview

The project now uses log-only visualization. Plots are generated directly from CSV telemetry without any kinematic simulation layer.

Use this guide with:

- scripts/visualize_pid_simulation_standalone.py

The script renders three panels:

1. Detected heading (`theta`) against 90 deg reference.
2. Servo command against servo center.
3. Logged heading error (`theta - 90`) and servo offset (`servo_command - center`).

GAPPING periods are highlighted with a light red background in all panels.

## Run Commands

Generate a plot from one log:

```bash
python scripts/visualize_pid_simulation_standalone.py logs/csv/1_4/log_09_43.csv --output-plot logs/pid_sim/1_4/09_43.png
```

A/B comparison workflow:

```bash
# OLD algorithm run
python process_video.py videos/your_video.mp4 --output logs/csv/ab_old.csv --video-output result_videos/ab_old.mp4

# NEW algorithm run
python process_video.py videos/your_video.mp4 --output logs/csv/ab_new.csv --video-output result_videos/ab_new.mp4

# Plot each run
python scripts/visualize_pid_simulation_standalone.py logs/csv/ab_old.csv --output-plot logs/pid_sim/ab_old.png
python scripts/visualize_pid_simulation_standalone.py logs/csv/ab_new.csv --output-plot logs/pid_sim/ab_new.png
```

## Interpretation Checklist

Use the following quick checks when comparing runs:

- Fewer abrupt `theta` jumps around cluttered regions.
- Fewer and shorter GAPPING spans.
- Smaller oscillation around 90 deg in heading error panel.
- Reduced servo over-correction (offset spikes).

## Notes

- This visualization intentionally excludes abstract kinematic states.
- The output reflects only logged detector and controller telemetry.
