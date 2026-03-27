# Vision Module

The `vision` package provides the computer-vision pipeline that extracts heading information from camera frames.

There is a single class that handles the complete pipeline:

| Class | File | Purpose |
|-------|------|---------|
| `LineDetector` | `vision/detector.py` | Full tile-gap angle detector with trapezoid ROI, grouping, reference selection, and sanity checks |

---

## `LineDetector` (`vision/detector.py`)

### Overview

`LineDetector` runs a seven-stage pipeline on every frame and returns the **tile-gap angle θ** (degrees).  
The heading error is `e = θ − 90°`; a line parallel to the robot's forward direction gives `e = 0`.

**Pipeline:**

```
Frame
  │
  ▼ (1) Trapezoid mask  – apply cv2.fillPoly with ROI geometry from RobotState
  ▼ (2) Pre-process     – CLAHE equalisation + 5×5 Gaussian blur
  ▼ (3) Edge detection  – Canny
  ▼ (4) Line detection  – Probabilistic Hough Transform (PPHT)
  ▼ (5) Grouping        – merge segments with |Δθ| < 3° and close midpoints
  ▼ (6) Reference       – pick the group lowest in the image (nearest the robot)
  ▼ (7) Sanity check    – reject if Δθ from previous frame > 20°
  │
  └─→ angle θ (float, degrees, range [0°, 180°)) or None
```

### Constructor

```python
LineDetector(state: RobotState)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `state` | `models.robot_state.RobotState` | Shared state supplying ROI geometry parameters and the `debug_mode` flag. |

### Public Method

#### `get_reference_angle(frame: np.ndarray) → Optional[float]`

Runs the complete pipeline and returns the reference tile-gap angle.

```python
from models.robot_state import RobotState
from vision.detector import LineDetector

state = RobotState()
detector = LineDetector(state)
theta = detector.get_reference_angle(frame)   # e.g. 87.5 or None
# heading error: e = theta - 90.0
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `frame` | `np.ndarray` | Full BGR (or grayscale) camera frame. |

**Returns:** `float` — angle θ in degrees relative to the x-axis (range `[0°, 180°)`), or `None` if no valid tile-gap line is found.

---

### Trapezoid ROI

The ROI is a filled trapezoid centred horizontally on the frame, shaped by three parameters from `RobotState`:

| `RobotState` field | Default | Description |
|--------------------|---------|-------------|
| `roi_height_pct` | `0.4` | Height of the trapezoid as a fraction of frame height (bottom portion). |
| `roi_top_width_pct` | `0.6` | Width of the top edge as a fraction of frame width. |
| `roi_bottom_width_pct` | `1.0` | Width of the bottom edge as a fraction of frame width. |

The four vertices are computed as:

```
cx = frame_width // 2
top_y    = frame_height − int(frame_height × roi_height_pct)
top_left  = (cx − top_w // 2,  top_y)
top_right = (cx + top_w // 2,  top_y)
bot_right = (cx + bot_w // 2,  frame_height − 1)
bot_left  = (cx − bot_w // 2,  frame_height − 1)
```

The mask is applied with `cv2.fillPoly`; all pixels outside the trapezoid are zeroed.

### Debug Mode

When `state.debug_mode = True`, the binary mask is written to `debug_mask.jpg` **once** on the first call to `get_reference_angle()`.  
Inspect this file to verify the trapezoid covers the intended floor region.

```python
state = RobotState(debug_mode=True)
detector = LineDetector(state)
detector.get_reference_angle(frame)  # writes debug_mask.jpg
```

---

### Grouping Logic

After PPHT, segments are merged into groups using a greedy algorithm:

1. For each unassigned segment **i**, create a new group.
2. Add any other unassigned segment **j** that satisfies:
   - `|Δθ(i, j)| < 3°` (similar slope)
   - Euclidean midpoint distance `< 50 px` (spatially close)

### Reference Selection

The group with the **highest y-midpoint** (closest to the bottom of the image — nearest the robot) is chosen as the reference.  
The final angle is the **length-weighted average** of all segments in the winning group:

```
θ_avg = Σ(θ_i · length_i) / Σ(length_i)
```

### Angle Convention

Each segment angle is computed as:

```
θ = atan2(y₂ − y₁, x₂ − x₁) × (180 / π)  mod 180°
```

A line parallel to the robot's forward path (vertical in the image) gives θ ≈ 90°, so the heading error `e = θ − 90° = 0`.

### Sanity Check

If the angle from the current frame differs by more than **20°** from the previous valid angle, the detection is discarded and `None` is returned.  
This prevents sudden large steering corrections caused by transient noise.

---

### Tunable Constants

| Constant | Default | Description |
|----------|---------|-------------|
| `_CLAHE_CLIP_LIMIT` | `2.0` | CLAHE clip limit for contrast enhancement. |
| `_CLAHE_TILE_GRID` | `(8, 8)` | CLAHE tile grid size. |
| `_BLUR_KERNEL` | `(5, 5)` | Gaussian blur kernel size (must be odd). |
| `_CANNY_LOW` | `50` | Canny lower hysteresis threshold. |
| `_CANNY_HIGH` | `150` | Canny upper hysteresis threshold. |
| `_HOUGH_RHO` | `1` px | Distance resolution for Hough transform. |
| `_HOUGH_THETA` | `π/180` rad | Angle resolution for Hough transform. |
| `_HOUGH_THRESHOLD` | `50` | Minimum votes to consider a Hough line. |
| `_HOUGH_MIN_LINE_LEN` | `30` px | Minimum accepted segment length. |
| `_HOUGH_MAX_LINE_GAP` | `10` px | Maximum collinear gap to bridge two segments. |
| `_ANGLE_THRESHOLD` | `3.0°` | Max angle difference to merge segments into a group. |
| `_MIDPOINT_THRESHOLD` | `50.0` px | Max midpoint distance to merge segments into a group. |
| `_SANITY_MAX_DELTA` | `20.0°` | Max inter-frame angle jump before rejection. |
