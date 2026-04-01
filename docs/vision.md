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
  ▼ (5) Grouping        – cluster segments in normal space (rho/theta)
  ▼ (6) Group filter    – keep only horizontal candidates with enough total length
  ▼ (6) Reference       – pick the most horizontal group (angle nearest 0°/180°)
  ▼ (7) Transform       – convert horizontal angle to vertical-equivalent angle
  ▼ (8) Sanity check    – reject if Δθ from previous frame > configured limit
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
| `roi_height_pct` | `0.6` | Height of the trapezoid as a fraction of frame height (bottom portion). |
| `roi_top_width_pct` | `0.75` | Width of the top edge as a fraction of frame width. |
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

### ROI Border Removal

**Problem:**  
Even after masking the ROI, Canny edge detection can respond to the sharp intensity discontinuity at the ROI boundary—the transition from the interior image texture to black (0) outside the mask. This causes spurious edges that form the trapezoid outline, which can be falsely detected as tile-gap lines.

**Solution:**  
A two-stage border-suppression approach:

#### Stage 1: Mask-Level Border Stripping  
The function `_strip_roi_border_hits()` removes the outermost white pixels from the binary ROI mask:
- **Vertical pass:** For each column, scans from the ROI start row downward and clears the first non-zero pixel found. This removes the top edge of the trapezoid.
- **Horizontal pass:** For each row, clears the leftmost and rightmost non-zero pixels to remove the side edges.
- **Bottom cleanup:** Clears the final image row to remove the bottom edge.

Controlled by:
| Constant | Default | Description |
|----------|---------|-------------|
| `_ROI_BORDER_BLACK_PX` | `2` | Polyline thickness (additional border blacking) applied when drawing the ROI boundary. |

#### Stage 2: Edge-Space ROI Erosion  
The `_detect_edges()` function suppresses boundary-induced Canny responses by restricting edges to an eroded inner ROI:
- Build a fresh trapezoid mask after Canny.
- Erode it inward by `_ROI_EDGE_MARGIN_PX` to create a safety margin.
- Apply the eroded mask with `cv2.bitwise_and()` to keep only interior edges.
- Force-clear the bottom `_ROI_BOTTOM_CLEAR_ROWS` rows (where blur/Canny often misses the true image edge) to suppress any remaining horizontal boundary artifacts.

Controlled by:
| Constant | Default | Description |
|----------|---------|-------------|
| `_ROI_EDGE_MARGIN_PX` | `4` px | Inward erosion radius to create a safety margin from ROI boundaries in edge space. |
| `_ROI_BOTTOM_CLEAR_ROWS` | `10` rows | Number of rows cleared from the bottom of the edge map to suppress boundary responses caused by blur shift. |

**Result:**  
The tile-gap line is clearly isolated in the edge map without trapezoid-outline interference.

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

After PPHT, segments are merged into groups in normal-space coordinates.

For each segment, the detector computes:

- `theta_normal = (theta_segment + 90) mod 180`
- `rho = x_mid * cos(theta_normal) + y_mid * sin(theta_normal)`

Then a greedy clustering step groups segments when both conditions pass:

- `|Δtheta_normal| < VISION_CLUSTER_ANGLE_BIAS_DEG`
- `|Δrho| < VISION_CLUSTER_RHO_BIAS_PX`

This mirrors classic Hough clustering behavior while still using PPHT segments.

Each horizontal candidate group must also satisfy:

- total segment length >= `VISION_MIN_GROUP_TOTAL_LENGTH_PX`

### Reference Selection

The group whose **fitted angle is nearest horizontal** (closest to `0°` or `180°`) is chosen as the reference.  
If two groups are equally horizontal, the tie is broken by choosing the one with the higher y-midpoint (closer to the robot).  
The selected horizontal angle is then converted to a vertical-equivalent output angle:

```
θ_out = (θ_horizontal + 90°) mod 180°
```

Examples:
- `0° / 180° -> 90°`
- `20° -> 110°`
- `160° -> 70°`

For multi-segment groups, angle is estimated from a fitted line through segment endpoints (`cv2.fitLine`) to reduce jitter. For single-segment groups, the original segment angle is used.

### Angle Convention

Each segment angle is computed as:

```
θ = atan2(y₂ − y₁, x₂ − x₁) × (180 / π)  mod 180°
```

A line parallel to the robot's forward path (vertical in the image) gives θ ≈ 90°, so the heading error `e = θ − 90° = 0`.

### Environment Configuration

All detector constants are loaded from environment variables in `config/settings.py` (via `.env`).
Use `.env.example` for the full list of detector variables.

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
| `_ROI_BORDER_BLACK_PX` | `2` px | Polyline thickness for initial ROI border blacking. |
| `_ROI_EDGE_MARGIN_PX` | `4` px | Inward erosion radius to suppress boundary-induced Canny responses. |
| `_ROI_BOTTOM_CLEAR_ROWS` | `10` rows | Number of bottom rows cleared in edge map to remove boundary blur artifacts. |
| `_HOUGH_RHO` | `1` px | Distance resolution for Hough transform. |
| `_HOUGH_THETA` | `π/180` rad | Angle resolution for Hough transform. |
| `_HOUGH_THRESHOLD` | `50` | Minimum votes to consider a Hough line. |
| `_HOUGH_MIN_LINE_LEN` | `30` px | Minimum accepted segment length. |
| `_HOUGH_MAX_LINE_GAP` | `10` px | Maximum collinear gap to bridge two segments. |
| `_CLUSTER_ANGLE_BIAS_DEG` | `4.0°` | Max normal-space angle gap used in segment clustering. |
| `_CLUSTER_RHO_BIAS_PX` | `25.0` px | Max normal-space rho gap used in segment clustering. |
| `_MIDPOINT_THRESHOLD` | `30.0` px | Midpoint threshold constant (retained for debug metadata). |
| `_MIN_GROUP_TOTAL_LENGTH_PX` | `120.0` px | Minimum total segment length for a candidate horizontal group. |
| `_HORIZONTAL_MAX_ERROR_DEG` | `20.0°` | Maximum horizontal-angle error before candidate rejection. |
| `_SANITY_MAX_DELTA` | `40.0°` | Max inter-frame angle jump before rejection. |
