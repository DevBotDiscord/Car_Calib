# Vision Module

The `vision` package provides the computer-vision pipeline that extracts heading information from camera frames.  
It contains two classes with different levels of sophistication:

| Class | File | Purpose |
|-------|------|---------|
| `HeadingDetector` | `vision/detector.py` | Simple heading-error detector (returns `e = |θ − 90°|`) |
| `LineProcessor` | `vision/line_processor.py` | Full tile-gap angle detector with grouping, reference selection, and sanity checks |

---

## `HeadingDetector` (`vision/detector.py`)

### Overview

`HeadingDetector` runs a five-stage pipeline on every frame and returns the **heading error** `e` (in degrees).

**Pipeline:**

```
Frame
  │
  ▼ (1) ROI mask  – keep bottom 40 % of the frame
  ▼ (2) Pre-process – CLAHE equalisation + 5×5 Gaussian blur
  ▼ (3) Edge detection – Canny
  ▼ (4) Line detection – Probabilistic Hough Transform (PPHT)
  ▼ (5) Angle → error  – mean θ of all segments, then e = |θ − 90°|
  │
  └─→ heading error e (float, degrees) or None
```

### Constructor

```python
HeadingDetector(roi_keep_fraction: float = 0.4)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `roi_keep_fraction` | `float` | `0.4` | Fraction of frame height to keep (from the bottom). `0.4` discards the top 60 %. |

### Public Method

#### `compute_heading_error(frame: np.ndarray) → Optional[float]`

Runs the complete pipeline and returns the heading error.

```python
detector = HeadingDetector()
error = detector.compute_heading_error(frame)   # e.g. 7.3 or None
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `frame` | `np.ndarray` | Full BGR (or grayscale) camera frame. |

**Returns:** `float` — heading error `e = |θ − 90°|` in degrees, or `None` if no lines are detected.

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

---

## `LineProcessor` (`vision/line_processor.py`)

### Overview

`LineProcessor` provides a more robust pipeline than `HeadingDetector`.  
It adds **line grouping**, **reference selection** (nearest tile gap), and an **inter-frame sanity check** before returning the raw tile-gap angle θ.

**Pipeline:**

```
Frame
  │
  ▼ (1) ROI mask       – keep bottom 40 % of the frame
  ▼ (2) Pre-process    – CLAHE equalisation + 5×5 Gaussian blur
  ▼ (3) Edge detection – Canny
  ▼ (4) Line detection – PPHT
  ▼ (5) Grouping       – merge segments with |Δθ| < 3° and close midpoints
  ▼ (6) Reference      – pick the group lowest in the image (nearest the robot)
  ▼ (7) Sanity check   – reject if Δθ from previous frame > 20°
  │
  └─→ angle θ (float, degrees, range [0°, 180°)) or None
```

### Constructor

```python
LineProcessor(roi_keep_fraction: float = 0.4)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `roi_keep_fraction` | `float` | `0.4` | Fraction of frame height to keep (from the bottom). |

### Public Method

#### `get_reference_angle(frame: np.ndarray) → Optional[float]`

Runs the full pipeline and returns the reference tile-gap angle.

```python
processor = LineProcessor()
theta = processor.get_reference_angle(frame)   # e.g. 87.5 or None
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `frame` | `np.ndarray` | Full BGR (or grayscale) camera frame. |

**Returns:** `float` — angle θ in degrees relative to the x-axis (range `[0°, 180°)`), or `None` if no valid tile-gap line is found.

### Grouping Logic

After PPHT, segments are merged into groups using a greedy algorithm:

1. For each unassigned segment **i**, create a new group.
2. Add any other unassigned segment **j** that satisfies:
   - `|Δθ(i, j)| < 3°` (similar slope)
   - Euclidean midpoint distance `< 50 px` (spatially close)

### Reference Selection

The group with the **highest y-midpoint** (i.e. closest to the bottom of the image — nearest the robot) is chosen as the reference.  
The final angle is the **length-weighted average** of all segments in the winning group:

```
θ_avg = Σ(θ_i · length_i) / Σ(length_i)
```

### Sanity Check

If the angle from the current frame differs by more than **20°** from the previous valid angle, the detection is discarded and `None` is returned.  
This prevents sudden large steering corrections caused by transient noise.

### Tunable Constants

| Constant | Default | Description |
|----------|---------|-------------|
| `_CLAHE_CLIP_LIMIT` | `2.0` | CLAHE clip limit. |
| `_CLAHE_TILE_GRID` | `(8, 8)` | CLAHE tile grid size. |
| `_BLUR_KERNEL` | `(5, 5)` | Gaussian blur kernel size. |
| `_CANNY_LOW` | `50` | Canny lower threshold. |
| `_CANNY_HIGH` | `150` | Canny upper threshold. |
| `_HOUGH_RHO` | `1` px | Hough distance resolution. |
| `_HOUGH_THETA` | `π/180` rad | Hough angle resolution. |
| `_HOUGH_THRESHOLD` | `50` | Minimum Hough votes. |
| `_HOUGH_MIN_LINE_LEN` | `30` px | Minimum segment length. |
| `_HOUGH_MAX_LINE_GAP` | `10` px | Maximum collinear gap. |
| `_ANGLE_THRESHOLD` | `3.0°` | Max angle difference to merge segments. |
| `_MIDPOINT_THRESHOLD` | `50.0` px | Max midpoint distance to merge segments. |
| `_SANITY_MAX_DELTA` | `20.0°` | Max inter-frame angle jump before rejection. |

---

## Choosing Between `HeadingDetector` and `LineProcessor`

| Feature | `HeadingDetector` | `LineProcessor` |
|---------|-------------------|-----------------|
| Output | Heading error `e = \|θ−90°\|` | Raw angle θ |
| Line grouping | No | Yes |
| Reference selection | No (averages all lines) | Yes (nearest tile gap) |
| Sanity check | No | Yes |
| Complexity | Low | Higher |
| Used by | `control.HeadingController` | `control.ServoPID` (via `main.py`) |
