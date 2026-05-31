## 1. Keep vs. Replace Matrix

This matrix defines the boundaries of our refactor. We are surgically replacing the mathematical calculation layers while keeping the entire outer wrapper of the environmental, visual, and streaming utilities intact.

| Domain | Replaced (Legacy Architecture) | Kept (Legacy Utilities API) |
| --- | --- | --- |
| **Control Logic** | Dual controllers, conflicting heading/centering math | `RobotState`, `PIDConstants`, Target Loop Hz (`MAIN_TARGET_HZ`) |
| **Vision Extractor** | Legacy horizontal/vertical angle heuristics | The ROI parameters and camera retry limits (`MAIN_CAMERA_RETRY_LIMIT`) |
| **Debugging / Visuals** | Ad-hoc print statements and hardcoded shapes | All `MAIN_SHOW_*` toggles, `MAIN_DEBUG_MODE`, `draw_overlay`, `build_detector_debug_panel` |
| **Data & Video Logging** | Legacy redundant `print()` statements | `init_csv_logger`, `init_video_writer`, `MAIN_WRITE_DEBUG_VIDEO`, Main CSV row schemas |
| **Streaming** | None | `SharedFrameStore`, `HttpsMjpegServer`, all `MAIN_HTTPS_*` config variables |

---

## 2. System Flow Outline

Data moves through the new architecture in a strict, Single-Input, Single-Output (SISO) linear flow to ensure hardware safety:

1. **Input:** Raw BGR camera frame is captured by the main loop.
2. **Stage 1 (Initialization):** The `VisionProcessor` crops the top half of the frame (ROI), runs Canny/HoughP, and applies a Geometric Filter to isolate the two optimal lane lines (opposing slopes).
3. **Stage 3 Safety Check (Danger Zone):** The `GeometryCalculator` computes the bottom $x$-intercepts of the detected lines. The `SteeringController` checks these against `DANGER_MARGIN_PX`. If a boundary is crossed, a fixed safety nudge is output, bypassing Stage 2.
4. **Stage 2 Tracking (Normal Ops):** If the robot is safely inside the margins, the `GeometryCalculator` calculates the Vanishing Point ($x_{vp}, y_{vp}$). The `SteeringController` converts this to $\theta_{raw}$, applies the Hysteresis bounds (`VP_INNER_THRESH` / `VP_OUTER_THRESH`), and outputs a smoothed PD angle.
5. **Output & Telemetry:** The final computed servo angle is sent to the hardware API. Simultaneously, the `TelemetryLogger` renders the HUD through `OverlayDrawer`, records the metrics to the CSV, conditionally writes to the local debug MP4, and pushes the frame to the `SharedFrameStore`.

---

## 3. Class Definitions & Interfaces

### A. `ConfigManager`

**Responsibility:** Acts as the comprehensive registry for all tuning, debugging, and system-level `.env` parameters, keeping the math layers clean while providing toggles to the outer loop.

* **Core Method Signatures:**
* `__init__(self) -> None`
* `get_system_configs(self) -> dict` (Returns `MAIN_TARGET_HZ`, `MAIN_CAMERA_INDEX`, `MAIN_FLIP_FRAME`)
* `get_debug_configs(self) -> dict` (Returns `MAIN_DEBUG_MODE`, `MAIN_SHOW_PREVIEW`, `MAIN_SHOW_GUIDANCE_OVERLAY`)
* `get_video_configs(self) -> dict` (Returns `MAIN_WRITE_DEBUG_VIDEO`, `MAIN_DEBUG_VIDEO_OUTPUT`, `MAIN_VIDEO_OUTPUT_FPS`)
* `get_stream_configs(self) -> dict` (Returns all `MAIN_HTTPS_*` constants)
* `get_vp_thresholds(self) -> tuple[float, float]` (Returns Inner, Outer thresholds)
* `get_danger_margins(self) -> tuple[int, float]` (Returns Pixels, Nudge Deg)


* **Integration Points:**
* Pulls directly from `config.settings._get_bool()`, `_get_float()`, and `_get_str()`.



### B. `VisionProcessor`

**Responsibility:** Handles raw image manipulation, ROI cropping, and executes Stage 1 (Initialization).

* **Core Method Signatures:**
* `__init__(self, roi_height_pct: float) -> None`
* `process_frame(self, frame: np.ndarray) -> list[tuple[int, int, int, int]]` (Returns raw Hough lines)
* `_apply_geometric_filter(self, lines: list) -> tuple[tuple, tuple] | None` (Finds the two opposing slope lines)


* **Integration Points:**
* Reads ROI constraints inherited from `models.robot_state.RobotState`.



### C. `GeometryCalculator`

**Responsibility:** Pure mathematical calculation layer (Stateless). Responsible for intersection, perspective mapping, and providing coordinates for the visualizer.

* **Core Method Signatures:**
* `calculate_vanishing_point(line1: tuple, line2: tuple) -> tuple[int, int] | None`
* `calculate_bottom_intercepts(line1: tuple, line2: tuple, frame_height: int) -> tuple[int, int]`
* `map_vp_to_angle(vp_x: int, frame_width: int) -> float`: Evaluates $\theta_{raw} = \frac{180}{\text{frame\_width}} \cdot x_{vp}$



### D. `SteeringController`

**Responsibility:** The core governor. Implements Stage 2 and Stage 3 logic, managing the PD loop, hysteresis thresholds, and danger fallbacks.

* **Core Method Signatures:**
* `__init__(self, pid_constants: PIDConstants, danger_margin: int, nudge_deg: float, inner_thresh: float, outer_thresh: float) -> None`
* `compute_steering(self, vp_angle: float | None, left_intercept: int | None, right_intercept: int | None, frame_width: int) -> tuple[float, str]` (Returns final angle and state string)
* `_apply_pd(self, error: float) -> float`


* **Integration Points:**
* Uses `PIDConstants` data structure.



### E. `TelemetryLogger`

**Responsibility:** Bridges the new module's data to the legacy visualization, CSV infrastructure, and streaming, explicitly handling the new algorithm's visual markers.

* **Core Method Signatures:**
* `__init__(self, config: ConfigManager) -> None`
* `log_state(self, frame_num: int, telemetry_data: dict) -> None`
* `update_visuals(self, frame: np.ndarray, telemetry_data: dict, debug_data: dict) -> np.ndarray` (Renders the HUD via `OverlayDrawer`, including telemetry, danger zones, lane-line coloring, vanishing-point marker, and the hysteresis gauge; falls back to the legacy overlay helper if needed).
* `publish_stream(self, frame: np.ndarray, telemetry_data: dict) -> None` (Pushes to `SharedFrameStore` if `MAIN_HTTPS_STREAM_ENABLED` is true).


* **Integration Points:**
* `runtime.video_runtime_helpers.init_csv_logger`
* `overlay_drawer.OverlayDrawer`
* `runtime.video_runtime_helpers.draw_overlay`
* `runtime.video_runtime_helpers.build_detector_debug_panel`
* `runtime.video_runtime_helpers.init_video_writer`
* `runtime.https_stream.SharedFrameStore`



### F. `UnifiedCalibrator` (The Facade)

**Responsibility:** Orchestrates the system and enforces the legacy timing loops.

* **Core Method Signatures:**
* `__init__(self, config: ConfigManager) -> None`
* `run(self) -> None` (The continuous loop that opens the camera using `init_camera_with_retries`, captures the frame, passes it to the `update()` method, and triggers `_manage_loop_timing()`).
* `update(self, frame: np.ndarray, frame_num: int) -> float` (Takes frame, executes the pipeline, returns exact servo angle to be dispatched to motors).
* `_manage_loop_timing(self, loop_start_time: float) -> None`


* **Integration Points:**
* Calls `runtime.video_runtime_helpers.sleep_remainder` using `MAIN_TARGET_HZ` to ensure consistent loop execution.

---

## 5. Offline Video Path

`process_video.py` now drives the same frame-processing pipeline as the live loop and writes the annotated frame to the debug MP4 output.

Key behavior:

* The CLI configures the shared runtime for offline processing and debug video output.
* `UnifiedCalibrator.update(...)` stores the rendered frame after telemetry overlay generation.
* The saved MP4 includes the telemetry panel, danger zones, dashed center line, lane coloring, vanishing-point crosshair, and the angle gauge.
* The CSV output remains aligned with the main telemetry schema.

---
---

## 4. State Machine Blueprint

The internal state flow is determined dynamically per-frame within the `SteeringController`, adhering strictly to API constraints (no speed control, steering only).

### **State 1: Vision Lost (Gapping)**

* **Condition:** `VisionProcessor` returns `None` (lines lost or geometric filter fails).
* **Action:** Coast safely. Bypass PD loop entirely.
* **Output:** Lock steering to **90°**.

### **State 2: Danger Zone (Bottom Intercept Override)**

* **Condition:** Lines exist. `GeometryCalculator` reports bottom intercepts crossing `DANGER_MARGIN_PX`.
* **Action:** Override VP logic. Governor limit applied.
* **Sub-state A (Left Danger):** Left $x$-intercept > `left_margin` (e.g., $x > 100$). Output **95°** (Rightward nudge).
* **Sub-state B (Right Danger):** Right $x$-intercept < `right_margin` (e.g., $x < 540$). Output **85°** (Leftward nudge).


* **Output:** Hold fixed nudge angle until bounds are cleared.

### **State 3: Tracking (VP Hysteresis)**

* **Condition:** Lines exist AND bottom intercepts are safely inside margins.
* **Action:** Evaluate Vanishing Point angle against Hysteresis rules.
* **Inner Threshold (Coasting):** If $|\theta_{raw} - 90| \le \text{VP\_INNER\_THRESH}$. Action: Do not update PD. Output **90°**.
* **Outer Threshold (Trigger):** If $|\theta_{raw} - 90| > \text{VP\_OUTER\_THRESH}$. Action: Activate PD calculation. Continue outputting PD value until the angle crosses back into the Inner Threshold.


* **Output:** Proportional-Derivative dynamic angle.
