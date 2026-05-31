**Utilities API Specification**

Purpose: extract the stable, reusable utility API surface from the legacy codebase so a new core system can plug into existing configuration, logging, visualization, streaming and helper modules.

Sections:
- **Configuration**
- **Logging / CSV**
- **Visualization / Debug Overlays**
- **Streaming**
- **Math / Helpers**
- **Data Schemas**
- **Minimal Usage Examples**

---

**Configuration**

- Module: `config.settings`

Public functions (helpers used internally but safe to call):

- `_get_str(name: str, default: str) -> str`
- `_get_int(name: str, default: int) -> int`
- `_get_float(name: str, default: float) -> float`
- `_get_bool(name: str, default: bool) -> bool`

Well-known environment-backed constants (name -> default value):

- MAIN_TARGET_HZ -> 30.0
- MAIN_CAMERA_INDEX -> 0
- MAIN_CSV_LOG_FILE -> "run_logs.csv"
- MAIN_FLIP_FRAME -> False
- MAIN_TERMINAL_LOG -> True
- MAIN_DEBUG_MODE -> False
- MAIN_SHOW_PREVIEW -> False
- MAIN_SHOW_GUIDANCE_OVERLAY -> True
- MAIN_SHOW_DETECTOR_DEBUG -> False
- MAIN_WRITE_DEBUG_VIDEO -> False
- MAIN_DEBUG_VIDEO_OUTPUT -> "main_debug.mp4"
- MAIN_VIDEO_OUTPUT_FPS -> 20.0
- MAIN_DEBUG_FRAME_SCALE -> 1.25
- MAIN_DEBUG_OVERLAY_SCALE -> 0.75
- MAIN_CAMERA_RETRY_LIMIT -> 3
- MAIN_VIDEO_RETRY_LIMIT -> 5
- MAIN_HARDWARE_RETRY_LIMIT -> 5
- MAIN_HTTPS_STREAM_ENABLED -> False
- MAIN_HTTPS_STREAM_HOST -> "127.0.0.1"
- MAIN_HTTPS_STREAM_PORT -> 8443
- MAIN_HTTPS_STREAM_PUBLIC -> False
- MAIN_HTTPS_STREAM_PATH -> "/stream.mjpg"
- MAIN_HTTPS_SNAPSHOT_PATH -> "/snapshot.jpg"
- MAIN_HTTPS_STATUS_PATH -> "/status"
- MAIN_HTTPS_TOKEN -> ""
- MAIN_HTTPS_CERT_FILE -> "certs/main_stream_cert.pem"
- MAIN_HTTPS_KEY_FILE -> "certs/main_stream_key.pem"
- MAIN_HTTPS_SELF_SIGNED_DAYS -> 365

- PROCESS_VIDEO_CSV_OUTPUT -> "video_log.csv"
- PROCESS_VIDEO_OUTPUT -> "processed_video.mp4"
- PROCESS_VIDEO_SEND_TO_SERVO -> True
- PROCESS_VIDEO_TERMINAL_LOG -> False
- PROCESS_VIDEO_SHOW_GUIDANCE_OVERLAY -> False
- PROCESS_VIDEO_SHOW_DETECTOR_DEBUG -> False
- PROCESS_VIDEO_START_CALIB_THRESHOLD_DEG -> 5.0
- PROCESS_VIDEO_STOP_CALIB_THRESHOLD_DEG -> 3.0
- PROCESS_VIDEO_FLIP_FRAME -> False
- PROCESS_VIDEO_FRAME_SLEEP_MS -> 0.0

- PID_KP -> 1.0
- PID_KI -> 0.05
- PID_KD -> 0.1
- PID_CALIBRATION_CLEAR_TOLERANCE_DEG -> 1.0
- SERVO_CENTER_ANGLE -> 90.0
- MAX_STEERING_OFFSET -> 30.0
- ROI_HEIGHT_PCT -> 0.6
- ROI_TOP_WIDTH_PCT -> 0.75
- ROI_BOTTOM_WIDTH_PCT -> 1.0
- ROBOT_DEBUG_MODE -> False

- Vision tunables (examples):
  - VISION_CLAHE_CLIP_LIMIT -> 2.0
  - VISION_CLAHE_TILE_GRID_W/H -> 8
  - VISION_BLUR_KERNEL_{W,H} -> 5
  - VISION_CANNY_LOW/HIGH -> 50/150
  - Various VISION_HOUGH_*, VISION_CLUSTER_*, VISION_MIN_GROUP_TOTAL_LENGTH_PX, etc.

Notes: The settings module exposes all runtime configuration as module-level constants loaded from environment/.env using python-dotenv. A new system should import the required constants directly from `config.settings`.

---

**Logging / CSV**

- Module: `runtime.video_runtime_helpers`

Public API (signatures):

- `init_csv_logger(path: str, fieldnames: list[str]) -> tuple[csv.DictWriter, TextIO]`
  - Opens (or appends to) CSV file at `path`, ensures header is present if file was created, returns `(writer, fileobj)`.

- `print_progress(frame_num: int, total_frames: int) -> None`
  - In-place terminal progress bar (no return).

- `init_video(path: str, logger: logging.Logger) -> tuple[cv2.VideoCapture, float, int, int, int]`
  - Opens a video file and returns `(capture, fps, frame_count, width, height)`.

- `init_video_writer(path: str, fps: float, width: int, height: int) -> cv2.VideoWriter`
  - Create MP4 writer; may raise RuntimeError on failure.

- `init_live_video_writer(path: str, fps: float, width: int, height: int) -> tuple[cv2.VideoWriter, str]`
  - Returns a timestamped writer and the resolved path `(writer, resolved_path)`.

- `configure_terminal_logging(enabled: bool) -> None`
  - Toggle INFO vs ERROR visibility for root logger output to terminal.

- `init_camera(index: int) -> cv2.VideoCapture`
  - Open and validate a camera device; raises `RuntimeError` on failure.

- `init_camera_with_retries(index: int, retries: int, logger: logging.Logger) -> cv2.VideoCapture`
  - Open a camera with bounded retries and logged warnings; raises `RuntimeError` on failure.

- `sleep_remainder(loop_start: float, loop_period: float, logger: logging.Logger) -> None`
  - Sleep for remaining time of a periodic loop; logs overruns.

- `build_process_video_arg_parser(...) -> argparse.ArgumentParser`
  - Builds CLI parser used by `process_video` entrypoint (see runtime.video_runtime_helpers for full signature).

- `build_main_arg_parser(...) -> argparse.ArgumentParser`
  - Builds CLI parser used by `main` realtime entrypoint (see runtime.video_runtime_helpers for full signature).

Usage contract for CSV rows (Main loop):
- The main realtime CSV uses fieldnames (ordered):
  - frame_num, mono_timestamp, utc_timestamp, loop_ms, loop_overrun_ms,
    fsm_state, calibration_active, theta, theta_source, theta_for_overlay,
    theta_horizontal, reference_group_index, selected_group_bbox,
    lines_count, groups_count, horizontal_ok, sanity_ok, stale_output,
    servo_angle, servo_center_angle, servo_offset, pid_error, pid_p_term,
    pid_i_term, pid_d_term, pid_integral, pid_last_error,
    hardware_send_latency_ms, stream_enabled, stream_host, stream_port

- The process-video CSV uses fieldnames: frame_num, timestamp, fsm_state, theta, servo_angle, pid_integral, pid_last_error

Notes: `init_csv_logger` returns a `csv.DictWriter` which callers use via `writer.writerow(dict)` followed by `file.flush()`.

---

**Visualization / Debug Overlays**

- Module: `runtime.video_runtime_helpers`

Public functions (signatures):

- `maybe_flip_frame(frame: np.ndarray, flip_frame: bool) -> np.ndarray`
  - Returns the (optionally) flipped frame.

- `draw_overlay(
    frame: np.ndarray,
    frame_num: int,
    theta: float | None,
    theta_for_overlay: float | None,
    servo_angle: float,
    servo_center_angle: float,
    fsm_state: str,
    show_guidance_overlay: bool,
    start_calib_threshold_deg: float,
    stop_calib_threshold_deg: float,
    overlay_scale: float = 1.0,
) -> np.ndarray`
  - Renders textual and gauge overlays onto a BGR frame and returns the annotated frame.

- `build_detector_debug_panel(frame_width: int, panel_height: int, detector_debug: dict[str, Any]) -> np.ndarray`
  - Creates a compact 2x2 debug panel composed from detector stage images and metadata.

Contract for `detector_debug` dict (as produced by `vision.detector.LineDetector.get_reference_angle_debug`):
- Expected keys (examples, types):
  - "gray": np.ndarray (grayscale image)
  - "roi": np.ndarray (grayscale masked ROI)
  - "preprocessed": np.ndarray
  - "edges": np.ndarray (binary edge map)
  - "hough_vis": np.ndarray (BGR image with raw lines drawn)
  - "grouped_vis": np.ndarray (BGR image with grouped lines drawn)
  - "lines_count": int
  - "groups_count": int
  - "reference_group_index": int | None
  - "selected_group_bbox": tuple[int,int,int,int] | None
  - "selected_lines": list[tuple[int,int,int,int]] | None
  - "theta_horizontal": float | None
  - "theta_candidate": float | None
  - "horizontal_ok": bool
  - "sanity_ok": bool
  - "theta_output": float | None
  - "stale_output": bool
  - Additional numeric tunables mirrored from `config.settings` may also be present.

Notes: `selected_lines` is the opposing lane-line pair selected by `vision.detector.LineDetector` and is used by the new HUD overlay.

---

**Streaming**

- Module: `runtime.https_stream`

Public classes and functions (signatures):

- `@dataclass SharedFrameStore(jpeg_bytes: Optional[bytes] = None, timestamp_unix: Optional[float] = None, telemetry: dict[str, Any] | None = None)`
  - Methods:
    - `set_frame(frame_bgr: Any, telemetry: dict[str, Any]) -> None`
      - Encodes BGR frame to JPEG and stores bytes, unix timestamp and telemetry under a lock.
    - `snapshot() -> tuple[Optional[bytes], Optional[float], dict[str, Any] | None]`
      - Return a consistent snapshot of `(jpeg_bytes, timestamp_unix, telemetry)`.

- `class HttpsMjpegServer` (constructor):
  - `__init__(self, host: str, port: int, stream_path: str, snapshot_path: str, status_path: str, token: str, cert_file: str, key_file: str, frame_store: SharedFrameStore) -> None`
  - Methods:
    - `start() -> None`  # starts uvicorn server thread
    - `stop() -> None`   # shutdown
    - `stream_url() -> str`
    - `status_url() -> str`
    - `snapshot_url() -> str`

- `ensure_self_signed_cert(cert_file: str, key_file: str, host: str, valid_days: int) -> None`
  - Ensures TLS cert + key exist and are loadable; generates self-signed pair using cryptography if missing.

Contract: `HttpsMjpegServer` reads JPEG bytes and telemetry from a `SharedFrameStore` instance; the server exposes `/status`, `/snapshot`, `/stream` endpoints with optional `token` query parameter.

---

**Math / Helpers**

- Detector-side helper (pure math):

- `_angle_diff(a: float, b: float) -> float`
  - Returns minimum angular difference in degrees (handles 0°/180° wrap), range [0,90].

Notes: Many detector helpers (line grouping, _segment_props, _weighted_angle, etc.) are implemented as methods inside `vision.detector.LineDetector`. The only public-facing API to use is `get_reference_angle` and `get_reference_angle_debug` (see below). If callers need lower-level access they may reimplement or import internal helpers; these are not exported from a stable utilities module in the legacy code.

---

**Detector (Vision) API**

- Module: `vision.detector`

Public class (signature):

- `class LineDetector:`
  - `__init__(self, state: models.robot_state.RobotState) -> None`
  - `get_reference_angle(self, frame: np.ndarray) -> Optional[float]`
    - Runs full pipeline and returns `theta` in degrees `[0,180)` or `None` if no valid detection.
  - `get_reference_angle_debug(self, frame: np.ndarray) -> tuple[Optional[float], dict[str, Any]]`
    - Returns `(theta, debug_data)` where `debug_data` contains intermediate images and metadata described earlier.

Behavioral notes: `LineDetector` uses `RobotState` ROI parameters and `debug_mode` flag. The outputs are purely diagnostic (images, numbers) and do not mutate external state (they mutate internal detector last-angle only). Use `get_reference_angle` for production and `get_reference_angle_debug` to obtain `detector_debug` dictionary consumed by `build_detector_debug_panel` and CSV logging.

---

**Models / Data Structures**

- Module: `models.robot_state`

Dataclasses and fields (contracts):

- `@dataclass PIDConstants`:
  - `kp: float` (default from `PID_KP`)
  - `ki: float` (default from `PID_KI`)
  - `kd: float` (default from `PID_KD`)

- `@dataclass RobotState`:
  - `pid: PIDConstants`
  - `servo_center_angle: float`
  - `max_steering_offset: float`
  - `last_valid_servo_angle: float`
  - `last_valid_command: float`
  - `roi_height_pct: float`
  - `roi_top_width_pct: float`
  - `roi_bottom_width_pct: float`
  - `debug_mode: bool`
  - `fsm_state: FSMState` (enum: SEARCHING, LOCKED, GAPPING)
  - `calibration_active: bool`
  - `pid_integral: float`
  - `pid_last_error: float`
  - Methods:
    - `transition_to(self, new_state: FSMState) -> None`
    - `reset_pid_integral(self) -> None`

Notes: The `RobotState` instance is the canonical configuration holder (ROI fractions, servo centre) and the recommended place to read tuning parameters for vision and control. The new core should instantiate and populate a `RobotState` (or a compatible shim) and pass it to `LineDetector`.

---

**Data Schemas**

- `Main CSV row` (keys and types):
  - frame_num: int
  - mono_timestamp: str (float formatted string)
  - utc_timestamp: str (ISO 8601)
  - loop_ms: str (float formatted string)
  - loop_overrun_ms: str
  - fsm_state: str
  - calibration_active: int (0/1)
  - theta: str (float formatted) | ""
  - theta_source: str ("live"/"stale"/"none")
  - theta_for_overlay: str | ""
  - theta_horizontal: str | ""
  - reference_group_index: int | ""
  - selected_group_bbox: str ("x,y,w,h" or "")
  - lines_count: int | ""
  - groups_count: int
  - horizontal_ok: bool | ""
  - sanity_ok: bool | ""
  - stale_output: bool | ""
  - servo_angle: str (float formatted)
  - servo_center_angle: str
  - servo_offset: str
  - pid_error, pid_p_term, pid_i_term, pid_d_term, pid_integral, pid_last_error: strings (floats)
  - hardware_send_latency_ms: str
  - stream_enabled: int (0/1)
  - stream_host: str
  - stream_port: int | ""

- `Process-video CSV row` (keys and types):
  - frame_num: int
  - timestamp: str (float formatted)
  - fsm_state: str
  - theta: str | ""
  - servo_angle: str
  - pid_integral: str
  - pid_last_error: str

---

**Minimal Usage Examples**

- Initialize CSV logger and write a row (main-like):

```python
from runtime.video_runtime_helpers import init_csv_logger
writer, f = init_csv_logger("out.csv", ["frame_num","theta"])
writer.writerow({"frame_num": 1, "theta": "90.00"})
f.flush()
```

- Build and render overlay for a frame:

```python
from runtime.video_runtime_helpers import draw_overlay
annotated = draw_overlay(frame, 1, theta, last_theta, servo_angle, 90.0, "SEARCHING", True, 5.0, 3.0)
```

- Stream frames to the HTTPS MJPEG server:

```python
from runtime.https_stream import SharedFrameStore, HttpsMjpegServer, ensure_self_signed_cert
store = SharedFrameStore()
ensure_self_signed_cert("cert.pem","key.pem","127.0.0.1",365)
server = HttpsMjpegServer("127.0.0.1",8443,"/stream.mjpg","/snapshot.jpg","/status","", "cert.pem","key.pem", store)
server.start()
# To publish a frame: store.set_frame(bgr_frame, {"frame_num": 1})
```

---

Notes & Recommendations for integration:
- Treat `config.settings` as the single source of runtime tunables. Import values by name.
- Use `init_csv_logger` and the documented CSV fieldnames when producing logs compatible with existing post-processing/visualization scripts.
- Use `LineDetector.get_reference_angle` for production detection; call `get_reference_angle_debug` only when you also consume the `detector_debug` dict in a debug panel or log.
- The streaming and frame-store APIs are thread-safe and intentionally minimal: publish BGR frames via `SharedFrameStore.set_frame()` and let the server expose them.

End of specification.
