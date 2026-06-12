# Runtime Video Helpers API

Module: `runtime/video_runtime_helpers.py`

## Logging / CSV

- `init_csv_logger(path: str, fieldnames: list[str]) -> tuple[csv.DictWriter, TextIO]`
- `print_progress(frame_num: int, total_frames: int) -> None`

## Video / Camera

- `init_video(path: str, logger: logging.Logger) -> tuple[cv2.VideoCapture, float, int, int, int]`
- `init_video_writer(path: str, fps: float, width: int, height: int) -> cv2.VideoWriter`
- `init_live_video_writer(path: str, fps: float, width: int, height: int) -> tuple[cv2.VideoWriter, str]`
- `init_camera(index: int) -> cv2.VideoCapture`
- `init_camera_with_retries(index: int, retries: int, logger: logging.Logger) -> cv2.VideoCapture`

## Runtime Control

- `configure_terminal_logging(enabled: bool) -> None`
- `sleep_remainder(loop_start: float, loop_period: float, logger: logging.Logger) -> None`
- `build_process_video_arg_parser(...) -> argparse.ArgumentParser`
- `build_main_arg_parser(...) -> argparse.ArgumentParser`

## Visualization

- `maybe_flip_frame(frame: np.ndarray, flip_frame: bool) -> np.ndarray`
- `draw_overlay(...) -> np.ndarray`
- `build_vision_debug_panel(frame_width: int, panel_height: int, vision_debug: dict[str, Any]) -> np.ndarray`

## Notes

- `draw_overlay(...)` is the legacy guidance overlay helper and remains available as a fallback.
- The newer offline HUD path lives in `overlay_drawer.OverlayDrawer` and is used by `TelemetryLogger.update_visuals(...)`.
