# Vision Detector API

Module: `vision/detector.py`

## Public API

- `_angle_diff(a: float, b: float) -> float`
  - Returns minimum heading difference in `[0, 90]` with 180-degree wrap handling.

- `class LineDetector`
  - `__init__(self, state: models.robot_state.RobotState) -> None`
  - `get_reference_angle(self, frame: np.ndarray) -> Optional[float]`
  - `get_reference_angle_debug(self, frame: np.ndarray) -> tuple[Optional[float], dict[str, Any]]`

## Debug Contract

`get_reference_angle_debug` returns a dictionary including stage images and telemetry keys such as:

- `gray`, `roi`, `preprocessed`, `edges`, `hough_vis`, `grouped_vis`
- `lines_count`, `groups_count`, `reference_group_index`, `selected_group_bbox`
- `selected_lines`
- `theta_horizontal`, `theta_candidate`, `horizontal_ok`, `sanity_ok`, `theta_output`, `stale_output`

`selected_lines` contains the opposing lane-line pair selected by `LineDetector` and is used by the HUD overlay.
