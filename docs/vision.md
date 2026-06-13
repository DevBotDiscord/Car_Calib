# Vision Processing

Vision calibration is exposed through `UnifiedCalibrator`.

`VisionProcessor.process_frame_debug(frame)` performs:

1. Empty-frame handling.
2. Top ROI crop using `ROI_HEIGHT_PCT`.
3. Grayscale conversion.
4. Gaussian blur.
5. Canny edge detection.
6. Probabilistic Hough line extraction.
7. Generation of visualization intermediates.

`VisionProcessor._apply_geometric_filter(lines)` currently rejects vertical
and near-horizontal segments and selects the longest negative-slope and
positive-slope lines. The method remains private because pair-selection
contracts will be extracted in a later approved stage-separation gate.

The approved geometry refinement is to choose the most opposite valid slopes,
with explicit rejection reasons and tests. It is not part of the current
centralization gate.

Current debug data includes grayscale, ROI, blurred image, edges, Hough
visualization, selected-line visualization, raw line count, and selected lines.
The outer `vision_debug` key is consumed by existing overlay helpers.

Selected-pair diagnostics explicitly expose the current negative-slope line as
`selected_left_line` (`LEFT / NEG`) and the positive-slope line as
`selected_right_line` (`RIGHT / POS`). Each line has endpoints, slope, length,
and its own projected bottom intercept. This naming describes the current image
geometry and will be revalidated during direction-convention refinement.

The raw vanishing point remains unbounded. A point above, below, or horizontally
outside the frame keeps its original coordinates and receives a `vp_location`
classification rather than being clamped or hidden.
