"""CLI for generating calibration evaluation datasets from recorded video."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from calibration_evaluation import CalibrationEvaluator, iter_video_frames


def build_parser() -> argparse.ArgumentParser:
    """Build calibration evaluation CLI parser."""
    parser = argparse.ArgumentParser(description="Evaluate calibration against a recorded video")
    parser.add_argument("--input", required=True, help="Input video path")
    parser.add_argument("--output-dir", required=True, help="Evaluation artifact directory")
    parser.add_argument(
        "--start-frame",
        type=int,
        default=0,
        help="First evaluated frame; skipped frames do not warm up controller state",
    )
    parser.add_argument("--max-frames", type=int)
    parser.add_argument(
        "--stride",
        type=int,
        default=1,
        help="Evaluate every Nth frame; use 1 for steering/controller comparisons",
    )
    parser.add_argument(
        "--review-every",
        type=int,
        default=0,
        help="Save a labeled review panel every N evaluated frames; 0 disables periodic panels",
    )
    parser.add_argument(
        "--no-review-missing",
        action="store_false",
        dest="review_missing",
        help="Do not save panels for frames without an observation",
    )
    parser.add_argument(
        "--no-review-errors",
        action="store_false",
        dest="review_errors",
        help="Do not save panels for processing errors",
    )
    parser.add_argument(
        "--max-review-panels",
        type=int,
        default=200,
        help="Maximum review images to save; use a negative value for no limit",
    )
    parser.set_defaults(review_missing=True, review_errors=True)
    return parser


def main() -> None:
    """Run calibration evaluation and print its summary."""
    args = build_parser().parse_args()
    evaluator = CalibrationEvaluator(
        output_dir=Path(args.output_dir),
        review_every=args.review_every,
        review_missing=args.review_missing,
        review_errors=args.review_errors,
        max_review_panels=None if args.max_review_panels < 0 else args.max_review_panels,
    )
    try:
        _records, summary = evaluator.evaluate_frames(
            iter_video_frames(
                args.input,
                start_frame=args.start_frame,
                max_frames=args.max_frames,
                stride=args.stride,
            )
        )
    finally:
        evaluator.close()
    print(json.dumps(summary.as_dict(), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
