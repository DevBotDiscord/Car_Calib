"""Tests for runtime.video_runtime_helpers."""

from __future__ import annotations

import importlib
import sys
import types

import numpy as np


def _load_helpers():
    fake_cv2 = types.SimpleNamespace(
        flip=lambda frame, mode: np.flip(frame, axis=1 if mode == 1 else 0),
    )
    sys.modules["cv2"] = fake_cv2
    sys.modules.pop("runtime.video_runtime_helpers", None)
    return importlib.import_module("runtime.video_runtime_helpers")


def test_maybe_flip_frame_mirrors_horizontally():
    helpers = _load_helpers()
    frame = np.array(
        [
            [[1, 0, 0], [2, 0, 0], [3, 0, 0]],
            [[4, 0, 0], [5, 0, 0], [6, 0, 0]],
        ],
        dtype=np.uint8,
    )

    flipped = helpers.maybe_flip_frame(frame, True)

    assert flipped.tolist() == [
        [[3, 0, 0], [2, 0, 0], [1, 0, 0]],
        [[6, 0, 0], [5, 0, 0], [4, 0, 0]],
    ]
