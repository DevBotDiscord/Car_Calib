#!/usr/bin/env python3
"""Raspberry Pi control-direct entrypoint.

Runs current direct calibration/dashboard loop with pigpio hardware backend.
"""

from __future__ import annotations

import os

os.environ.setdefault("CONTROL_HARDWARE", "pigpio")

from main_jetson import main


if __name__ == "__main__":
    main()
