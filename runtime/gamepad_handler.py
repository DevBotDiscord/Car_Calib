"""MiniPC gamepad/keyboard handler — evdev input reading ported from RPi bridge."""

from __future__ import annotations

import logging
from typing import Any

from evdev import InputDevice, ecodes, list_devices

from scripts.input_device_helpers import (
    find_optional_abs_input_device,
    open_optional_input_device,
)

logger = logging.getLogger(__name__)


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _apply_deadzone(value: float, deadzone: float) -> float:
    return 0.0 if abs(value) < deadzone else value


class InputDeviceHandler:
    """Reads gamepad + keyboard via evdev, tracks axis/button/key state."""

    def __init__(
        self,
        keyboard_device_path: str = "",
        gamepad_device_path: str = "",
        gamepad_name_hints: tuple[str, ...] = (),
        steer_axis: int = ecodes.ABS_RX,
        drive_axis: int = ecodes.ABS_Y,
        hat_y_axis: int = ecodes.ABS_HAT0Y,
    ) -> None:
        self._keyboard_path = keyboard_device_path
        self._gamepad_path = gamepad_device_path
        self._gamepad_name_hints = gamepad_name_hints
        self._steer_axis = steer_axis
        self._drive_axis = drive_axis
        self._hat_y_axis = hat_y_axis

        self.keyboard: InputDevice | None = None
        self.gamepad: InputDevice | None = None
        self.pressed_keys: set[str] = set()
        self.pressed_buttons: set[int] = set()
        self.axis_state: dict[int, float] = {}
        self.hat_state: dict[int, int] = {}

    # ------------------------------------------------------------------
    # setup / teardown
    # ------------------------------------------------------------------

    def setup(self) -> None:
        self._setup_keyboard()
        self._setup_gamepad()
        self.axis_state = {self._steer_axis: 0.0, self._drive_axis: 0.0}
        self.hat_state = {self._hat_y_axis: 0}
        if self.keyboard is not None:
            try:
                self.keyboard.grab()
            except OSError:
                pass
        if self.gamepad is not None:
            try:
                self.gamepad.grab()
            except OSError:
                pass

    def close(self) -> None:
        for dev in (self.keyboard, self.gamepad):
            if dev is None:
                continue
            try:
                dev.ungrab()
            except OSError:
                pass
            try:
                dev.close()
            except OSError:
                pass
        self.keyboard = None
        self.gamepad = None

    # ------------------------------------------------------------------
    # poll — read all pending evdev events, non-blocking
    # ------------------------------------------------------------------

    def poll(self) -> None:
        """Call once per main loop iteration (~30 Hz)."""
        for dev in (self.keyboard, self.gamepad):
            if dev is None:
                continue
            try:
                events = dev.read()
            except (BlockingIOError, OSError):
                continue
            for event in events:
                if dev is self.keyboard:
                    self._handle_key_event(event)
                elif dev is self.gamepad:
                    self._handle_gamepad_event(event)

    # ------------------------------------------------------------------
    # internal event handlers (ported from controls.py)
    # ------------------------------------------------------------------

    def _handle_key_event(self, event: Any) -> None:
        if event.type != ecodes.EV_KEY:
            return
        keycode = ecodes.KEY.get(event.code)
        if not keycode:
            return
        keys = keycode if isinstance(keycode, list) else [keycode]
        if event.value in (1, 2):
            for key in keys:
                self.pressed_keys.add(key)
        elif event.value == 0:
            for key in keys:
                self.pressed_keys.discard(key)

    def _handle_gamepad_event(self, event: Any) -> None:
        if event.type == ecodes.EV_KEY:
            self._handle_gamepad_button(event)
        elif event.type == ecodes.EV_ABS:
            self._handle_gamepad_axis(event)

    def _handle_gamepad_button(self, event: Any) -> None:
        if event.value == 1:
            self.pressed_buttons.add(event.code)
        elif event.value == 0:
            self.pressed_buttons.discard(event.code)

    def _handle_gamepad_axis(self, event: Any) -> None:
        code = event.code
        if code in (self._steer_axis, self._drive_axis):
            self.axis_state[code] = self._normalize_axis(code, event.value)
        elif code == self._hat_y_axis and event.value != self.hat_state.get(self._hat_y_axis):
            self.hat_state[self._hat_y_axis] = event.value

    def _normalize_axis(self, axis_code: int, raw_value: int) -> float:
        if self.gamepad is None:
            return 0.0
        try:
            info = self.gamepad.absinfo(axis_code)
        except Exception:
            return 0.0
        minimum, maximum = info.min, info.max
        center = (minimum + maximum) / 2.0
        half_range = (maximum - minimum) / 2.0
        if half_range <= 0:
            return 0.0
        return _clamp((raw_value - center) / half_range, -1.0, 1.0)

    # ------------------------------------------------------------------
    # setup helpers
    # ------------------------------------------------------------------

    def _setup_keyboard(self) -> None:
        if not self._keyboard_path:
            logger.info("INPUT: no keyboard device configured, skipping")
            return
        self.keyboard = open_optional_input_device(
            self._keyboard_path,
            log=logger.info,
            device_factory=InputDevice,
        )

    def _setup_gamepad(self) -> None:
        self.gamepad = find_optional_abs_input_device(
            self._gamepad_path if self._gamepad_path else "",
            log=logger.info,
            device_factory=InputDevice,
            list_devices_fn=list_devices,
            name_hints=self._gamepad_name_hints if self._gamepad_name_hints else ("edra", "joystick", "gamepad", "controller", "pad"),
            ev_abs_code=ecodes.EV_ABS,
        )

    # ------------------------------------------------------------------
    # convenience
    # ------------------------------------------------------------------

    @property
    def has_keyboard(self) -> bool:
        return self.keyboard is not None

    @property
    def has_gamepad(self) -> bool:
        return self.gamepad is not None
