"""MiniPC input controller — ported process_controls() priority logic.

Runs on MiniPC, reads gamepad/keyboard state from InputDeviceHandler,
publishes control decisions (base, steer, relay) via the driver layer.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from config.settings import (
    BUTTON_CENTER_MINUS,
    BUTTON_CENTER_PLUS,
    BUTTON_CRUISE,
    BUTTON_IMU_MODE,
    BUTTON_LOCK,
    BUTTON_QUIT,
    BUTTON_REMOTE_STEER_ONLY,
    BUTTON_STOP,
    BUTTON_UNLOCK,
    CRUISE_DURATION_S,
    DRIVE_AXIS,
    GAMEPAD_DRIVE_DEADZONE,
    GAMEPAD_STEER_DEADZONE,
    HAT_Y_AXIS,
    INVERT_DRIVE_AXIS,
    INVERT_STEER_AXIS,
    LEFT_LIMIT,
    MANUAL_STEER_HOLD,
    RELAY_BLINK_INTERVAL_S,
    RIGHT_LIMIT,
    SERVO_CENTER_ANGLE,
    SERVO_MAX_ANGLE_DEG,
    SERVO_STEP,
    SQUARE_STRAIGHT_DURATION,
    SQUARE_TURN_DURATION,
    STEER_AXIS,
    STEER_DEADBAND_DEG,
)

if TYPE_CHECKING:
    from runtime.gamepad_handler import InputDeviceHandler

logger = logging.getLogger(__name__)

_RELAY_BLINK_HOLD_THRESHOLD = 0.3


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _apply_deadzone(value: float, deadzone: float) -> float:
    return 0.0 if abs(value) < deadzone else value


@dataclass
class ControlDecision:
    base_command: str | None = None
    steer_angle: float | None = None  # None = use vision PID
    relay_command: str | None = None
    steer_source: str = "VISION"


class InputController:
    """Central control logic — same priority as RPi process_controls()."""

    def __init__(
        self,
        handler: InputDeviceHandler,
        center_angle: float = SERVO_CENTER_ANGLE,
        max_angle_deg: float = SERVO_MAX_ANGLE_DEG,
        step: float = SERVO_STEP,
        manual_steer_hold: float = MANUAL_STEER_HOLD,
        steer_deadband_deg: float = STEER_DEADBAND_DEG,
        steer_axis: int = STEER_AXIS,
        drive_axis: int = DRIVE_AXIS,
        hat_y_axis: int = HAT_Y_AXIS,
        gamepad_steer_deadzone: float = GAMEPAD_STEER_DEADZONE,
        gamepad_drive_deadzone: float = GAMEPAD_DRIVE_DEADZONE,
        invert_steer_axis: bool = INVERT_STEER_AXIS,
        invert_drive_axis: bool = INVERT_DRIVE_AXIS,
        button_stop: int = BUTTON_STOP,
        button_imu_mode: int = BUTTON_IMU_MODE,
        button_lock: int = BUTTON_LOCK,
        button_unlock: int = BUTTON_UNLOCK,
        button_remote_steer_only: int = BUTTON_REMOTE_STEER_ONLY,
        button_quit: int = BUTTON_QUIT,
        button_cruise: int = BUTTON_CRUISE,
        button_center_plus: int | None = BUTTON_CENTER_PLUS,
        button_center_minus: int = BUTTON_CENTER_MINUS,
        cruise_duration_s: float = CRUISE_DURATION_S,
        square_straight_duration: float = SQUARE_STRAIGHT_DURATION,
        square_turn_duration: float = SQUARE_TURN_DURATION,
        relay_blink_interval: float = RELAY_BLINK_INTERVAL_S,
    ) -> None:
        self._h = handler
        self._center_angle = center_angle
        self._steer_angle = center_angle
        self._max_angle_deg = max_angle_deg
        self._step = step
        self._manual_steer_hold = manual_steer_hold
        self._steer_deadband_deg = steer_deadband_deg
        self._steer_axis = steer_axis
        self._drive_axis = drive_axis
        self._hat_y_axis = hat_y_axis
        self._gamepad_steer_deadzone = gamepad_steer_deadzone
        self._gamepad_drive_deadzone = gamepad_drive_deadzone
        self._invert_steer_axis = invert_steer_axis
        self._invert_drive_axis = invert_drive_axis
        self._button_stop = button_stop
        self._button_imu_mode = button_imu_mode
        self._button_lock = button_lock
        self._button_unlock = button_unlock
        self._button_remote_steer_only = button_remote_steer_only
        self._button_quit = button_quit
        self._button_cruise = button_cruise
        self._button_center_plus = button_center_plus
        self._button_center_minus = button_center_minus
        self._cruise_duration_s = cruise_duration_s
        self._square_straight_duration = square_straight_duration
        self._square_turn_duration = square_turn_duration
        self._relay_blink_interval = relay_blink_interval

        # mutable state
        self.left_limit = self._center_angle - self._max_angle_deg
        self.right_limit = self._center_angle + self._max_angle_deg
        self.manual_override_until = 0.0
        self.manual_override_source: str | None = None
        self.controller_remote_steer_only = False
        self.last_controller_remote_steer_only = False
        self.cruise_active = False
        self.cruise_start_time = 0.0
        self.cruise_prev_remote_steer_only = False
        self.square_pattern_active = False
        self.square_phase = "straight"
        self.square_phase_start = 0.0
        self.relay_on = False
        self.relay_blink_active = False
        self.relay_rb_press_time = 0.0
        self.relay_last_blink_time = 0.0
        self._last_base_command: str | None = None
        self._last_relay_command: str | None = None
        self._last_steer_angle: float | None = None
        self._last_steer_source: str | None = None

        # edge-triggered button processing state
        self._prev_pressed_buttons: set[int] = set()
        self._prev_pressed_keys: set[str] = set()

    # ------------------------------------------------------------------
    # main entry point — call once per main loop iteration
    # ------------------------------------------------------------------

    def process(self, now: float) -> ControlDecision:
        """Run control logic and return a ControlDecision."""
        self._process_edge_triggers(now)
        base_decision = self._keyboard_base()
        base_command = base_decision.base_command if base_decision is not None else None

        # --- cruise timeout ---
        if self.cruise_active and (now - self.cruise_start_time) >= self._cruise_duration_s:
            self.cruise_active = False
            self.controller_remote_steer_only = self.cruise_prev_remote_steer_only
            logger.info("CRUISE: finished (%.0fs elapsed)", self._cruise_duration_s)
            return ControlDecision(base_command="STOP", steer_source="CRUISE-STOP")

        # --- square pattern ---
        if self.square_pattern_active:
            return self._square_pattern(now)

        # --- relay blink (RB hold) ---
        if self._button_unlock in self._h.pressed_buttons and self.relay_rb_press_time > 0:
            if (now - self.relay_rb_press_time) > _RELAY_BLINK_HOLD_THRESHOLD:
                self.relay_blink_active = True
                if (now - self.relay_last_blink_time) >= self._relay_blink_interval:
                    self.relay_last_blink_time = now
                    cmd = "OFF" if self.relay_on else "ON"
                    self.relay_on = not self.relay_on
                    self._last_relay_command = cmd
                    return ControlDecision(relay_command=cmd)

        # --- cruise active ---
        if self.cruise_active:
            return self._cruise_steer(now)

        # --- keyboard steer ---
        ks = self._keyboard_steer(now)
        if ks is not None:
            ks.base_command = base_command
            return ks

        # --- manual override hold ---
        if now < self.manual_override_until:
            return ControlDecision(base_command=base_command, steer_angle=self._steer_angle, steer_source="HOLD")

        # --- gamepad steer ---
        if not self.controller_remote_steer_only:
            gs = self._gamepad_steer(now)
            if gs is not None:
                gs.base_command = base_command
                return gs

        # --- remote / idle ---
        return ControlDecision(base_command=base_command, steer_angle=None, steer_source="VISION")

    # ------------------------------------------------------------------
    # square pattern state machine
    # ------------------------------------------------------------------

    def _square_pattern(self, now: float) -> ControlDecision:
        elapsed = now - self.square_phase_start
        if self.square_phase == "straight":
            if elapsed >= self._square_straight_duration:
                self.square_phase = "right_turn"
                self.square_phase_start = now
                logger.info("SQUARE: turn right (%.1fs)", self._square_turn_duration)
            # straight: forward, vision steer (None = use PID)
            return ControlDecision(base_command="FORWARD", steer_angle=None, steer_source="SQUARE-MQTT")
        else:
            if elapsed >= self._square_turn_duration:
                self.square_phase = "straight"
                self.square_phase_start = now
                logger.info("SQUARE: straight (%.1fs, MQTT steer)", self._square_straight_duration)
            return ControlDecision(base_command="FORWARD", steer_angle=self.right_limit, steer_source="SQUARE-TURN")

    # ------------------------------------------------------------------
    # cruise steer
    # ------------------------------------------------------------------

    def _cruise_steer(self, now: float) -> ControlDecision:
        # Cruise auto-drives forward; steer from vision (None)
        return ControlDecision(base_command="FORWARD", steer_angle=None, steer_source="CRUISE")

    # ------------------------------------------------------------------
    # keyboard base (W/S/X/L/U)
    # ------------------------------------------------------------------

    def _keyboard_base(self) -> ControlDecision | None:
        pk = self._h.pressed_keys
        if "KEY_L" in pk:
            return ControlDecision(base_command="LOCK", steer_source="KEYBOARD")
        if "KEY_U" in pk:
            return ControlDecision(base_command="UNLOCK", steer_source="KEYBOARD")

        has_w = "KEY_W" in pk
        has_s = "KEY_S" in pk
        has_x = "KEY_X" in pk
        kb_active = has_x or has_w or has_s

        if has_x or (has_w and has_s):
            return ControlDecision(base_command="STOP", steer_source="KEYBOARD")
        if has_w:
            return ControlDecision(base_command="FORWARD", steer_source="KEYBOARD")
        if has_s:
            return ControlDecision(base_command="BACKWARD", steer_source="KEYBOARD")

        # gamepad base
        drive = self._h.axis_state.get(self._drive_axis, 0.0)
        if self._invert_drive_axis:
            drive = -drive
        drive = _apply_deadzone(drive, self._gamepad_drive_deadzone)

        if self._button_lock in self._h.pressed_buttons:
            return ControlDecision(base_command="LOCK", steer_source="GAMEPAD")
        if self._button_stop in self._h.pressed_buttons:
            return ControlDecision(base_command="STOP", steer_source="GAMEPAD")
        if drive < 0:
            return ControlDecision(base_command="FORWARD", steer_source="GAMEPAD")
        if drive > 0:
            return ControlDecision(base_command="BACKWARD", steer_source="GAMEPAD")
        if kb_active:
            return ControlDecision(base_command="STOP", steer_source="KEYBOARD")
        return ControlDecision(base_command="STOP", steer_source="IDLE")

    # ------------------------------------------------------------------
    # keyboard steer (D/C)
    # ------------------------------------------------------------------

    def _keyboard_steer(self, now: float) -> ControlDecision | None:
        pk = self._h.pressed_keys
        has_d = "KEY_D" in pk
        has_c = "KEY_C" in pk

        # log remote-only mode transitions
        remote = self.controller_remote_steer_only
        if remote != self.last_controller_remote_steer_only:
            self.last_controller_remote_steer_only = remote
            logger.info("MODE: %s", "drive local, steer=MQTT" if remote else "steer+drive local from controller")

        if has_c:
            self._activate_manual_override("KEYBOARD", now)
            self._steer_angle = self._center_angle
            return ControlDecision(steer_angle=self._center_angle, steer_source="KEYBOARD")
        if has_d:
            self._activate_manual_override("KEYBOARD", now)
            self._steer_angle = _clamp(self._steer_angle - self._step, self.left_limit, self.right_limit)
            return ControlDecision(steer_angle=self._steer_angle, steer_source="KEYBOARD")
        return None

    # ------------------------------------------------------------------
    # gamepad steer (right stick X)
    # ------------------------------------------------------------------

    def _gamepad_steer(self, now: float) -> ControlDecision | None:
        axis = self._h.axis_state.get(self._steer_axis, 0.0)
        if self._invert_steer_axis:
            axis = -axis
        axis = _apply_deadzone(axis, self._gamepad_steer_deadzone)
        if axis == 0.0:
            return None

        if axis < 0:
            target = self._center_angle - axis * (self.right_limit - self._center_angle)
        else:
            target = self._center_angle - axis * (self._center_angle - self.left_limit)

        target = _clamp(target, self.left_limit, self.right_limit)
        if self._last_steer_angle is not None and abs(target - self._last_steer_angle) < self._steer_deadband_deg:
            return ControlDecision(steer_angle=target, steer_source="GAMEPAD")

        self._last_steer_angle = target
        self._steer_angle = target
        self._activate_manual_override("GAMEPAD", now)
        return ControlDecision(steer_angle=target, steer_source="GAMEPAD")

    # ------------------------------------------------------------------
    # edge-triggered button actions
    # ------------------------------------------------------------------

    def _process_edge_triggers(self, now: float) -> None:
        """Process buttons that fire on press (not hold)."""
        self._process_button_edges(now)
        self._process_key_edges(now)

    def _process_button_edges(self, now: float) -> None:
        current = self._h.pressed_buttons
        for code in current - self._prev_pressed_buttons:
            if code == self._button_remote_steer_only:
                self.controller_remote_steer_only = not self.controller_remote_steer_only
                logger.info("MODE: remote steer = %s", self.controller_remote_steer_only)
            elif code == self._button_imu_mode:
                self.square_pattern_active = not self.square_pattern_active
                if self.square_pattern_active:
                    self.square_phase = "straight"
                    self.square_phase_start = now
                    logger.info("SQUARE: ON (straight %.1fs → turn %.1fs)", self._square_straight_duration, self._square_turn_duration)
                else:
                    logger.info("SQUARE: OFF")
            elif code == self._button_cruise:
                if self.cruise_active:
                    self.cruise_active = False
                    self.controller_remote_steer_only = self.cruise_prev_remote_steer_only
                    logger.info("CRUISE: cancelled")
                else:
                    self.cruise_active = True
                    self.cruise_start_time = now
                    self.cruise_prev_remote_steer_only = self.controller_remote_steer_only
                    self.controller_remote_steer_only = True
                    logger.info("CRUISE: started (%.0fs)", self._cruise_duration_s)
            elif self._button_center_plus is not None and code == self._button_center_plus:
                self._adjust_center(1)
            elif code == self._button_center_minus:
                self._adjust_center(-1)
            elif code == self._button_quit:
                import os
                os._exit(0)
            elif code == self._button_unlock:
                self.relay_rb_press_time = now

        # RB release: tap vs blink-end
        released = self._prev_pressed_buttons - current
        for code in released:
            if code == self._button_unlock:
                if self.relay_blink_active:
                    self.relay_blink_active = False
                    self.relay_on = False
                    self._last_relay_command = "OFF"
                    logger.info("RELAY: OFF (blink stop)")
                elif (now - self.relay_rb_press_time) < _RELAY_BLINK_HOLD_THRESHOLD:
                    self.relay_on = not self.relay_on
                    cmd = "ON" if self.relay_on else "OFF"
                    self._last_relay_command = cmd
                    logger.info("RELAY: %s (toggle)", cmd)
                self.relay_rb_press_time = 0.0

        self._prev_pressed_buttons = current.copy()

    def _process_key_edges(self, now: float) -> None:
        current = self._h.pressed_keys
        for key in current - self._prev_pressed_keys:
            if key == "KEY_1":
                self._adjust_center(1)
            elif key == "KEY_2":
                self._adjust_center(-1)
            elif key == "KEY_Q":
                import os
                os._exit(0)
            elif key == "KEY_A":
                self.relay_on = not self.relay_on
                cmd = "ON" if self.relay_on else "OFF"
                self._last_relay_command = cmd
                logger.info("RELAY: %s (keyboard toggle)", cmd)
            elif key == "KEY_ENTER":
                if self.cruise_active:
                    self.cruise_active = False
                    self.controller_remote_steer_only = self.cruise_prev_remote_steer_only
                    logger.info("CRUISE: cancelled (keyboard)")
                else:
                    self.cruise_active = True
                    self.cruise_start_time = now
                    self.cruise_prev_remote_steer_only = self.controller_remote_steer_only
                    self.controller_remote_steer_only = True
                    logger.info("CRUISE: started (keyboard, %.0fs)", self._cruise_duration_s)

        self._prev_pressed_keys = current.copy()

    # ------------------------------------------------------------------
    # center angle adjustment (with D-pad integration)
    # ------------------------------------------------------------------

    def _adjust_center(self, delta: float) -> None:
        self._center_angle = _clamp(self._center_angle + delta, self.left_limit, self.right_limit)
        self.left_limit = self._center_angle - self._max_angle_deg
        self.right_limit = self._center_angle + self._max_angle_deg
        logger.info("CENTER_ANGLE: %.1f", self._center_angle)

    # ------------------------------------------------------------------
    # manual override timing
    # ------------------------------------------------------------------

    def _activate_manual_override(self, source: str, now: float) -> None:
        self.manual_override_until = now + self._manual_steer_hold
        self.manual_override_source = source
