"""RPi control logic — gamepad/keyboard → base/servo/relay + MQTT control signals."""

from __future__ import annotations

import logging
from dataclasses import dataclass

from . import config
from .mqtt_client import publish_mode, publish_route_control

logger = logging.getLogger(__name__)

_RELAY_BLINK_HOLD_THRESHOLD = 0.3


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _apply_deadzone(value: float, deadzone: float) -> float:
    return 0.0 if abs(value) < deadzone else value


@dataclass
class ControlDecision:
    base_command: str | None = None
    manual_steer: bool = False
    steer_angle: float = 0.0
    relay_command: str | None = None
    route_command: str | None = None
    mode_command: str | None = None


class InputController:
    """RPi control logic — same priority as original MiniPC process_controls()."""

    def __init__(self, handler) -> None:
        self._h = handler
        self._center_angle = config.CENTER_ANGLE
        self._steer_angle = config.CENTER_ANGLE
        self._step = config.SERVO_STEP
        self._manual_steer_hold = config.MANUAL_STEER_HOLD
        self._steer_deadband_deg = config.STEER_DEADBAND_DEG
        self._gamepad_steer_deadzone = config.GAMEPAD_STEER_DEADZONE
        self._gamepad_drive_deadzone = config.GAMEPAD_DRIVE_DEADZONE
        self._cruise_duration_s = config.CRUISE_DURATION_S
        self._square_straight_duration = config.SQUARE_STRAIGHT_DURATION
        self._square_turn_duration = config.SQUARE_TURN_DURATION
        self._relay_blink_interval = config.RELAY_BLINK_INTERVAL_S

        self.left_limit = config.LEFT_LIMIT
        self.right_limit = config.RIGHT_LIMIT
        self.manual_override_until = 0.0
        self.controller_remote_steer_only = False
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
        self._last_steer_angle: float | None = None

        self._prev_pressed_buttons: set[int] = set()
        self._prev_pressed_keys: set[str] = set()

        # Auto-return to home after a turn ends.
        self._steering_active = False
        self._recenter_pending = False

    def process(self, now: float) -> ControlDecision:
        """Run control logic and return a ControlDecision."""
        self._process_edge_triggers(now)
        base_decision = self._keyboard_base()
        base_command = base_decision.base_command if base_decision is not None else None

        # Cruise timeout
        if self.cruise_active and (now - self.cruise_start_time) >= self._cruise_duration_s:
            self.cruise_active = False
            self.controller_remote_steer_only = self.cruise_prev_remote_steer_only
            self._last_steer_angle = None
            logger.info("CRUISE: finished")
            return ControlDecision(base_command="STOP", route_command="STOP")

        # Square pattern
        if self.square_pattern_active:
            return self._square_pattern(now, base_command)

        # Relay blink (RB hold)
        if config.BUTTON_UNLOCK in self._h.pressed_buttons and self.relay_rb_press_time > 0:
            if (now - self.relay_rb_press_time) > _RELAY_BLINK_HOLD_THRESHOLD:
                self.relay_blink_active = True
                if (now - self.relay_last_blink_time) >= self._relay_blink_interval:
                    self.relay_last_blink_time = now
                    cmd = "OFF" if self.relay_on else "ON"
                    self.relay_on = not self.relay_on
                    return ControlDecision(relay_command=cmd)

        # Cruise active
        if self.cruise_active:
            return ControlDecision(base_command="FORWARD", manual_steer=False)

        # Keyboard steer
        ks = self._keyboard_steer(now)
        if ks is not None:
            ks.base_command = base_command
            return ks

        # Gamepad steer (runs first so new input clears any pending recenter)
        if not self.controller_remote_steer_only:
            gs = self._gamepad_steer(now)
            if gs is not None:
                gs.base_command = base_command
                return gs

        # Auto-recenter after steering release (sticky until next steer input)
        if self._recenter_pending:
            self._steer_angle = self._center_angle
            self.manual_override_until = 0.0
            return ControlDecision(base_command=base_command, manual_steer=True, steer_angle=self._center_angle)

        # Manual override hold
        if now < self.manual_override_until:
            return ControlDecision(base_command=base_command, manual_steer=True, steer_angle=self._steer_angle)

        # Remote / idle
        return ControlDecision(base_command=base_command, manual_steer=False)

    def _square_pattern(self, now: float, base_command: str | None) -> ControlDecision:
        elapsed = now - self.square_phase_start
        if self.square_phase == "straight":
            if elapsed >= self._square_straight_duration:
                self.square_phase = "right_turn"
                self.square_phase_start = now
                logger.info("SQUARE: turn right")
            return ControlDecision(base_command="FORWARD", manual_steer=False)
        else:
            if elapsed >= self._square_turn_duration:
                self.square_phase = "straight"
                self.square_phase_start = now
                logger.info("SQUARE: straight")
            return ControlDecision(base_command="FORWARD", manual_steer=True, steer_angle=self.right_limit)

    def _keyboard_base(self) -> ControlDecision | None:
        pk = self._h.pressed_keys
        if "KEY_L" in pk:
            return ControlDecision(base_command="LOCK")
        if "KEY_U" in pk:
            return ControlDecision(base_command="UNLOCK")

        has_w = "KEY_W" in pk
        has_s = "KEY_S" in pk
        has_x = "KEY_X" in pk

        if has_x or (has_w and has_s):
            return ControlDecision(base_command="STOP")
        if has_w:
            return ControlDecision(base_command="FORWARD")
        if has_s:
            return ControlDecision(base_command="BACKWARD")

        # Gamepad base
        drive = self._h.axis_state.get(config.DRIVE_AXIS, 0.0)
        if config.INVERT_DRIVE_AXIS:
            drive = -drive
        drive = _apply_deadzone(drive, self._gamepad_drive_deadzone)

        if config.BUTTON_LOCK in self._h.pressed_buttons:
            return ControlDecision(base_command="LOCK")
        if config.BUTTON_STOP in self._h.pressed_buttons:
            return ControlDecision(base_command="STOP")
        if drive < 0:
            return ControlDecision(base_command="FORWARD")
        if drive > 0:
            return ControlDecision(base_command="BACKWARD")
        return ControlDecision(base_command="STOP")

    def _keyboard_steer(self, now: float) -> ControlDecision | None:
        pk = self._h.pressed_keys
        has_d = "KEY_D" in pk
        has_c = "KEY_C" in pk

        if has_c:
            self._activate_manual_override(now)
            self._steer_angle = self._center_angle
            self._clear_recenter_on_new_input()
            self._mark_steering_active(now)
            return ControlDecision(manual_steer=True, steer_angle=self._center_angle)
        if has_d:
            self._activate_manual_override(now)
            self._steer_angle = _clamp(self._steer_angle - self._step, self.left_limit, self.right_limit)
            self._clear_recenter_on_new_input()
            self._mark_steering_active(now)
            return ControlDecision(manual_steer=True, steer_angle=self._steer_angle)
        return None

    def _gamepad_steer(self, now: float) -> ControlDecision | None:
        axis = self._h.axis_state.get(config.STEER_AXIS, 0.0)
        if config.INVERT_STEER_AXIS:
            axis = -axis
        axis = _apply_deadzone(axis, self._gamepad_steer_deadzone)
        if axis == 0.0:
            self._mark_steering_idle(now)
            return None

        if axis < 0:
            target = self._center_angle - axis * (self.right_limit - self._center_angle)
        else:
            target = self._center_angle - axis * (self._center_angle - self.left_limit)

        target = _clamp(target, self.left_limit, self.right_limit)
        self._clear_recenter_on_new_input()
        if self._last_steer_angle is not None and abs(target - self._last_steer_angle) < self._steer_deadband_deg:
            self._mark_steering_active(now)
            return ControlDecision(manual_steer=True, steer_angle=target)

        self._last_steer_angle = target
        self._steer_angle = target
        self._activate_manual_override(now)
        self._mark_steering_active(now)
        return ControlDecision(manual_steer=True, steer_angle=target)

    def _mark_steering_active(self, now: float) -> None:
        self._steering_active = True
        self._recenter_pending = False

    def _mark_steering_idle(self, now: float) -> None:
        if self._steering_active:
            self._steering_active = False
            self._recenter_pending = True
            logger.info("AUTO_RECENTER: pending center hold")

    def _clear_recenter_on_new_input(self) -> None:
        self._recenter_pending = False
        self.manual_override_until = 0.0

    def _process_edge_triggers(self, now: float) -> None:
        """Process buttons that fire on press (not hold)."""
        self._process_button_edges(now)
        self._process_key_edges(now)

    def _process_button_edges(self, now: float) -> None:
        current = self._h.pressed_buttons
        for code in current - self._prev_pressed_buttons:
            print(f"BTN_DEBUG: pressed code={code}")
            if code == config.BUTTON_REMOTE_STEER:
                if self.cruise_active:
                    logger.info("REMOTE_STEER toggle ignored during CRUISE")
                else:
                    self.controller_remote_steer_only = not self.controller_remote_steer_only
                    mode = "REMOTE_STEER" if self.controller_remote_steer_only else "AUTO"
                    publish_mode(mode)
                    logger.info("MODE: %s", mode)
            elif code == config.BUTTON_SQUARE:
                self.square_pattern_active = not self.square_pattern_active
                if self.square_pattern_active:
                    if self.cruise_active:
                        self.cruise_active = False
                        self.controller_remote_steer_only = self.cruise_prev_remote_steer_only
                        self._last_steer_angle = None
                    self.square_phase = "straight"
                    self.square_phase_start = now
                    publish_mode("SQUARE")
                    publish_route_control("START")
                    logger.info("SQUARE: ON")
                else:
                    self._last_steer_angle = None
                    publish_mode("AUTO")
                    publish_route_control("STOP")
                    logger.info("SQUARE: OFF")
            elif code == config.BUTTON_CRUISE:
                if self.cruise_active:
                    self.cruise_active = False
                    self.controller_remote_steer_only = self.cruise_prev_remote_steer_only
                    self._last_steer_angle = None
                    publish_mode("AUTO")
                    publish_route_control("STOP")
                    logger.info("CRUISE: cancelled")
                else:
                    if self.square_pattern_active:
                        self.square_pattern_active = False
                        self._last_steer_angle = None
                        publish_mode("AUTO")
                        publish_route_control("STOP")
                    self.cruise_active = True
                    self.cruise_start_time = now
                    self.cruise_prev_remote_steer_only = self.controller_remote_steer_only
                    self.controller_remote_steer_only = True
                    publish_mode("CRUISE")
                    publish_route_control("START")
                    logger.info("CRUISE: started")
            elif code == config.BUTTON_CENTER_MINUS:
                self._adjust_center(-1)
            elif code == config.BUTTON_QUIT:
                import os
                os._exit(0)
            elif code == config.BUTTON_UNLOCK:
                self.relay_rb_press_time = now

        # RB release: tap vs blink-end
        released = self._prev_pressed_buttons - current
        for code in released:
            if code == config.BUTTON_UNLOCK:
                if self.relay_blink_active:
                    self.relay_blink_active = False
                    self.relay_on = False
                    logger.info("RELAY: OFF (blink stop)")
                elif (now - self.relay_rb_press_time) < _RELAY_BLINK_HOLD_THRESHOLD:
                    self.relay_on = not self.relay_on
                    logger.info("RELAY: %s (toggle)", "ON" if self.relay_on else "OFF")
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
            elif key == "KEY_ENTER":
                if self.cruise_active:
                    self.cruise_active = False
                    self.controller_remote_steer_only = self.cruise_prev_remote_steer_only
                    self._last_steer_angle = None
                    publish_mode("AUTO")
                    publish_route_control("STOP")
                    logger.info("CRUISE: cancelled (keyboard)")
                else:
                    if self.square_pattern_active:
                        self.square_pattern_active = False
                        self._last_steer_angle = None
                        publish_mode("AUTO")
                        publish_route_control("STOP")
                    self.cruise_active = True
                    self.cruise_start_time = now
                    self.cruise_prev_remote_steer_only = self.controller_remote_steer_only
                    self.controller_remote_steer_only = True
                    publish_mode("CRUISE")
                    publish_route_control("START")
                    logger.info("CRUISE: started (keyboard)")

        self._prev_pressed_keys = current.copy()

    def _adjust_center(self, delta: float) -> None:
        self._center_angle = _clamp(self._center_angle + delta, self.left_limit, self.right_limit)
        self.left_limit = self._center_angle - config.SERVO_MAX_ANGLE_DEG
        self.right_limit = self._center_angle + config.SERVO_MAX_ANGLE_DEG
        logger.info("CENTER_ANGLE: %.1f", self._center_angle)

    def _activate_manual_override(self, now: float) -> None:
        self.manual_override_until = now + self._manual_steer_hold
