"""Latched safety state machine for Marvin hardware teleoperation."""

from __future__ import annotations

import enum
import time
from dataclasses import dataclass

import numpy as np

from xrobotoolkit_teleop.common.marvin_types import MarvinJointCommand, MarvinRobotState


class MarvinControlState(enum.Enum):
    DISCONNECTED = "disconnected"
    READ_ONLY = "read_only"
    ARMED = "armed"
    TELEOP = "teleop"
    RETURNING = "returning"
    HOLD = "hold"
    FAULT = "fault"
    SHUTDOWN = "shutdown"


@dataclass(frozen=True)
class MarvinSafetyConfig:
    xr_hold_ms: float = 100.0
    xr_fault_ms: float = 500.0
    feedback_hold_ms: float = 30.0
    feedback_fault_ms: float = 100.0
    command_validity_ms: float = 40.0
    tracking_error_hold_rad: float = np.deg2rad(3.0)
    tracking_error_fault_rad: float = np.deg2rad(8.0)

    def __post_init__(self):
        pairs = (
            (self.xr_hold_ms, self.xr_fault_ms, "XR"),
            (self.feedback_hold_ms, self.feedback_fault_ms, "feedback"),
            (
                self.tracking_error_hold_rad,
                self.tracking_error_fault_rad,
                "tracking error",
            ),
        )
        if self.command_validity_ms <= 0.0:
            raise ValueError("command_validity_ms must be positive")
        for hold, fault, label in pairs:
            if hold <= 0.0 or fault <= hold:
                raise ValueError(f"{label} fault threshold must exceed its positive hold threshold")


@dataclass(frozen=True)
class MarvinSafetyDecision:
    state: MarvinControlState
    send_command: bool
    use_feedback_hold: bool
    request_idle: bool
    reason: str


class MarvinSafetySupervisor:
    def __init__(self, config=MarvinSafetyConfig()):
        self.config = config
        self.state = MarvinControlState.DISCONNECTED
        self.reason = "not connected"
        self.transitions = []
        self._rearm_required = False

    def _transition(self, state, reason):
        if state != self.state or reason != self.reason:
            self.transitions.append(
                {
                    "monotonic_ns": time.monotonic_ns(),
                    "from": self.state.value,
                    "to": state.value,
                    "reason": reason,
                }
            )
        self.state = state
        self.reason = reason

    def connected_read_only(self):
        if self.state not in (MarvinControlState.DISCONNECTED, MarvinControlState.READ_ONLY):
            raise RuntimeError(f"cannot enter read-only from {self.state.value}")
        self._transition(MarvinControlState.READ_ONLY, "fresh read-only feedback")

    def arm(self):
        if self.state not in (
            MarvinControlState.READ_ONLY,
            MarvinControlState.HOLD,
            MarvinControlState.ARMED,
        ):
            raise RuntimeError(f"cannot arm from {self.state.value}")
        self._transition(MarvinControlState.ARMED, "hardware configuration verified")
        self._rearm_required = False

    def fault(self, reason):
        self._transition(MarvinControlState.FAULT, reason)
        return MarvinSafetyDecision(MarvinControlState.FAULT, False, False, True, reason)

    def hold(self, reason):
        if self.state == MarvinControlState.FAULT:
            return MarvinSafetyDecision(
                MarvinControlState.FAULT, False, False, True, self.reason
            )
        self._rearm_required = True
        self._transition(MarvinControlState.HOLD, reason)
        return MarvinSafetyDecision(MarvinControlState.HOLD, True, True, False, reason)

    def reset_fault_to_read_only(self):
        if self.state != MarvinControlState.FAULT:
            raise RuntimeError("fault reset is only valid from FAULT")
        self._transition(MarvinControlState.READ_ONLY, "manual fault reset")

    def shutdown(self):
        self._transition(MarvinControlState.SHUTDOWN, "shutdown requested")

    def _decision(self, state, reason):
        if state == MarvinControlState.FAULT:
            self._transition(state, reason)
            return MarvinSafetyDecision(state, False, False, True, reason)
        if state == MarvinControlState.HOLD:
            self._rearm_required = True
            self._transition(state, reason)
            return MarvinSafetyDecision(state, True, True, False, reason)
        self._transition(state, reason)
        return MarvinSafetyDecision(state, True, False, False, reason)

    def evaluate(
        self,
        robot_state: MarvinRobotState | None,
        command: MarvinJointCommand | None,
        xr_source_age_ms: float,
        now_ns=None,
    ):
        if now_ns is None:
            now_ns = time.monotonic_ns()
        if self.state in (MarvinControlState.DISCONNECTED, MarvinControlState.READ_ONLY):
            return MarvinSafetyDecision(self.state, False, False, False, self.reason)
        if self.state == MarvinControlState.SHUTDOWN:
            return MarvinSafetyDecision(self.state, False, False, True, self.reason)
        if self.state == MarvinControlState.FAULT:
            return MarvinSafetyDecision(self.state, False, False, True, self.reason)

        if robot_state is None:
            return self._decision(MarvinControlState.FAULT, "no robot feedback")
        if any(code != 0 for code in robot_state.error_code):
            return self._decision(
                MarvinControlState.FAULT,
                f"robot error codes {robot_state.error_code}",
            )
        if any(state == 100 for state in robot_state.arm_state):
            return self._decision(MarvinControlState.FAULT, "robot reported error state 100")
        if robot_state.arm_state != (3, 3):
            return self._decision(
                MarvinControlState.FAULT,
                f"robot left joint impedance state: {robot_state.arm_state}",
            )

        feedback_age_ms = robot_state.age_ms(now_ns)
        if feedback_age_ms > self.config.feedback_fault_ms:
            return self._decision(
                MarvinControlState.FAULT,
                f"feedback stale for {feedback_age_ms:.1f} ms",
            )
        if feedback_age_ms > self.config.feedback_hold_ms:
            return self._decision(
                MarvinControlState.HOLD,
                f"feedback age {feedback_age_ms:.1f} ms exceeded HOLD threshold",
            )

        if command is None:
            return self._decision(MarvinControlState.HOLD, "no valid joint command")
        command_age_ms = command.age_ms(now_ns)
        if command_age_ms > self.config.command_validity_ms:
            return self._decision(
                MarvinControlState.HOLD,
                f"command stale for {command_age_ms:.1f} ms",
            )

        active = any(command.active_arms)
        if self._rearm_required:
            if active:
                return self._decision(
                    MarvinControlState.HOLD,
                    "release all Grip controls before resuming after HOLD",
                )
            self._rearm_required = False
            return self._decision(MarvinControlState.ARMED, "HOLD cleared with Grip released")
        if active:
            if not np.isfinite(xr_source_age_ms) or xr_source_age_ms > self.config.xr_fault_ms:
                return self._decision(
                    MarvinControlState.FAULT,
                    f"XR source stale for {xr_source_age_ms:.1f} ms",
                )
            if xr_source_age_ms > self.config.xr_hold_ms:
                return self._decision(
                    MarvinControlState.HOLD,
                    f"XR source age {xr_source_age_ms:.1f} ms exceeded HOLD threshold",
                )

        tracking_error = float(np.max(np.abs(command.q_rad - robot_state.q_rad)))
        if tracking_error > self.config.tracking_error_fault_rad:
            return self._decision(
                MarvinControlState.FAULT,
                f"joint tracking error {np.rad2deg(tracking_error):.2f} deg",
            )
        if tracking_error > self.config.tracking_error_hold_rad:
            return self._decision(
                MarvinControlState.HOLD,
                f"joint tracking error {np.rad2deg(tracking_error):.2f} deg",
            )

        if active:
            return self._decision(MarvinControlState.TELEOP, "fresh active command")
        if any(command.returning_arms):
            return self._decision(
                MarvinControlState.RETURNING,
                "Grip released; returning to configured initial pose",
            )
        return self._decision(MarvinControlState.ARMED, "Grip released; holding feedback")
