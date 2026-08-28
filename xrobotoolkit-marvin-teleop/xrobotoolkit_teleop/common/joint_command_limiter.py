"""Feedback-aware joint command limiting for simulation and hardware backends."""

from __future__ import annotations

import numpy as np


def _positive_vector(value, length, name):
    array = np.asarray(value, dtype=float)
    if array.ndim == 0:
        array = np.full(length, float(array))
    array = array.reshape(-1)
    if array.size != length or not np.all(np.isfinite(array)) or np.any(array <= 0.0):
        raise ValueError(f"{name} must be positive finite scalar or length-{length} vector")
    return array.copy()


class FeedbackAwareJointLimiter:
    """Limit q targets using the command history plus measured q/dq braking state."""

    def __init__(
        self,
        lower_limits,
        upper_limits,
        max_velocity,
        max_acceleration,
        dt,
        limit_margin=0.0,
        max_jerk=None,
        target_natural_frequency=8.0,
    ):
        self.lower_limits = np.asarray(lower_limits, dtype=float).reshape(-1)
        self.upper_limits = np.asarray(upper_limits, dtype=float).reshape(-1)
        if self.lower_limits.shape != self.upper_limits.shape:
            raise ValueError("lower_limits and upper_limits must have the same shape")
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        if limit_margin < 0.0:
            raise ValueError("limit_margin must be non-negative")
        self.dt = float(dt)
        self.size = self.lower_limits.size
        self.max_velocity = _positive_vector(max_velocity, self.size, "max_velocity")
        self.max_acceleration = _positive_vector(
            max_acceleration, self.size, "max_acceleration"
        )
        self.max_jerk = (
            None if max_jerk is None else _positive_vector(max_jerk, self.size, "max_jerk")
        )
        # Do not use the outer software limit as a reachable motion target.
        # Reserve a full jerk-limited stop from maximum velocity plus one command
        # tick; the outer soft limit remains the emergency feedback boundary.
        self.target_braking_guard = (
            self._stopping_distance_for_velocity(self.max_velocity)
            + self.max_velocity * self.dt
        )
        self.jerk_braking_extra_distance_at_max_velocity = (
            np.zeros(self.size)
            if self.max_jerk is None
            else self.max_velocity * self.max_acceleration / (2.0 * self.max_jerk)
        )
        self.target_natural_frequency = _positive_vector(
            target_natural_frequency,
            self.size,
            "target_natural_frequency",
        )
        self.soft_lower = self.lower_limits + float(limit_margin)
        self.soft_upper = self.upper_limits - float(limit_margin)
        if np.any(self.soft_lower >= self.soft_upper):
            raise ValueError("limit_margin leaves one or more joints without a safe range")
        if np.any(
            self.soft_lower + self.target_braking_guard
            >= self.soft_upper - self.target_braking_guard
        ):
            raise ValueError(
                "joint range is too small for the jerk-limited target braking guard"
            )
        self.target_lower = self.soft_lower + self.target_braking_guard
        self.target_upper = self.soft_upper - self.target_braking_guard
        self.last_command = None
        self.last_velocity = np.zeros(self.size)
        self.last_acceleration = np.zeros(self.size)

    def hold(self, q_hold, hold_mask):
        """Make selected joints hold an exact target with zero command motion."""
        q_hold = self._state_vector(q_hold, "q_hold")
        hold_mask = np.asarray(hold_mask, dtype=bool).reshape(-1)
        if hold_mask.size != self.size:
            raise ValueError(f"hold_mask must have length {self.size}")
        outside = hold_mask & (
            (q_hold < self.soft_lower) | (q_hold > self.soft_upper)
        )
        if np.any(outside):
            joints = np.flatnonzero(outside).tolist()
            raise RuntimeError(
                f"hold target lies outside joint soft limits at indices {joints}; "
                "enter FAULT"
            )
        if self.last_command is None:
            self.reset(q_hold)
        self.last_command[hold_mask] = q_hold[hold_mask]
        self.last_velocity[hold_mask] = 0.0
        self.last_acceleration[hold_mask] = 0.0

    def _acceleration_cap_for_velocity_headroom(self, headroom):
        """Exact discrete acceleration cap that can jerk back to zero in time."""
        headroom = np.maximum(0.0, np.asarray(headroom, dtype=float))
        jerk_step = self.max_jerk * self.dt
        distance_unit = self.dt * jerk_step
        steps = np.floor(
            (np.sqrt(1.0 + 8.0 * headroom / distance_unit) - 1.0) / 2.0
        )
        return (
            headroom + distance_unit * steps * (steps + 1.0) / 2.0
        ) / (self.dt * (steps + 1.0))

    def _stopping_distance_for_velocity(self, velocity):
        """Zero-acceleration stopping distance under acceleration and jerk limits."""
        velocity = np.abs(np.asarray(velocity, dtype=float))
        if self.max_jerk is None:
            return velocity**2 / (2.0 * self.max_acceleration)

        acceleration = self.max_acceleration
        jerk = self.max_jerk
        transition_velocity = acceleration**2 / jerk
        triangular_distance = velocity * np.sqrt(velocity / jerk)
        trapezoidal_distance = (
            velocity**2 / (2.0 * acceleration)
            + velocity * acceleration / (2.0 * jerk)
        )
        return np.where(
            velocity <= transition_velocity,
            triangular_distance,
            trapezoidal_distance,
        )

    def reset(self, q_feedback, dq_feedback=None):
        q_feedback = self._state_vector(q_feedback, "q_feedback")
        outside = (q_feedback < self.soft_lower) | (q_feedback > self.soft_upper)
        if np.any(outside):
            joints = np.flatnonzero(outside).tolist()
            raise RuntimeError(
                f"feedback lies outside joint soft limits at indices {joints}; enter FAULT"
            )
        self.last_command = q_feedback.copy()
        self.last_velocity = (
            np.zeros(self.size)
            if dq_feedback is None
            else np.clip(
                self._state_vector(dq_feedback, "dq_feedback"),
                -self.max_velocity,
                self.max_velocity,
            )
        )
        self.last_acceleration = np.zeros(self.size)

    def _state_vector(self, value, name):
        array = np.asarray(value, dtype=float).reshape(-1)
        if array.size != self.size or not np.all(np.isfinite(array)):
            raise ValueError(f"{name} must be a finite length-{self.size} vector")
        return array

    def limit(
        self,
        target,
        q_feedback,
        dq_feedback,
        active_mask=None,
        target_guard_mask=None,
    ):
        target = self._state_vector(target, "target").copy()
        q_feedback = self._state_vector(q_feedback, "q_feedback")
        dq_feedback = self._state_vector(dq_feedback, "dq_feedback")
        outside = (q_feedback < self.soft_lower) | (q_feedback > self.soft_upper)
        if np.any(outside):
            joints = np.flatnonzero(outside).tolist()
            raise RuntimeError(
                f"feedback crossed joint soft limits at indices {joints}; enter FAULT"
            )
        if self.last_command is None:
            self.reset(q_feedback, dq_feedback)

        if active_mask is not None:
            active_mask = np.asarray(active_mask, dtype=bool).reshape(-1)
            if active_mask.size != self.size:
                raise ValueError(f"active_mask must have length {self.size}")
            target[~active_mask] = q_feedback[~active_mask]

        target = np.clip(target, self.soft_lower, self.soft_upper)
        if target_guard_mask is None:
            target_guard_mask = np.ones(self.size, dtype=bool)
        else:
            target_guard_mask = np.asarray(target_guard_mask, dtype=bool).reshape(-1)
            if target_guard_mask.size != self.size:
                raise ValueError(f"target_guard_mask must have length {self.size}")
        guarded_target = np.clip(target, self.target_lower, self.target_upper)
        target[target_guard_mask] = guarded_target[target_guard_mask]
        # A critically damped command-space servo makes a fixed position target
        # settle instead of exciting a bang-bang acceleration/jerk oscillation.
        # Physical velocity/acceleration/jerk constraints below remain strict.
        position_error = target - self.last_command
        omega = self.target_natural_frequency
        desired_acceleration = (
            omega**2 * position_error - 2.0 * omega * self.last_velocity
        )
        desired_velocity = self.last_velocity + desired_acceleration * self.dt

        # Build one feasible interval so the final result cannot silently
        # violate acceleration/jerk after a later braking clamp.
        velocity_lower = -self.max_velocity.copy()
        velocity_upper = self.max_velocity.copy()
        max_dv = self.max_acceleration * self.dt
        velocity_lower = np.maximum(velocity_lower, self.last_velocity - max_dv)
        velocity_upper = np.minimum(velocity_upper, self.last_velocity + max_dv)

        if self.max_jerk is not None:
            max_da = self.max_jerk * self.dt
            acceleration_lower = np.maximum(
                -self.max_acceleration,
                self.last_acceleration - max_da,
            )
            acceleration_upper = np.minimum(
                self.max_acceleration,
                self.last_acceleration + max_da,
            )
            # Do not continue accelerating so close to a velocity boundary that
            # the jerk limit would make the following cycle infeasible. The cap
            # includes the exact discrete velocity accumulated while acceleration
            # is ramped back to zero at maximum jerk.
            acceleration_upper = np.minimum(
                acceleration_upper,
                self._acceleration_cap_for_velocity_headroom(
                    self.max_velocity - self.last_velocity
                ),
            )
            acceleration_lower = np.maximum(
                acceleration_lower,
                -self._acceleration_cap_for_velocity_headroom(
                    self.max_velocity + self.last_velocity
                ),
            )
            velocity_lower = np.maximum(
                velocity_lower,
                self.last_velocity + acceleration_lower * self.dt,
            )
            velocity_upper = np.minimum(
                velocity_upper,
                self.last_velocity + acceleration_upper * self.dt,
            )

        # Brake from measured state, not only from the previous command. The
        # one-tick term keeps the continuous stopping-distance rule conservative.
        upper_braking_boundary = self.soft_upper
        lower_braking_boundary = self.soft_lower
        upper_distance = np.maximum(
            0.0,
            upper_braking_boundary
            - q_feedback
            - np.maximum(0.0, dq_feedback) * self.dt,
        )
        lower_distance = np.maximum(
            0.0,
            q_feedback
            - lower_braking_boundary
            - np.maximum(0.0, -dq_feedback) * self.dt,
        )
        tick_velocity = self.max_acceleration * self.dt
        upper_braking_velocity = (
            np.sqrt(tick_velocity**2 + 2.0 * self.max_acceleration * upper_distance)
            - tick_velocity
        )
        lower_braking_velocity = (
            np.sqrt(tick_velocity**2 + 2.0 * self.max_acceleration * lower_distance)
            - tick_velocity
        )
        velocity_lower = np.maximum(velocity_lower, -lower_braking_velocity)
        velocity_upper = np.minimum(velocity_upper, upper_braking_velocity)
        velocity_lower = np.maximum(
            velocity_lower,
            (self.soft_lower - self.last_command) / self.dt,
        )
        velocity_upper = np.minimum(
            velocity_upper,
            (self.soft_upper - self.last_command) / self.dt,
        )
        infeasible = velocity_lower > velocity_upper + 1e-12
        if np.any(infeasible):
            joints = np.flatnonzero(infeasible).tolist()
            diagnostics = [
                {
                    "index": int(index),
                    "q_feedback": float(q_feedback[index]),
                    "dq_feedback": float(dq_feedback[index]),
                    "last_command": float(self.last_command[index]),
                    "last_velocity": float(self.last_velocity[index]),
                    "last_acceleration": float(self.last_acceleration[index]),
                    "velocity_lower": float(velocity_lower[index]),
                    "velocity_upper": float(velocity_upper[index]),
                    "braking_lower": float(lower_braking_boundary[index]),
                    "braking_upper": float(upper_braking_boundary[index]),
                }
                for index in np.flatnonzero(infeasible)
            ]
            raise RuntimeError(
                "no command satisfies joint acceleration/jerk and predictive braking "
                f"for indices {joints}; diagnostics={diagnostics}; enter FAULT"
            )
        velocity = np.minimum(
            np.maximum(desired_velocity, velocity_lower),
            velocity_upper,
        )
        command = self.last_command + velocity * self.dt
        acceleration = (velocity - self.last_velocity) / self.dt
        self.last_command = command.copy()
        self.last_velocity = velocity.copy()
        self.last_acceleration = acceleration.copy()
        return command
