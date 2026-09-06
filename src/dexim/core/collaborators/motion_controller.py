"""MotionController -- rate limiting, velocity limiting, timeout, and safe position."""

from __future__ import annotations

import time

import numpy as np
from loguru import logger

from dexim.core.nodes.utils import RateLimiter, smootherstep
from dexim.core.robot_interface import JointCommand, JointState, RobotInterface

_SMOOTHERSTEP_PEAK_SLOPE = 1.875
_SAFE_POSITION_TOLERANCE_RAD = 0.02
_SAFE_POSITION_SETTLE_TIMEOUT_SEC = 5.0
_SAFE_POSITION_SETTLE_SAMPLES = 3


class MotionController:
    """Manages control-loop timing, velocity limiting, timeouts, and safe-position moves.

    Args:
        interface: RobotInterface for reading and writing joint commands.
        rate_hz: Target control rate in Hz.
        enable_velocity_limiting: Gate for per-joint velocity clamping.
        max_joint_velocity_rad_s: Scalar velocity limit (rad/s) used when
            ``max_joint_velocity_per_joint`` is None.
        max_joint_velocity_per_joint: Per-joint velocity limits (URDF order).
            When provided each joint is clamped independently.
        safe_position_on_timeout: Move to safe position on data timeout.
        timeout_sec: Seconds of no data before timeout action triggers.
        safe_position_max_velocity_rad_s: Optional separate velocity limit for
            safe-position moves.
    """

    def __init__(
        self,
        interface: RobotInterface,
        rate_hz: float = 30.0,
        enable_velocity_limiting: bool = False,
        max_joint_velocity_rad_s: float = 2.0,
        max_joint_velocity_per_joint: list[float] | None = None,
        safe_position_on_timeout: bool = True,
        timeout_sec: float = 2.0,
        safe_position_max_velocity_rad_s: float | None = None,
    ) -> None:
        self._interface = interface
        self.dt = 1.0 / rate_hz
        self.rate_limiter = RateLimiter(rate_hz=rate_hz)
        self._current_joint_positions: np.ndarray | None = None
        self._last_data_time: float = time.time()

        self._velocity_limiting_enabled = enable_velocity_limiting
        self._max_joint_velocity_rad_s = max_joint_velocity_rad_s
        self._max_joint_velocity_per_joint = (
            np.array(max_joint_velocity_per_joint, dtype=float)
            if max_joint_velocity_per_joint is not None
            else None
        )
        self._safe_position_on_timeout = safe_position_on_timeout
        self._timeout_sec = timeout_sec
        self._safe_position_max_velocity_rad_s = safe_position_max_velocity_rad_s

        logger.info(
            f"MotionController: {rate_hz}Hz ({self.dt * 1000:.1f}ms), "
            f"vel_limit={'on' if enable_velocity_limiting else 'off'}"
        )

    @property
    def iterations(self) -> int:
        """Number of completed rate-limiter iterations."""
        return self.rate_limiter.iterations

    def initialize(self) -> None:
        """Perform one-time init: read initial joint state for velocity limiting."""
        if self._velocity_limiting_enabled and self._current_joint_positions is None:
            try:
                state = self._interface.read()
                self._current_joint_positions = state.q
                logger.info("Velocity limiting initialised with current joint state")
            except Exception as exc:
                logger.warning(f"Cannot read initial position: {exc}")
        self._last_data_time = time.time()

    def start_control_epoch(self) -> JointState | None:
        """Reset timing and seed a new control epoch from observed joints.

        Returns:
            The freshly observed joint state, or ``None`` when the interface
            cannot be read. Velocity limiting fails closed to a fresh read on
            its first command instead of retaining a previous epoch's cache.
        """
        self.rate_limiter.reset()
        self._current_joint_positions = None
        self._last_data_time = time.time()

        try:
            state = self._interface.read()
        except Exception as exc:
            logger.warning(f"Cannot seed control epoch from hardware: {exc}")
            return None

        if self._velocity_limiting_enabled:
            self._current_joint_positions = state.q.copy()
            logger.info("Velocity limiting seeded from current joint state")
        return state

    def apply_velocity_limits(self, target: np.ndarray) -> np.ndarray:
        """Clamp joint-position change to respect velocity limits.

        Args:
            target: Desired joint positions.

        Returns:
            Velocity-limited joint positions.
        """
        if not self._velocity_limiting_enabled:
            return target

        if self._current_joint_positions is not None:
            current = self._current_joint_positions
        else:
            try:
                current = self._interface.read().q
            except Exception:
                return target

        delta = target - current
        if self._max_joint_velocity_per_joint is not None:
            max_delta = self._max_joint_velocity_per_joint * self.dt
        else:
            max_delta = self._max_joint_velocity_rad_s * self.dt
        clamped = np.clip(delta, -max_delta, max_delta)
        safe = current + clamped
        self._current_joint_positions = safe.copy()
        return safe

    def send(self, q: np.ndarray) -> None:
        """Write a position command to the interface.

        Args:
            q: Joint positions.
        """
        try:
            self._interface.write(JointCommand(q=q, mode="position"))
        except Exception as exc:
            logger.error(f"Error sending command: {exc}")

    def check_timeout(self) -> bool:
        """Return True when data has timed out."""
        if not self._safe_position_on_timeout:
            return False
        return (time.time() - self._last_data_time) > self._timeout_sec

    def record_data_received(self) -> None:
        """Mark that valid data was received (resets timeout clock)."""
        self._last_data_time = time.time()

    def read_state(self):
        """Read the current joint state from the interface.

        Returns:
            JointState, or None on failure.
        """
        try:
            return self._interface.read()
        except Exception as exc:
            logger.debug(f"Failed to read interface state: {exc}")
            return None

    def sleep(self) -> dict:
        """Sleep to maintain target rate.

        Returns:
            Timing dict from RateLimiter (keys: elapsed, overtime, jitter, ...).
        """
        return self.rate_limiter.sleep()

    def get_statistics(self) -> dict:
        """Return rate-limiter statistics."""
        return self.rate_limiter.get_statistics()

    def move_to_safe(
        self,
        safe_position: np.ndarray,
        max_velocity_rad_s: float | None = None,
        timeout_sec: float = 5.0,
    ) -> bool:
        """Smoothly interpolate to *safe_position* with velocity limiting.

        Args:
            safe_position: Target joint configuration.
            max_velocity_rad_s: Override for velocity limit.
            timeout_sec: Minimum movement time budget. Long movements extend
                beyond this value when necessary to preserve the velocity
                ceiling.

        Returns:
            True if target reached, False on timeout or when interface is
            unavailable.
        """
        # Guard: if the interface is already disconnected we cannot move.
        if not self._interface.is_connected():
            logger.warning("Cannot move to safe position: interface is not connected")
            return False

        try:
            current = self._interface.read().q
        except Exception as exc:
            logger.warning(f"Cannot read current position: {exc}")
            return False

        delta = safe_position - current
        max_joint_delta = np.max(np.abs(delta))
        if max_joint_delta < 1e-4:
            return self._wait_until_at_safe_position(safe_position)

        if max_velocity_rad_s is not None:
            vel = max_velocity_rad_s
        elif self._safe_position_max_velocity_rad_s is not None:
            vel = self._safe_position_max_velocity_rad_s
        else:
            vel = self._max_joint_velocity_rad_s
        if vel <= 0:
            raise ValueError("safe-position velocity limit must be greater than zero")

        # smootherstep has a peak derivative of 1.875 at t=0.5.  Size the
        # trajectory for that peak rather than its average speed so every
        # commanded step remains within the configured velocity ceiling.
        required_duration_sec = _SMOOTHERSTEP_PEAK_SLOPE * max_joint_delta / vel
        num_steps = max(1, int(np.ceil(required_duration_sec / self.dt)))
        movement_timeout_sec = max(
            timeout_sec,
            num_steps * self.dt + max(1.0, 2 * self.dt),
        )

        logger.info(
            f"Moving to safe position: {num_steps} steps (~{num_steps * self.dt:.2f}s)"
        )

        start = time.monotonic()
        next_command_time = start + self.dt
        for i in range(num_steps):
            self._sleep_until(next_command_time)
            if time.monotonic() - start > movement_timeout_sec:
                logger.warning("Safe-position movement timed out")
                return False
            t = (i + 1) / num_steps
            interp = current + smootherstep(t) * delta
            self.send(interp)
            next_command_time = time.monotonic() + self.dt

        # Explicit final send to guarantee the exact safe position is reached,
        # even if floating-point or timing issues caused the last interpolated
        # step to be imprecise.
        self.send(safe_position)
        if not self._wait_until_at_safe_position(safe_position):
            logger.warning("Safe-position verification timed out")
            return False

        logger.success(f"Reached safe position in {time.monotonic() - start:.2f}s")
        return True

    @staticmethod
    def _sleep_until(deadline: float) -> None:
        """Wait until a monotonic deadline without reusing control-loop state."""
        while (remaining := deadline - time.monotonic()) > 0:
            time.sleep(remaining)

    def _wait_until_at_safe_position(self, safe_position: np.ndarray) -> bool:
        """Require consecutive feedback samples within the target tolerance."""
        deadline = time.monotonic() + _SAFE_POSITION_SETTLE_TIMEOUT_SEC
        settled_samples = 0

        while time.monotonic() <= deadline:
            state = self.read_state()
            if state is not None and np.allclose(
                state.q,
                safe_position,
                rtol=0.0,
                atol=_SAFE_POSITION_TOLERANCE_RAD,
            ):
                settled_samples += 1
                if settled_samples >= _SAFE_POSITION_SETTLE_SAMPLES:
                    self._current_joint_positions = state.q.copy()
                    return True
            else:
                settled_samples = 0

            time.sleep(min(self.dt, 0.05))

        return False
