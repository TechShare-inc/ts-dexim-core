"""
Safety monitoring for robot teleoperation.

This module provides a SafetyMonitor class for validating robot commands
and detecting unsafe conditions.

Author: Haoyan Li
Date: October 24, 2025
"""
from __future__ import annotations

import time
from typing import Dict, Optional

import numpy as np
from loguru import logger

class SafetyMonitor:
    """Safety monitoring for robot teleoperation.

    Monitors and validates:
    - Data timeout (no data received for N seconds)
    - Joint position limits
    - Joint velocity limits (if applicable)

    Example:
        # Create monitor with limits
        monitor = SafetyMonitor(
            timeout_sec=5.0,
            joint_limits=np.array([[-1.0, 1.0], [-2.0, 2.0]]),
            velocity_limits=np.array([1.0, 2.0])
        )

        # In control loop
        while running:
            # Check data timeout
            if monitor.check_timeout():
                go_to_safe_position()

            # Get new data
            data = subscriber.receive()
            if data:
                q = process_data(data)

                # Validate command before sending
                if monitor.validate_joint_command(q, q_prev, dt):
                    interface.write(q)
                    monitor.update_last_data_time()
                else:
                    logger.warning("Command rejected by safety monitor")

        # Get violation report
        report = monitor.get_violation_report()
        print(f"Timeouts: {report['timeout_count']}")
    """

    def __init__(
        self,
        timeout_sec: float,
        joint_limits: Optional[np.ndarray] = None,
        velocity_limits: Optional[np.ndarray] = None,
    ):
        """Initialize safety monitor.

        Args:
            timeout_sec: Data timeout threshold (seconds)
            joint_limits: Nx2 array of [min, max] for each joint (radians or meters)
            velocity_limits: N array of max velocities (rad/s or m/s)
        """

        self.timeout_sec = timeout_sec
        self.last_data_time = time.time()

        self.joint_limits = joint_limits
        self.velocity_limits = velocity_limits

        # Statistics
        self.timeout_count = 0
        self.limit_violations = 0
        self.velocity_violations = 0
        self.total_validations = 0

        # Logging
        if joint_limits is not None:
            logger.debug(
                f"SafetyMonitor initialized with {len(joint_limits)} joint limits"
            )
        if velocity_limits is not None:
            logger.debug(
                f"SafetyMonitor initialized with {len(velocity_limits)} velocity limits"
            )
        logger.debug(f"Timeout threshold: {timeout_sec}s")

    def update_last_data_time(self):
        """Update timestamp of last valid data received.

        Call this after successfully receiving and processing data.
        """

        self.last_data_time = time.time()

    def check_timeout(self) -> bool:
        """Check if data timeout exceeded.

        Returns:
            True if timeout exceeded (time since last data > timeout_sec)
        """

        elapsed = time.time() - self.last_data_time
        if elapsed > self.timeout_sec:
            self.timeout_count += 1
            logger.warning(
                f"Data timeout detected: {elapsed:.2f}s > {self.timeout_sec}s "
                f"(timeout #{self.timeout_count})"
            )
            return True
        return False

    def validate_joint_limits(self, q: np.ndarray) -> bool:
        """Check if joint positions are within limits.

        Args:
            q: Joint positions array (N,)

        Returns:
            True if all joints within limits
        """

        if self.joint_limits is None:
            return True

        # Check array shape
        if len(q) != len(self.joint_limits):
            logger.error(
                f"Joint array size mismatch: got {len(q)}, "
                f"expected {len(self.joint_limits)}"
            )
            self.limit_violations += 1
            return False

        # Check each joint
        for i, (q_val, limits) in enumerate(zip(q, self.joint_limits)):
            q_min, q_max = limits
            if not (q_min <= q_val <= q_max):
                logger.warning(
                    f"Joint {i} limit violation: {q_val:.4f} rad "
                    f"not in [{q_min:.4f}, {q_max:.4f}] "
                    f"(exceeded by {max(q_val - q_max, q_min - q_val):.4f} rad)"
                )
                self.limit_violations += 1
                return False

        return True

    def validate_joint_velocities(
        self, q: np.ndarray, q_prev: np.ndarray, dt: float
    ) -> bool:
        """Check if joint velocities are within limits.

        Args:
            q: Current joint positions (N,)
            q_prev: Previous joint positions (N,)
            dt: Time step (seconds)

        Returns:
            True if all velocities within limits
        """

        if self.velocity_limits is None:
            return True

        # Check array shapes
        if len(q) != len(q_prev):
            logger.error(f"Joint array size mismatch: q={len(q)}, q_prev={len(q_prev)}")
            self.velocity_violations += 1
            return False

        if len(q) != len(self.velocity_limits):
            logger.error(
                f"Joint array size mismatch: got {len(q)}, "
                f"expected {len(self.velocity_limits)}"
            )
            self.velocity_violations += 1
            return False

        # Calculate velocities
        velocities = (q - q_prev) / dt

        # Check each joint
        for i, (v, v_max) in enumerate(zip(velocities, self.velocity_limits)):
            if abs(v) > v_max:
                logger.warning(
                    f"Joint {i} velocity violation: {abs(v):.4f} rad/s > {v_max:.4f} rad/s "
                    f"(exceeded by {abs(v) - v_max:.4f} rad/s)"
                )
                self.velocity_violations += 1
                return False

        return True

    def validate_joint_command(
        self,
        q: np.ndarray,
        q_prev: Optional[np.ndarray] = None,
        dt: float = 0.033,
    ) -> bool:
        """Validate complete joint command.

        This is the main validation method that checks all safety constraints.

        Args:
            q: Joint positions to validate (N,)
            q_prev: Previous positions for velocity check (N,), or None to skip
            dt: Time step (seconds), default is ~30Hz

        Returns:
            True if command passes all safety checks
        """

        self.total_validations += 1

        # Check for NaN or inf
        if not np.all(np.isfinite(q)):
            logger.error("Joint command contains NaN or inf values")
            self.limit_violations += 1
            return False

        # Check position limits
        if not self.validate_joint_limits(q):
            return False

        # Check velocity limits (if previous state provided)
        if q_prev is not None and self.velocity_limits is not None:
            if not self.validate_joint_velocities(q, q_prev, dt):
                return False

        return True

    def get_violation_report(self) -> Dict:
        """Get safety violation statistics.

        Returns:
            Dict with violation counts:
            - 'timeout_count': Number of data timeouts
            - 'limit_violations': Number of position limit violations
            - 'velocity_violations': Number of velocity limit violations
            - 'total_validations': Total number of validations performed
            - 'total_violations': Sum of all violations
            - 'violation_rate': Percentage of validations that failed
        """

        total_violations = (
            self.timeout_count + self.limit_violations + self.velocity_violations
        )

        violation_rate = (
            (total_violations / self.total_validations * 100)
            if self.total_validations > 0
            else 0.0
        )

        return {
            "timeout_count": self.timeout_count,
            "limit_violations": self.limit_violations,
            "velocity_violations": self.velocity_violations,
            "total_validations": self.total_validations,
            "total_violations": total_violations,
            "violation_rate": violation_rate,
        }

    def reset_statistics(self):
        """Reset violation statistics.

        Useful for starting a new monitoring session without creating a new instance.
        """

        self.timeout_count = 0
        self.limit_violations = 0
        self.velocity_violations = 0
        self.total_validations = 0
        logger.debug("SafetyMonitor statistics reset")

    def __repr__(self) -> str:
        """String representation of safety monitor."""

        report = self.get_violation_report()
        return (
            f"SafetyMonitor(timeout={self.timeout_sec}s, "
            f"violations={report['total_violations']}, "
            f"validations={report['total_validations']})"
        )
