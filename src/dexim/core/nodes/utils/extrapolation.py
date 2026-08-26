"""Target extrapolator for real-time feed-forward control loops.

The "Chasing the Carrot" technique keeps a hardware actuator's internal PID
saturated during motion by commanding a position slightly *ahead* of the
current desired trajectory.  When the target stops changing, the estimated
velocity drops to zero and the carrot collapses to the real target, producing
implicit smooth deceleration without any special-casing.

Typical usage in a 30 Hz control loop::

    extrapolator = TargetExtrapolator(
        num_joints=6,
        dt=1.0 / 30.0,
        lookahead_cycles=1.5,
        joint_min=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        joint_max=np.array([1.7, 1.68, 1.7, 1.7, 1.1, 0.5]),
    )

    while running:
        target_q, _ = target_reader.read()
        cmd_q = extrapolator.extrapolate(target_q)
        interface.write(JointCommand(q=cmd_q, mode="position"))
"""

from __future__ import annotations

import numpy as np


class TargetExtrapolator:
    """Velocity-based target extrapolator -- the "Chasing the Carrot" method.

    On each call to :meth:`extrapolate` the class:

    1. Computes a velocity estimate: ``v = (target - prev_target) / dt``.
    2. Projects a *carrot* position: ``carrot = target + v * lookahead_cycles * dt``.
    3. Clips ``carrot`` to ``[joint_min, joint_max]``.
    4. Returns ``carrot`` as the command to send to the hardware.

    Because step 2 is equivalent to
    ``carrot = target + (target - prev_target) * lookahead_cycles``, the
    extrapolation distance is proportional to how much the target *moved* in
    the last cycle.  When the target is stationary ``v = 0`` and
    ``carrot = target``, so the actuator decelerates into the real target
    without any explicit ramp-down logic.

    Attributes:
        num_joints: Number of controlled joints.
        dt: Control period in seconds.
        lookahead_cycles: Number of cycles to project ahead (may be fractional).
        joint_min: Lower joint-limit array, shape ``(num_joints,)``.
        joint_max: Upper joint-limit array, shape ``(num_joints,)``.

    Args:
        num_joints: Number of controlled joints.
        dt: Control period in seconds (e.g. ``1.0 / 30.0``).
        lookahead_cycles: Look-ahead distance expressed in control cycles.
            Typical range: 1-3.  Set to 0.0 to disable extrapolation
            (returns the raw target unchanged, after clipping).
        joint_min: Per-joint lower limits in radians, shape ``(num_joints,)``.
        joint_max: Per-joint upper limits in radians, shape ``(num_joints,)``.
    """

    def __init__(
        self,
        num_joints: int,
        dt: float,
        lookahead_cycles: float,
        joint_min: np.ndarray,
        joint_max: np.ndarray,
    ) -> None:
        self._nq = num_joints
        self._dt = dt
        self._lookahead = lookahead_cycles
        self._joint_min = np.asarray(joint_min, dtype=np.float64)
        self._joint_max = np.asarray(joint_max, dtype=np.float64)
        self._prev_target: np.ndarray | None = None

    def extrapolate(self, target_q: np.ndarray) -> np.ndarray:
        """Compute the carrot position for the current control tick.

        On the very first call the previous target is uninitialised, so the
        velocity is treated as zero and the raw (clipped) target is returned.
        This avoids a spurious jump at start-up.

        Args:
            target_q: Desired joint positions for this tick in radians,
                shape ``(num_joints,)``.

        Returns:
            Extrapolated (carrot) joint positions in radians, clipped to
            ``[joint_min, joint_max]``, shape ``(num_joints,)`` dtype float64.
        """
        target = np.asarray(target_q, dtype=np.float64)

        if self._prev_target is None:
            # First call -- bootstrap with no velocity.
            self._prev_target = target.copy()
            return np.clip(target, self._joint_min, self._joint_max)

        # velocity estimate (rad/s), then project forward
        velocity = (target - self._prev_target) / self._dt
        carrot = target + velocity * (self._lookahead * self._dt)
        carrot = np.clip(carrot, self._joint_min, self._joint_max)

        self._prev_target = target.copy()
        return carrot

    def reset(self) -> None:
        """Reset internal state (e.g. after a gap in target updates).

        After calling reset, the next :meth:`extrapolate` call behaves like
        the first call (zero-velocity bootstrap).
        """
        self._prev_target = None
