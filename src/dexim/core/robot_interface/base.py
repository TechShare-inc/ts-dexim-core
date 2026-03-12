from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Protocol, TypeVar

import numpy as np

# TypeVars for the generic sensor/actuator protocols.
# TState: covariant — only produced by read() (return type).
# TCmd:   contravariant — only consumed by write() (parameter type).
TState = TypeVar("TState", covariant=True)
TCmd = TypeVar("TCmd", contravariant=True)


@dataclass
class JointState:
    q: np.ndarray  # actuated positions [n_actuated]
    qd: np.ndarray  # actuated velocities [n_actuated]
    tau: np.ndarray  # actuated measured torques [n_actuated]
    stamp: float  # timestamp (seconds)
    # Optional full robot configuration/state (e.g., 29-DOF for G1)
    q_full: np.ndarray | None = None
    qd_full: np.ndarray | None = None
    tau_full: np.ndarray | None = None


@dataclass
class JointCommand:
    q: np.ndarray | None = None  # desired actuated positions
    qd: np.ndarray | None = None  # desired actuated velocities
    tau: np.ndarray | None = None  # desired actuated torques
    mode: str = "position"  # control mode ("position", "velocity", "torque")
    stamp: float = time.time()
    # Optional full robot configuration command
    q_full: np.ndarray | None = None
    qd_full: np.ndarray | None = None
    tau_full: np.ndarray | None = None


class SensorInterface(Protocol[TState]):
    """Protocol for read-only devices (sensors / state sources).

    Type parameter:
        TState: The observable state type emitted by ``read()``
                (e.g. ``JointState``).
    """

    # --- lifecycle ---
    def connect(self) -> None: ...
    def disconnect(self) -> None: ...

    # --- state ---
    def read(self) -> TState: ...

    # --- meta ---
    def time(self) -> float: ...


class ActuatorInterface(Protocol[TCmd]):
    """Protocol for write-only devices (actuators / command sinks).

    Type parameter:
        TCmd: The command type consumed by ``write()``
              (e.g. ``JointCommand``).
    """

    # --- lifecycle ---
    def connect(self) -> None: ...
    def disconnect(self) -> None: ...

    # --- command ---
    def write(self, cmd: TCmd) -> None: ...

    # --- meta ---
    def estop(self) -> bool: ...


class RobotInterface(
    SensorInterface[JointState], ActuatorInterface[JointCommand], Protocol
):
    """Full bidirectional robot interface (sensor + actuator).

    Pins ``TState`` to ``JointState`` and ``TCmd`` to ``JointCommand``.
    Inherits ``connect``, ``disconnect``, ``read``, ``write``, ``time``,
    and ``estop`` from the parent protocols.
    """

    # --- meta ---
    def num_joint_configurations(self) -> int: ...
    def joint_names(self) -> list[str]: ...

    # Actuated vs Full configuration (for robots with reduced actuation)
    def num_actuated_configurations(self) -> int: ...
    def actuated_joint_names(self) -> list[str]: ...
    def num_full_configurations(self) -> int: ...
    def full_joint_names(self) -> list[str]: ...
