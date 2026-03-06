from typing import Protocol
from dataclasses import dataclass
import numpy as np
import time


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


class RobotInterface(Protocol):
    # --- lifecycle ---
    def connect(self) -> None: ...
    def disconnect(self) -> None: ...

    # --- state I/O ---
    def read(self) -> JointState: ...
    def write(self, cmd: JointCommand) -> None: ...

    # --- meta ---
    def num_joint_configurations(self) -> int: ...
    def joint_names(self) -> list[str]: ...

    # Actuated vs Full configuration (optional for robots with reduced actuation)
    def num_actuated_configurations(self) -> int: ...
    def actuated_joint_names(self) -> list[str]: ...
    def num_full_configurations(self) -> int: ...
    def full_joint_names(self) -> list[str]: ...
    def time(self) -> float: ...
    def estop(self) -> bool: ...
