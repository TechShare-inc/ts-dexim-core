"""Protocols for core-node-framework type contracts.

This module defines runtime-checkable protocols for dependency injection and
type safety across the node framework. These protocols enable loose coupling
between components while maintaining type safety.

Protocols:
    DataSubscriber: For data subscribers that provide sensor data
    ControlNodeConfig: For control node configuration objects
    RobotInterfaceProtocol: Minimal interface for robot hardware/simulation
    RobotModelProtocol: For single-arm robot kinematic models
    DualArmRobotModelProtocol: For dual-arm robot kinematic models
    VectorOptimizerProtocol: For hand retargeting optimizers

Dataclasses:
    SensorWaitConfig: Configuration for sensor wait behavior

Note:
    The RobotInterfaceProtocol here is intentionally minimal for node framework use.
    Full robot interface contracts may include additional methods like connect(),
    num_joint_configurations(), time(), estop(), etc.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any, Literal, Protocol, runtime_checkable

import numpy as np

from dexim.core.spatial.transform import Transform3D


@dataclass
class SensorWaitConfig:
    """Configuration for sensor wait behavior.

    This dataclass configures how nodes wait for sensors (trackers/skeletons)
    to become available. It supports graceful degradation by allowing longer
    timeouts for cases where manus-node starts after hardware nodes.

    Attributes:
        timeout_sec: Maximum time to wait for sensor in seconds (default: 5.0)
        poll_interval_sec: Interval between sensor checks in seconds (default: 0.2)
        require_stable_frames: Number of consecutive frames sensor must be seen (default: 2)
        graceful_degradation: If True, log warning and continue on timeout; if False, raise exception (default: True)
        skip_on_setup: If True, skip skeleton waiting during setup_home_configuration (default: False)
    """

    timeout_sec: float = 5.0
    poll_interval_sec: float = 0.2
    require_stable_frames: int = 2
    graceful_degradation: bool = True
    skip_on_setup: bool = False

    def __post_init__(self):
        """Validate configuration values."""
        if self.timeout_sec <= 0:
            raise ValueError(f"timeout_sec must be positive, got {self.timeout_sec}")
        if self.poll_interval_sec <= 0:
            raise ValueError(
                f"poll_interval_sec must be positive, got {self.poll_interval_sec}"
            )
        if self.require_stable_frames < 1:
            raise ValueError(
                f"require_stable_frames must be >= 1, got {self.require_stable_frames}"
            )


# Default sensor wait configuration
DEFAULT_SENSOR_WAIT_CONFIG = SensorWaitConfig()


@runtime_checkable
class DataSubscriber(Protocol):
    """Protocol for data subscribers that provide sensor data.

    Implementations should provide:
    - read(): Get latest data from the subscriber (returns dict with 'skeletons', 'trackers', 'raw')
    - close(): Clean up subscriber resources
    - wait_for_sensor(): Wait for a specific sensor to appear
    - wait_for_sensors(): Wait for multiple sensors
    - get_landscape(): Get sensor landscape structure
    - get_metrics(): Get connection metrics and statistics
    """

    def read(self) -> dict[str, Any]:
        """Read latest data from the subscriber.

        Returns:
            Dict with parsed data, e.g., {"trackers": [...], "skeletons": {...}}
        """
        ...

    def close(self) -> None:
        """Close the subscriber connection and release resources."""
        ...

    def wait_for_sensor(
        self,
        sensor_type: str,
        side: str,
        glove_id: str | int | None = None,
        timeout_sec: float = 30.0,
        poll_interval_sec: float = 0.2,
        require_stable_frames: int = 2,
    ) -> bool:
        """Wait for a specific sensor to appear in the landscape.

        Args:
            sensor_type: Type of sensor ("tracker" or "skeleton")
            side: Side to match ("left" or "right")
            glove_id: Glove ID for skeleton matching (required for skeletons)
            timeout_sec: Maximum time to wait in seconds
            poll_interval_sec: Interval between polls
            require_stable_frames: Number of frames sensor must be seen

        Returns:
            True if sensor found, False if timeout
        """
        ...

    def wait_for_sensors(
        self,
        sensors: list[tuple],
        timeout_sec: float = 30.0,
        poll_interval_sec: float = 0.2,
        require_stable_frames: int = 2,
    ) -> dict[str, bool]:
        """Wait for multiple sensors to appear in the landscape.

        Args:
            sensors: List of (sensor_type, side) tuples,
                e.g., [("tracker", "left"), ("tracker", "right")]
            timeout_sec: Maximum time to wait in seconds
            poll_interval_sec: Interval between polls
            require_stable_frames: Number of frames each sensor must be seen

        Returns:
            Dict mapping "sensor_type_side" to whether it was found

        Example:
            >>> results = subscriber.wait_for_sensors(
            ...     [("tracker", "left"), ("tracker", "right")],
            ...     timeout_sec=30.0
            ... )
            >>> if all(results.values()):
            ...     # All sensors available
        """
        ...

    def get_landscape(self) -> dict[str, Any] | None:
        """Get current sensor landscape structure.

        Returns:
            Dict with "trackers" and "skeletons" keys, or None if not available
        """
        ...

    def get_metrics(self) -> dict[str, Any]:
        """Get connection metrics and statistics.

        Returns:
            Dict with connection info, message counts, etc.
        """
        ...


@runtime_checkable
class ControlNodeConfig(Protocol):
    """Protocol for control node configuration.

    Implementations should provide:
    - robot_type: String identifier for the robot type
    - subscriber: Subscriber configuration (address, etc.)
    - control: Control loop configuration (rate, timeout, velocity limits)
    - interface: Interface configuration (sim/real mode)
    - data_endpoint: ZMQ endpoint for Data Plane publishing
    - bind_data: Whether to bind (True) or connect (False) to data endpoint
    - observation_rate_hz: Rate for publishing observations (None = same as control rate)
    """

    @property
    def robot_type(self) -> str:
        """Robot type identifier."""
        ...

    @property
    def subscriber(self) -> Any:
        """Subscriber configuration with address, etc."""
        ...

    @property
    def control(self) -> Any:
        """Control configuration with rate_hz, timeout_sec, etc."""
        ...

    @property
    def interface(self) -> Any:
        """Interface configuration with mode (sim/real)."""
        ...

    @property
    def data_endpoint(self) -> str:
        """ZMQ endpoint for Data Plane publishing (default: 'tcp://*:5556')."""
        ...

    @property
    def bind_data(self) -> bool:
        """Whether to bind (True) or connect (False) to data endpoint (default: True)."""
        ...

    @property
    def observation_rate_hz(self) -> float | None:
        """Rate for publishing observations in Hz (default: None = same as control rate)."""
        ...


@runtime_checkable
class RobotInterfaceProtocol(Protocol):
    """Minimal protocol for robot hardware/simulation interfaces.

    This is a minimal protocol for node framework use. Full robot interface
    contracts may include additional methods like time(), estop(),
    num_joint_configurations(), joint_names(), etc.

    Required methods:
    - connect(): Establish connection to robot
    - read(): Get current robot state (JointState)
    - write(): Send command to robot (JointCommand)
    - disconnect(): Clean up interface resources
    """

    def connect(self) -> None:
        """Establish connection to robot hardware/simulation."""
        ...

    def read(self) -> Any:
        """Read current robot state.

        Returns:
            Robot state object with at least a 'q' attribute for joint positions
        """
        ...

    def write(self, cmd: Any) -> None:
        """Write command to robot.

        Args:
            cmd: Command object (e.g., JointCommand)
        """
        ...

    def disconnect(self) -> None:
        """Disconnect from robot and release resources."""
        ...


@runtime_checkable
class RobotModelProtocol(Protocol):
    """Protocol for single-arm robot kinematic models.

    For dual-arm robots, see DualArmRobotModelProtocol.

    Implementations should provide:
    - nq: Number of joint position variables (DOF)
    - get_neutral_configuration(): Get neutral joint configuration
    - compute_forward_kinematics(): FK computation
    - retarget(): IK computation for target pose
    """

    @property
    def nq(self) -> int:
        """Number of position variables (DOF)."""
        ...

    def get_neutral_configuration(self) -> np.ndarray:
        """Get neutral (zero/reference) joint configuration.

        Returns:
            Neutral joint configuration array (shape: [nq])
        """
        ...

    def compute_forward_kinematics(self, joint_angles: np.ndarray) -> Any:
        """Compute forward kinematics for given joint configuration.

        Args:
            joint_angles: Joint angles in radians (shape: [nq])

        Returns:
            End-effector pose as pinocchio SE3 object
        """
        ...

    def retarget(
        self,
        target_pose: np.ndarray,
        initial_guess: np.ndarray | None = None,
        **kwargs,
    ) -> tuple[np.ndarray, str]:
        """Retarget end-effector to a target pose using inverse kinematics.

        Args:
            target_pose: Target end-effector pose as 7-element array
                [x, y, z, qx, qy, qz, qw] (position in meters, normalized quaternion)
            initial_guess: Initial joint configuration in radians. If None, uses neutral config.
            **kwargs: Additional IK parameters (max_iterations, tolerance, damping, etc.)

        Returns:
            Tuple of (q_optimal, reason) where:
                - q_optimal: Best joint configuration found (shape: [nq])
                - reason: Human-readable description of result
        """
        ...


# Type alias for end-effector selector
EESelector = Literal["left", "right"]


@runtime_checkable
class DualArmRobotModelProtocol(Protocol):
    """Protocol for dual-arm robot kinematic models.

    For single-arm robots, see RobotModelProtocol.

    Implementations should provide:
    - nq: Number of joint position variables (DOF)
    - get_neutral_configuration(): Get neutral joint configuration
    - compute_forward_kinematics(): FK computation for specified end-effector
    - retarget(): IK computation for target pose(s)
    """

    @property
    def nq(self) -> int:
        """Number of position variables (DOF)."""
        ...

    def get_neutral_configuration(self) -> np.ndarray:
        """Get neutral (zero/reference) joint configuration.

        Returns:
            Neutral joint configuration array (shape: [nq])
        """
        ...

    def compute_forward_kinematics(
        self, joint_angles: np.ndarray, ee: EESelector
    ) -> Any:
        """Compute forward kinematics for given joint configuration.

        Args:
            joint_angles: Joint angles in radians (shape: [nq])
            ee: End-effector selector ("left" or "right")

        Returns:
            End-effector pose as pinocchio SE3 object
        """
        ...

    def retarget(
        self,
        target_poses: list[Any],
        ee: EESelector | None = None,
        initial_guess: np.ndarray | None = None,
        **kwargs,
    ) -> tuple[np.ndarray, str]:
        """Retarget end-effector(s) to target pose(s) using inverse kinematics.

        Args:
            target_poses: List of target SE3 poses.
                - If ee is specified, must be length 1 (pose for that EE)
                - If ee is None, must be length 2 in order [left, right]
            ee: Optional end-effector selector ("left" or "right").
                If None, both end-effectors are targeted.
            initial_guess: Initial joint configuration in radians. If None, uses last config.
            **kwargs: Additional IK parameters (position_weight, orientation_weight, etc.)

        Returns:
            Tuple of (q_optimal, reason) where:
                - q_optimal: Best joint configuration found (shape: [nq])
                - reason: Human-readable description of result
        """
        ...


class NloptReturnProtocol(Enum):
    """Protocol-compatible subset of NLopt return codes.

    Success-like values are >= 1, failure codes are negative.
    """

    SUCCESS = 1
    STOPVAL_REACHED = 2
    FTOL_REACHED = 3
    XTOL_REACHED = 4
    MAXEVAL_REACHED = 5
    MAXTIME_REACHED = 6
    FAILURE = -1
    INVALID_ARGS = -2
    OUT_OF_MEMORY = -3
    ROUNDOFF_LIMITED = -4
    FORCED_STOP = -5


@runtime_checkable
class VectorOptimizerProtocol(Protocol):
    """Protocol for hand retargeting optimizers (used by hand teleop nodes).

    The retarget() method returns an NloptReturn enum indicating the optimization
    result. Success-like values are >= 1 (SUCCESS, STOPVAL_REACHED, FTOL_REACHED,
    XTOL_REACHED, MAXEVAL_REACHED, MAXTIME_REACHED). Failure codes are negative.

    Implementations should provide:
    - alpha: Scaling factors for finger feature vectors
    - retarget(): Optimize joint angles from feature vectors
    """

    alpha: list[float]

    def retarget(self, target_features: np.ndarray) -> tuple[np.ndarray, Any]:
        """Retarget feature vectors to joint angles.

        Args:
            target_features: Target feature vectors array with shape
                (num_features, feature_dim), e.g., (5, 3) for 5 finger vectors in 3D

        Returns:
            Tuple of (joint_angles, nlopt_result) where:
                - joint_angles: Optimal joint configuration
                - nlopt_result: NloptReturn enum indicating optimization result
                  (SUCCESS=1, FTOL_REACHED=3, FAILURE=-1, etc.)
        """
        ...


@runtime_checkable
class TrackerDataProtocol(Protocol):
    """Protocol for tracker data objects.

    Represents 6DOF tracker pose data from tracking systems.

    Required attributes:
    - tracker_id: Unique identifier for the tracker
    - tracker_type: Type of tracker (e.g., "left_hand", "right_hand", "hmd")
    - is_hmd: Whether this tracker is a head-mounted display
    - system_type: Tracking system type (e.g., "SteamVR", "OptiTrack")
    - timestamp: Timestamp when the data was captured
    - transform: 6DOF pose (position + rotation) as Transform3D
    - user_id: User identifier
    """

    tracker_id: str
    tracker_type: str
    is_hmd: bool
    system_type: str
    timestamp: float
    transform: Transform3D
    user_id: int

    @property
    def side(self) -> str | None:
        """Infer side from tracker_type.

        Returns:
            'left' | 'right' | None
        """
        ...


@runtime_checkable
class SkeletonDataProtocol(Protocol):
    """Protocol for skeleton data objects.

    Represents skeleton data from glove systems with joint transforms
    and hierarchy information.

    Required attributes:
    - glove_id: Identifier of the glove
    - timestamp: Timestamp when the data was captured
    - joint_count: Total number of joints in the skeleton
    - joint_transforms: Dictionary mapping joint_id to Transform3D
    - hierarchy_infos: Dictionary mapping joint_id to hierarchy info
    """

    glove_id: int
    timestamp: float
    joint_count: int
    joint_transforms: dict[int, Transform3D]
    hierarchy_infos: dict[int, Any]

    @property
    def side(self) -> str | None:
        """Return side derived from hierarchy info.

        Returns:
            'left' | 'right' | None
        """
        ...
