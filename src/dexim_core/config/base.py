"""Base configuration dataclasses shared across robot control nodes.

This module provides the common configuration classes that are reused by
multiple packages (inspire-node, nova-node, g1-node, etc.).
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np
import pinocchio as pin
import scipy.spatial.transform


@dataclass
class SubscriberConfig:
    """Configuration for Manus data subscriber.

    Attributes:
        address: ZMQ address to subscribe to (e.g., "tcp://localhost:5555")
        timeout_ms: Receive timeout in milliseconds
    """

    address: str = "tcp://localhost:5555"
    timeout_ms: int = 1000


@dataclass
class ControlConfig:
    """Configuration for control loop.

    Attributes:
        rate_hz: Control loop rate in Hz
        timeout_sec: Data timeout in seconds
        safe_position_on_timeout: Move to safe position on timeout
        move_to_home_at_start: Move to home position at startup
        enable_velocity_limiting: Enable velocity limiting for safe motion
        max_joint_velocity_rad_s: Maximum joint velocity in rad/s
        safe_position_max_velocity_rad_s: Optional separate velocity limit for safe position movements
    """

    rate_hz: float = 30.0
    timeout_sec: float = 1.0
    safe_position_on_timeout: bool = True
    move_to_home_at_start: bool = False

    # Velocity limiting
    enable_velocity_limiting: bool = True
    max_joint_velocity_rad_s: float = 3.14
    safe_position_max_velocity_rad_s: Optional[float] = None


@dataclass
class SimInterfaceConfig:
    """Configuration for simulation interface.

    Attributes:
        mode: Interface mode (always "sim")
        host: Simulation server host
        port: Simulation server port
        backend: Optional label for sim backend (e.g., 'ros')
    """

    mode: str = "sim"
    host: str = "localhost"
    port: int = 8080
    backend: Optional[str] = None


@dataclass
class TCPIPProtocolConfig:
    """TCP/IP protocol configuration.

    Attributes:
        ip: IP address of the device
        port: Port number
    """

    ip: str = "192.168.1.100"
    port: int = 5001


@dataclass
class RS485ProtocolConfig:
    """RS485 protocol configuration.

    Attributes:
        port: Serial port path (e.g., "/dev/ttyUSB0" or "COM3")
        baud: Baud rate
    """

    port: str = "/dev/ttyUSB0"
    baud: int = 115200


@dataclass
class BaseOffsetConfig:
    """Base offset transformation configuration.

    Defines the transformation between the robot base frame and the world frame.
    All angles are specified in DEGREES for user convenience, and converted to
    radians internally.

    Attributes:
        euler_xyz_deg: Euler angles [roll, pitch, yaw] in degrees (XYZ convention)
        translation_m: Translation [x, y, z] in meters
    """

    euler_xyz_deg: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    translation_m: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])

    def to_pin_se3(self) -> "pin.SE3":
        """Convert to Pinocchio SE3 transformation.

        Returns:
            pin.SE3 transformation matrix
        """
        # Convert degrees to radians
        euler_rad = np.deg2rad(self.euler_xyz_deg)

        # Create rotation matrix from Euler angles (XYZ convention)
        R = scipy.spatial.transform.Rotation.from_euler("xyz", euler_rad).as_matrix()

        # Create SE3 transformation
        return pin.SE3(R, np.array(self.translation_m))


# =============================================================================
# Nova-specific configurations
# =============================================================================


@dataclass
class NovaRealConfig:
    """Nova-specific real-hardware interface fields."""

    protocol: str = "tcpip"

    tcpip: TCPIPProtocolConfig = field(
        default_factory=lambda: TCPIPProtocolConfig(ip="192.168.1.100")
    )

    use_simple_servo: bool = False
    servo_t: float = 0.1
    servo_lookahead: int = 50
    servo_gain: int = 500

    def __post_init__(self):
        """Validate Nova protocol (TCP/IP only) and ensure payload exists."""
        allowed = {"tcpip"}
        if self.protocol not in allowed:
            raise ValueError(
                f"Invalid protocol for NovaRealConfig: {self.protocol}. Must be one of {allowed}"
            )

        if self.tcpip is None:
            raise ValueError(
                "NovaRealConfig requires a non-None 'tcpip' field of type TCPIPProtocolConfig"
            )

    def get_connection_info(self) -> Dict[str, Any]:
        """Return a normalized dict describing the connection info."""
        assert self.tcpip is not None
        return {"type": "tcpip", "ip": self.tcpip.ip, "port": self.tcpip.port}


@dataclass
class NovaConfig:
    """Nova arm-specific configuration.

    All angles are specified in DEGREES for user convenience.
    """

    control_mode: str = "relative_pose"
    handedness: str = "left"  # "left" or "right" - determines tracker name only
    calibration_file: Optional[str] = None  # Path to calibration JSON
    home_joints_deg: List[float] = field(
        default_factory=lambda: [0.0] * 6
    )  # Joint angles in degrees
    base_offset: Optional[BaseOffsetConfig] = None

    def __post_init__(self):
        """Validate configuration."""
        if self.handedness not in ["left", "right"]:
            raise ValueError(
                f"Invalid handedness: {self.handedness}. Must be 'left' or 'right'"
            )

        if len(self.home_joints_deg) != 6:
            raise ValueError(
                f"home_joints_deg must have 6 elements, got {len(self.home_joints_deg)}"
            )

    @property
    def home_joints_rad(self) -> np.ndarray:
        """Get home joints in radians."""
        return np.deg2rad(self.home_joints_deg)

    def get_base_placement(self) -> "pin.SE3":
        """Get base placement transformation."""
        if self.base_offset is None:
            return pin.SE3.Identity()
        return self.base_offset.to_pin_se3()


# =============================================================================
# Inspire-specific configurations
# =============================================================================


@dataclass
class InspireRealConfig:
    """Inspire-specific real-hardware interface fields."""

    protocol: str = "tcpip"  # 'tcpip' or 'rs485'

    tcpip: Optional[TCPIPProtocolConfig] = None
    rs485: Optional[RS485ProtocolConfig] = None

    modbus_id: Optional[int] = None

    def __post_init__(self):
        allowed = {"tcpip", "rs485"}
        if self.protocol not in allowed:
            raise ValueError(
                f"Invalid protocol for InspireRealConfig: {self.protocol}. Must be one of {allowed}"
            )

        if self.protocol == "tcpip" and self.tcpip is None:
            raise ValueError(
                "InspireRealConfig.protocol='tcpip' requires a non-None 'tcpip' field"
            )
        if self.protocol == "rs485" and self.rs485 is None:
            raise ValueError(
                "InspireRealConfig.protocol='rs485' requires a non-None 'rs485' field"
            )

    def get_connection_info(self) -> Dict[str, Any]:
        """Return a normalized dict describing the active connection."""
        if self.protocol == "tcpip":
            assert self.tcpip is not None
            return {"type": "tcpip", "ip": self.tcpip.ip, "port": self.tcpip.port}
        assert self.rs485 is not None
        return {
            "type": "rs485",
            "port": self.rs485.port,
            "baud": self.rs485.baud,
            "modbus_id": self.modbus_id,
        }


@dataclass
class InspireConfig:
    """Inspire hand-specific configuration."""

    feature_extraction: Dict[str, Any] = field(
        default_factory=lambda: {
            "src_indices": [1, 6, 11, 16, 21],
            "dst_indices": [4, 9, 14, 19, 24],
            "apply_rotation": True,
        }
    )

    alpha: List[float] = field(default_factory=lambda: [1.0] * 5)
    handedness: str = "left"
    active_dofs_override: Optional[int] = None

    def __post_init__(self):
        if self.handedness not in ["left", "right"]:
            raise ValueError(
                f"Invalid handedness for InspireConfig: {self.handedness}. Must be 'left' or 'right'"
            )

        if not isinstance(self.alpha, list) or len(self.alpha) != 5:
            raise ValueError(
                f"InspireConfig.alpha must be a list of 5 floats, got: {self.alpha}"
            )


# =============================================================================
# DH5-specific configurations
# =============================================================================


@dataclass
class DH5RealConfig:
    """DH5-specific real-hardware interface fields."""

    protocol: str = "rs485"

    rs485: RS485ProtocolConfig = field(
        default_factory=lambda: RS485ProtocolConfig(port="/dev/ttyUSB0", baud=115200)
    )

    modbus_id: Optional[int] = None

    def __post_init__(self):
        """Validate DH5 protocol (RS485 only) and ensure payload exists."""
        allowed = {"rs485"}
        if self.protocol not in allowed:
            raise ValueError(
                f"Invalid protocol for DH5RealConfig: {self.protocol}. Must be one of {allowed}"
            )

        if self.rs485 is None:
            raise ValueError(
                "DH5RealConfig requires a non-None 'rs485' field of type RS485ProtocolConfig"
            )

    def get_connection_info(self) -> Dict[str, Any]:
        """Return a normalized dict describing the RS485 connection info."""
        assert self.rs485 is not None
        info: Dict[str, Any] = {
            "type": "rs485",
            "port": self.rs485.port,
            "baud": self.rs485.baud,
        }
        if self.modbus_id is not None:
            info["modbus_id"] = self.modbus_id
        return info


@dataclass
class DH5Config:
    """DH5 hand-specific configuration."""

    handedness: str = "left"  # "left" or "right"
    feature_extraction: Dict[str, Any] = field(
        default_factory=lambda: {
            "src_indices": [1, 6, 11, 16, 21],  # Metacarpals
            "dst_indices": [4, 9, 14, 19, 24],  # Fingertips
            "apply_rotation": True,
        }
    )
    alpha: List[float] = field(
        default_factory=lambda: [1.5, 1.0, 1.0, 1.0, 1.0]
    )  # Scaling factors

    def __post_init__(self):
        if self.handedness not in ["left", "right"]:
            raise ValueError(
                f"Invalid handedness for DH5Config: {self.handedness}. Must be 'left' or 'right'"
            )


# =============================================================================
# G1 (Unitree) specific configurations
# =============================================================================


@dataclass
class G1RealConfig:
    """Unitree G1-specific real-hardware interface fields."""

    protocol: str = "dds"  # DDS communication protocol
    network_interface: str = "enp2s0"  # Network interface for DDS
    control_mode_pr: int = 0  # Control mode for ankle joints (0=PR, 1=AB)
    control_dt: float = 0.002  # Control loop timestep in seconds (2ms)
    kp: Optional[List[float]] = None  # Proportional gains
    kd: Optional[List[float]] = None  # Derivative gains

    def __post_init__(self):
        """Validate G1 DDS configuration."""
        if self.protocol != "dds":
            raise ValueError(
                f"Invalid protocol for G1RealConfig: {self.protocol}. Must be 'dds'"
            )

        if self.control_mode_pr not in [0, 1]:
            raise ValueError(
                f"Invalid control_mode_pr: {self.control_mode_pr}. Must be 0 (PR) or 1 (AB)"
            )

        if self.control_dt <= 0:
            raise ValueError(f"Invalid control_dt: {self.control_dt}. Must be > 0")

    def get_connection_info(self) -> Dict[str, Any]:
        """Return a normalized dict describing the DDS connection info."""
        return {
            "type": "dds",
            "network_interface": self.network_interface,
            "control_mode_pr": self.control_mode_pr,
            "control_dt": self.control_dt,
        }


@dataclass
class G1Config:
    """Unitree G1 dual-arm humanoid configuration.

    Supports two DOF variants:
    - 23-DOF: 5 DOF per arm (shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll)
              Total 10 arm joints, home_joints_deg should have 10 elements
    - 29-DOF: 7 DOF per arm (adds wrist_pitch, wrist_yaw)
              Total 14 arm joints, home_joints_deg should have 14 elements
    """

    dof: int = 29  # DOF variant: 23 or 29 (29 includes wrist pitch/yaw)
    control_mode: str = "relative_pose"
    calibration_file: Optional[str] = None
    home_joints_deg: List[float] = field(
        default_factory=lambda: [0.0] * 14
    )  # Arm joints (14 for 29-DOF, 10 for 23-DOF)
    base_offset: Optional[BaseOffsetConfig] = field(
        default_factory=lambda: BaseOffsetConfig(translation_m=[0.0, 0.0, 0.75])
    )

    def __post_init__(self):
        """Validate configuration."""
        if self.dof not in [23, 29]:
            raise ValueError(f"Invalid dof: {self.dof}. Must be 23 or 29")

        # Validate home_joints_deg length based on DOF variant
        expected_joints = 14 if self.dof == 29 else 10
        if len(self.home_joints_deg) != expected_joints:
            raise ValueError(
                f"home_joints_deg must have {expected_joints} elements for {self.dof}-DOF variant, "
                f"got {len(self.home_joints_deg)}"
            )

    @property
    def arm_dof(self) -> int:
        """Get the number of DOF per arm."""
        return 7 if self.dof == 29 else 5

    @property
    def home_joints_rad(self) -> np.ndarray:
        """Get home joints in radians."""
        return np.deg2rad(self.home_joints_deg)

    def get_base_placement(self) -> "pin.SE3":
        """Get base placement transformation."""
        if self.base_offset is None:
            return pin.SE3(np.eye(3), np.array([0.0, 0.0, 0.75]))
        return self.base_offset.to_pin_se3()


# =============================================================================
# Interface configurations
# =============================================================================


@dataclass
class RealInterfaceConfig:
    """Configuration for a real hardware interface.

    The `hardware` field selects which nested config must be provided.
    """

    mode: str = "real"
    hardware: str = "nova"  # one of: 'nova', 'dh5', 'inspire', 'unitree_g1'

    nova: Optional[NovaRealConfig] = None
    dh5: Optional[DH5RealConfig] = None
    inspire: Optional[InspireRealConfig] = None
    g1: Optional[G1RealConfig] = None

    def __post_init__(self):
        allowed = {"nova", "dh5", "inspire", "unitree_g1"}
        if self.hardware not in allowed:
            raise ValueError(
                f"Invalid hardware for RealInterfaceConfig: {self.hardware}. Must be one of {allowed}"
            )

        if self.hardware == "nova" and self.nova is None:
            raise ValueError(
                "RealInterfaceConfig.hardware='nova' requires 'nova' field"
            )
        if self.hardware == "dh5" and self.dh5 is None:
            raise ValueError("RealInterfaceConfig.hardware='dh5' requires 'dh5' field")
        if self.hardware == "inspire" and self.inspire is None:
            raise ValueError(
                "RealInterfaceConfig.hardware='inspire' requires 'inspire' field"
            )
        if self.hardware == "unitree_g1" and self.g1 is None:
            raise ValueError(
                "RealInterfaceConfig.hardware='unitree_g1' requires 'g1' field"
            )


@dataclass
class InterfaceConfig:
    """Top-level interface config which discriminates sim vs real."""

    mode: str = "sim"  # 'sim' or 'real'
    sim: Optional[SimInterfaceConfig] = field(default_factory=SimInterfaceConfig)
    real: Optional[RealInterfaceConfig] = None

    def __post_init__(self):
        if self.mode == "sim":
            if self.sim is None:
                self.sim = SimInterfaceConfig()
        elif self.mode == "real":
            if self.real is None:
                raise ValueError(
                    "InterfaceConfig.mode='real' requires the 'real' field"
                )
        else:
            raise ValueError("InterfaceConfig.mode must be 'sim' or 'real'")

    def is_sim(self) -> bool:
        return self.mode == "sim"

    def is_real(self) -> bool:
        return self.mode == "real"

    def get_hardware_config(self):
        """Return the nested hardware-specific config for real mode."""
        if not self.is_real():
            return None

        assert self.real is not None
        hw = self.real.hardware
        if hw == "nova":
            return self.real.nova
        if hw == "dh5":
            return self.real.dh5
        if hw == "inspire":
            return self.real.inspire
        if hw == "unitree_g1":
            return self.real.g1
        return None


# =============================================================================
# Hand tracking configurations
# =============================================================================


@dataclass
class CameraConfig:
    """Camera configuration for HandTrackingNode."""

    mode: str = "usb"  # "usb" | "realsense"
    device_index: int = 0
    serial_number: Optional[str] = None
    preset: str = "720p30"
    publish_raw_frames: bool = False

    def __post_init__(self):
        # Import here to avoid circular dependencies
        from dexim_core.messages import TopicValidator

        validator = TopicValidator()

        # Validate camera mode and preset
        if self.mode not in ["usb", "realsense"]:
            raise ValueError(
                f"Invalid camera mode: {self.mode}. Must be 'usb' or 'realsense'"
            )

        valid_presets = ["480p30", "720p30", "1080p30", "480p60"]
        if self.preset not in valid_presets:
            raise ValueError(
                f"Invalid preset: {self.preset}. Must be one of {valid_presets}"
            )

        # Validate camera configuration can produce valid topics
        is_valid, error_msg = validator.validate_camera_config(
            self.mode, self.device_index, self.serial_number
        )
        if not is_valid:
            raise ValueError(f"Invalid camera configuration: {error_msg}")


@dataclass
class MediaPipeConfig:
    """MediaPipe Hands configuration."""

    max_hands: int = 2
    min_detection_confidence: float = 0.5
    min_tracking_confidence: float = 0.5
    model_complexity: int = 1

    def __post_init__(self):
        if self.max_hands < 1 or self.max_hands > 2:
            raise ValueError(f"Invalid max_hands: {self.max_hands}. Must be 1 or 2")

        if not 0.0 <= self.min_detection_confidence <= 1.0:
            raise ValueError(
                f"Invalid min_detection_confidence: {self.min_detection_confidence}"
            )

        if not 0.0 <= self.min_tracking_confidence <= 1.0:
            raise ValueError(
                f"Invalid min_tracking_confidence: {self.min_tracking_confidence}"
            )

        if self.model_complexity not in [0, 1]:
            raise ValueError(f"Invalid model_complexity: {self.model_complexity}")


@dataclass
class EndpointsConfig:
    """ZMQ endpoint configuration for HandTrackingNode."""

    data: str = "tcp://*:5556"
    control: str = "tcp://localhost:5550"
    status: str = "tcp://localhost:5551"


@dataclass
class PublishConfig:
    """Publishing behavior configuration."""

    rate_hz: Optional[float] = None  # None = publish at camera FPS


@dataclass
class HandTrackingConfig:
    """Complete configuration for HandTrackingNode."""

    node_type: str
    node_id: str
    camera: CameraConfig
    mediapipe: MediaPipeConfig
    endpoints: EndpointsConfig
    publish: PublishConfig
    heartbeat_interval: float = 1.0

    def __post_init__(self):
        # Import here to avoid circular dependencies
        from dexim_core.messages import TopicValidator

        validator = TopicValidator()

        if self.node_type != "hand_tracking":
            raise ValueError(
                f"Invalid node_type: {self.node_type}. Must be 'hand_tracking'"
            )

        if not self.node_id:
            raise ValueError("node_id cannot be empty")

        # Validate node_id can be used as device identifier
        is_valid, error_msg = validator.validate_device_id(self.node_id)
        if not is_valid:
            raise ValueError(f"Invalid node_id: {error_msg}")

        if self.heartbeat_interval <= 0:
            raise ValueError(
                f"Invalid heartbeat_interval: {self.heartbeat_interval}. Must be > 0"
            )


# =============================================================================
# Main control node configuration
# =============================================================================


@dataclass
class ControlNodeConfig:
    """Complete configuration for a control node."""

    robot_type: str  # "dh5", "nova", "inspire", or "unitree_g1"

    subscriber: SubscriberConfig = field(default_factory=SubscriberConfig)
    interface: InterfaceConfig = field(default_factory=InterfaceConfig)
    control: ControlConfig = field(default_factory=ControlConfig)

    # Robot-specific configs (only one will be used based on robot_type)
    dh5: Optional[DH5Config] = None
    nova: Optional[NovaConfig] = None
    inspire: Optional[InspireConfig] = None
    g1: Optional[G1Config] = None

    def __post_init__(self):
        """Validate configuration after initialization."""
        if self.robot_type not in ["dh5", "nova", "inspire", "unitree_g1"]:
            raise ValueError(
                f"Invalid robot_type: {self.robot_type}. Must be 'dh5', 'nova', 'inspire', or 'unitree_g1'"
            )

        # Initialize robot-specific config if not provided
        if self.robot_type == "dh5" and self.dh5 is None:
            self.dh5 = DH5Config()
        elif self.robot_type == "nova" and self.nova is None:
            self.nova = NovaConfig()
        elif self.robot_type == "inspire" and self.inspire is None:
            self.inspire = InspireConfig()
        elif self.robot_type == "unitree_g1" and self.g1 is None:
            self.g1 = G1Config()

    @property
    def robot_specific(self):
        """Get robot-specific config based on robot_type."""
        if self.robot_type == "dh5":
            return self.dh5
        elif self.robot_type == "nova":
            return self.nova
        elif self.robot_type == "inspire":
            return self.inspire
        elif self.robot_type == "unitree_g1":
            return self.g1
        else:
            raise ValueError(f"Unknown robot_type: {self.robot_type}")
