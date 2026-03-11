"""core-node-framework: Base classes for ZMQ orchestrator-controlled services.

This package provides reusable base classes for building distributed service nodes
that can be controlled by a central orchestrator using ZMQ messaging.

Node Hierarchy:
    ManagedNode: Abstract base for all orchestrator-controlled nodes
    ├── TeleopNode: Base for robot teleoperation control nodes
    │   ├── ArmTeleopNode: For arm robots using tracker data (IK-based)
    │   ├── DualArmTeleopNode: For dual-arm robots (e.g., G1 humanoid)
    │   └── HandTeleopNode: For hand robots using skeleton data (optimizer-based)
    ├── HardwarePublisherNode: For hardware data publishing nodes
    ├── CommandNode: For command publishing nodes
    └── RecorderNode: For data recording nodes

Protocols:
    DataSubscriber: Protocol for data subscribers
    ControlNodeConfig: Protocol for control node configuration
    RobotInterfaceProtocol: Protocol for robot interfaces
    RobotModelProtocol: Protocol for robot kinematic models
    VectorOptimizerProtocol: Protocol for hand retargeting optimizers

Utilities:
    RateLimiter: Precise timing utility for control loops

Example:
    from dexim.core.nodes import HandTeleopNode

    class DH5ControlNode(HandTeleopNode):
        def setup(self):
            self.subscriber = ManusSubscriber(...)
            self.model = DH5Model(...)
            self.interface = DH5Interface(...)

        def process_data(self, data):
            skeleton = self._get_skeleton_for_handedness(data)
            vectors = self._extract_features(skeleton)
            return self.optimizer.retarget(vectors)

    with DH5ControlNode("dh5_left", config) as node:
        node.run()
"""

from __future__ import annotations

# Base node classes
from dexim.core.nodes.arm_teleop_node import ArmTeleopNode
from dexim.core.nodes.command_node import CommandNode
from dexim.core.nodes.dual_arm_teleop_node import DualArmTeleopNode
from dexim.core.nodes.hand_teleop_node import HandTeleopNode
from dexim.core.nodes.hardware_publisher import HardwarePublisherNode
from dexim.core.nodes.managed import ManagedNode

# Orchestrator
from dexim.core.nodes.orchestrator import NodeStatus, TeleopOrchestrator

# Protocols
from dexim.core.nodes.protocols import (
    DEFAULT_SENSOR_WAIT_CONFIG,
    ControlNodeConfig,
    DataSubscriber,
    RobotInterfaceProtocol,
    RobotModelProtocol,
    SensorWaitConfig,
    SkeletonDataProtocol,
    TrackerDataProtocol,
    VectorOptimizerProtocol,
)

# Recorder node
from dexim.core.nodes.recorder_node import RecorderNode

# Teleop node hierarchy
from dexim.core.nodes.teleop_node import TeleopNode

# Generic typed subscriber (hardware-agnostic replacement for device subscribers)
from dexim.core.nodes.topic_subscriber import TopicSubscriber

# Utilities
from dexim.core.nodes.utils.rate_limiter import RateLimiter

__version__ = "0.2.0"

__all__ = [
    # Base nodes
    "ManagedNode",
    "HardwarePublisherNode",
    "CommandNode",
    # Teleop node hierarchy
    "TeleopNode",
    "ArmTeleopNode",
    "DualArmTeleopNode",
    "HandTeleopNode",
    # Recorder
    "RecorderNode",
    # Orchestrator
    "TeleopOrchestrator",
    "NodeStatus",
    # Protocols
    "DataSubscriber",
    "TopicSubscriber",
    "ControlNodeConfig",
    "RobotInterfaceProtocol",
    "RobotModelProtocol",
    "VectorOptimizerProtocol",
    "TrackerDataProtocol",
    "SkeletonDataProtocol",
    "SensorWaitConfig",
    "DEFAULT_SENSOR_WAIT_CONFIG",
    # Utilities
    "RateLimiter",
]
