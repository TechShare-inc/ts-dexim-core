"""core-node-framework: Base classes for ZMQ orchestrator-controlled services.

This package provides reusable base classes for building distributed service nodes
that can be controlled by a central orchestrator using ZMQ messaging.

Node Hierarchy:
    ManagedNode: Abstract base for all orchestrator-controlled nodes
    +-- DeviceNode: Node owning a hardware/simulation interface (disconnect on shutdown)
    |   +-- PublisherDeviceNode: Role marker -- publish-only (sensors/cameras)
    |   +-- SubscriberDeviceNode: Role marker -- pure-actuator (command sinks)
    |   +-- PubSubDeviceNode: Base for bidirectional nodes (read state + write commands)
    |       +-- Subclass must implement _run_pipeline()
    +-- HardwarePublisherNode: For hardware data publishing nodes
    +-- CommandNode: For command publishing nodes
    +-- RecorderNode: For data recording nodes

Protocols:
    DataSubscriber: Protocol for data subscribers
    ControlNodeConfig: Protocol for control node configuration
    RobotInterfaceProtocol: Protocol for robot interfaces
    RobotModelProtocol: Protocol for robot kinematic models
    VectorOptimizerProtocol: Protocol for hand retargeting optimizers

Utilities:
    RateLimiter: Precise timing utility for control loops

Example:
    from dexim.core.nodes import PubSubDeviceNode

    class MyRobotNode(PubSubDeviceNode):
        def _run_pipeline(self):
            # full per-iteration control logic here
            ...

    with MyRobotNode("my_robot") as node:
        node.run()
"""

from __future__ import annotations

# Base node classes
from dexim.core.nodes.command_node import CommandNode
from dexim.core.nodes.device_node import (
    DeviceNode,
    PublisherDeviceNode,
    PubSubDeviceNode,
    SubscriberDeviceNode,
)
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
    # Device nodes
    "DeviceNode",
    "PublisherDeviceNode",
    "SubscriberDeviceNode",
    "PubSubDeviceNode",
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
