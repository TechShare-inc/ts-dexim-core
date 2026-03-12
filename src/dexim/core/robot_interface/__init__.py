"""
Core Robot Interface Protocol

Provides base protocol and data structures for robot communication:
- DeviceInterface: Bidirectional device base protocol (connect/disconnect/read/write), generic over TState and TCmd
- RobotInterface: Full bidirectional robot interface (sensor + actuator); child of DeviceInterface
- SensorInterface: Read-only device protocol (state source), generic over TState
- ActuatorInterface: Write-only device protocol (command sink), generic over TCmd
- TState / TCmd: TypeVars for use with DeviceInterface / SensorInterface / ActuatorInterface
- JointState: Data structure for joint state information
- JointCommand: Data structure for joint commands
- SafetyMonitor: Safety monitoring and validation for robot commands
"""

from __future__ import annotations

from dexim.core.robot_interface.base import (
    ActuatorInterface,
    DeviceInterface,
    JointCommand,
    JointState,
    RobotInterface,
    SensorInterface,
    TCmd,
    TState,
)
from dexim.core.robot_interface.safety import SafetyMonitor

__all__ = [
    "DeviceInterface",
    "RobotInterface",
    "SensorInterface",
    "ActuatorInterface",
    "TState",
    "TCmd",
    "JointState",
    "JointCommand",
    "SafetyMonitor",
]
