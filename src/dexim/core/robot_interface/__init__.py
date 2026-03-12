"""
Core Robot Interface Protocol

Provides base protocol and data structures for robot communication:
- RobotInterface: Full bidirectional robot interface (sensor + actuator)
- SensorInterface: Read-only device protocol (state source), generic over TState
- ActuatorInterface: Write-only device protocol (command sink), generic over TCmd
- TState / TCmd: TypeVars for use with SensorInterface / ActuatorInterface
- JointState: Data structure for joint state information
- JointCommand: Data structure for joint commands
- SafetyMonitor: Safety monitoring and validation for robot commands
"""
from __future__ import annotations


from dexim.core.robot_interface.base import (
    ActuatorInterface,
    JointCommand,
    JointState,
    RobotInterface,
    SensorInterface,
    TCmd,
    TState,
)
from dexim.core.robot_interface.safety import SafetyMonitor

__all__ = [
    "RobotInterface",
    "SensorInterface",
    "ActuatorInterface",
    "TState",
    "TCmd",
    "JointState",
    "JointCommand",
    "SafetyMonitor",
]
