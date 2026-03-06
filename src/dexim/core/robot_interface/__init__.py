"""
Core Robot Interface Protocol

Provides base protocol and data structures for robot communication:
- RobotInterface: Protocol defining standard robot interface
- JointState: Data structure for joint state information
- JointCommand: Data structure for joint commands
- SafetyMonitor: Safety monitoring and validation for robot commands
"""
from __future__ import annotations


from dexim.core.robot_interface.base import RobotInterface, JointState, JointCommand
from dexim.core.robot_interface.safety import SafetyMonitor

__all__ = [
    "RobotInterface",
    "JointState",
    "JointCommand",
    "SafetyMonitor",
]
