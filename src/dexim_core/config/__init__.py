"""
Core Config Package

Shared configuration dataclasses and utilities for robot control nodes.

This package provides reusable configuration classes that are shared across
multiple robot control node packages.

Example:
    >>> from dexim_core.config import SubscriberConfig, ControlConfig, load_config
    >>> subscriber = SubscriberConfig(address="tcp://localhost:5555")
    >>> control = ControlConfig(rate_hz=30.0)
    >>> config = load_config("config.yaml")
"""

from .base import (
    # Base configs
    SubscriberConfig,
    ControlConfig,
    SimInterfaceConfig,
    TCPIPProtocolConfig,
    RS485ProtocolConfig,
    BaseOffsetConfig,
    # Nova configs
    NovaRealConfig,
    NovaConfig,
    # Inspire configs
    InspireRealConfig,
    InspireConfig,
    # DH5 configs
    DH5RealConfig,
    DH5Config,
    # G1 configs
    G1RealConfig,
    G1Config,
    # Interface configs
    RealInterfaceConfig,
    InterfaceConfig,
    # Hand tracking configs
    CameraConfig,
    MediaPipeConfig,
    EndpointsConfig,
    PublishConfig,
    HandTrackingConfig,
    # Main control node config
    ControlNodeConfig,
)

from .loader import (
    load_config,
    merge_config,
    validate_config,
    save_config,
    config_to_dict,
)

__version__ = "0.1.0"

__all__ = [
    # Base configs
    "SubscriberConfig",
    "ControlConfig",
    "SimInterfaceConfig",
    "TCPIPProtocolConfig",
    "RS485ProtocolConfig",
    "BaseOffsetConfig",
    # Nova configs
    "NovaRealConfig",
    "NovaConfig",
    # Inspire configs
    "InspireRealConfig",
    "InspireConfig",
    # DH5 configs
    "DH5RealConfig",
    "DH5Config",
    # G1 configs
    "G1RealConfig",
    "G1Config",
    # Interface configs
    "RealInterfaceConfig",
    "InterfaceConfig",
    # Hand tracking configs
    "CameraConfig",
    "MediaPipeConfig",
    "EndpointsConfig",
    "PublishConfig",
    "HandTrackingConfig",
    # Main control node config
    "ControlNodeConfig",
    # Loader functions
    "load_config",
    "merge_config",
    "validate_config",
    "save_config",
    "config_to_dict",
]
