"""YAML configuration loading and merging utilities.

This module provides functions for loading, merging, validating, and saving
configuration files.
"""

from __future__ import annotations

from dataclasses import asdict
from pathlib import Path
from typing import Any

import yaml
from loguru import logger

from .base import (
    BaseOffsetConfig,
    CameraConfig,
    ControlConfig,
    ControlNodeConfig,
    DH5Config,
    DH5RealConfig,
    EndpointsConfig,
    G1Config,
    G1RealConfig,
    HandTrackingConfig,
    InspireConfig,
    InspireRealConfig,
    InterfaceConfig,
    MediaPipeConfig,
    NovaConfig,
    NovaRealConfig,
    PublishConfig,
    RealInterfaceConfig,
    RS485ProtocolConfig,
    SimInterfaceConfig,
    SubscriberConfig,
    TCPIPProtocolConfig,
)


def load_config(
    yaml_path: str,
) -> ControlNodeConfig | HandTrackingConfig:
    """Load configuration from YAML file.

    Supports both ControlNodeConfig and HandTrackingConfig based on node_type.

    Args:
        yaml_path: Path to YAML configuration file

    Returns:
        ControlNodeConfig or HandTrackingConfig: Loaded configuration

    Raises:
        FileNotFoundError: If config file doesn't exist
        ValueError: If config is invalid
    """

    config_path = Path(yaml_path)

    if not config_path.exists():
        raise FileNotFoundError(f"Configuration file not found: {yaml_path}")

    logger.info(f"Loading configuration from {yaml_path}")

    with open(config_path) as f:
        config_dict = yaml.safe_load(f)

    if not config_dict:
        raise ValueError(f"Empty configuration file: {yaml_path}")

    # Determine config type based on node_type
    node_type = config_dict.get("node_type")

    if node_type == "hand_tracking":
        hand_tracking_config = _dict_to_hand_tracking_config(config_dict)
        logger.success(
            "HandTrackingConfig loaded: "
            f"node_id={hand_tracking_config.node_id}, "
            f"camera_mode={hand_tracking_config.camera.mode}"
        )
        return hand_tracking_config

    control_config = _dict_to_control_config(config_dict)
    logger.success(
        f"Configuration loaded: robot_type={control_config.robot_type}, "
        f"interface_mode={control_config.interface.mode}"
    )
    return control_config


def _parse_interface(
    interface_dict: dict[str, Any], robot_type: str
) -> InterfaceConfig:
    """Parse the interface section into an InterfaceConfig.

    Args:
        interface_dict: Mapping for the `interface` section from YAML
        robot_type: The parent `robot_type` (used as a hint/for validation)

    Returns:
        InterfaceConfig instance
    """

    if not interface_dict:
        return InterfaceConfig()

    mode = interface_dict.get("mode", "sim")
    if mode == "sim":
        sim_vals = {
            k: v for k, v in interface_dict.items() if k in ("host", "port", "backend")
        }
        if sim_vals:
            sim = SimInterfaceConfig(**sim_vals)
        else:
            sim = SimInterfaceConfig()
        return InterfaceConfig(mode="sim", sim=sim)

    if mode == "real":
        hardware = interface_dict.get("hardware", robot_type)
        if hardware is None:
            raise ValueError(
                "'hardware' must be specified in interface when mode is 'real'"
            )

        nova = None
        dh5 = None
        inspire = None
        g1 = None

        if hardware == "nova":
            nova_src = dict(interface_dict.get("nova", {}))
            if "tcpip" in nova_src:
                tcpip_kwargs = nova_src.get("tcpip", {})
                tcpip = TCPIPProtocolConfig(**tcpip_kwargs)
            else:
                tcpip_kwargs = {
                    k: v for k, v in nova_src.items() if k in ("ip", "port")
                }
                tcpip = (
                    TCPIPProtocolConfig(**tcpip_kwargs)
                    if tcpip_kwargs
                    else TCPIPProtocolConfig()
                )

            nova = NovaRealConfig(
                protocol=nova_src.get("protocol", "tcpip"),
                tcpip=tcpip,
                use_simple_servo=nova_src.get("use_simple_servo", False),
                servo_t=nova_src.get("servo_t", 0.1),
                servo_lookahead=nova_src.get("servo_lookahead", 50),
                servo_gain=nova_src.get("servo_gain", 500),
            )
        elif hardware == "dh5":
            dh5_src = dict(interface_dict.get("dh5", {}))
            if "rs485" in dh5_src:
                rs485_kwargs = dh5_src.get("rs485", {})
                rs485 = RS485ProtocolConfig(**rs485_kwargs)
            else:
                rs485_kwargs = {
                    k: v for k, v in dh5_src.items() if k in ("port", "baud")
                }
                rs485 = (
                    RS485ProtocolConfig(**rs485_kwargs)
                    if rs485_kwargs
                    else RS485ProtocolConfig()
                )

            dh5 = DH5RealConfig(
                protocol=dh5_src.get("protocol", "rs485"),
                rs485=rs485,
                modbus_id=dh5_src.get("modbus_id"),
            )
        elif hardware == "inspire":
            insp_src = dict(interface_dict.get("inspire", {}))
            tcpip = None
            rs485 = None
            if "tcpip" in insp_src:
                tcpip = TCPIPProtocolConfig(**insp_src.get("tcpip", {}))
            elif "ip" in insp_src or "port" in insp_src:
                tcpip_kwargs = {
                    k: v for k, v in insp_src.items() if k in ("ip", "port")
                }
                tcpip = TCPIPProtocolConfig(**tcpip_kwargs)

            if "rs485" in insp_src:
                rs485 = RS485ProtocolConfig(**insp_src.get("rs485", {}))
            elif "port" in insp_src or "baud" in insp_src:
                rs485_kwargs = {
                    k: v for k, v in insp_src.items() if k in ("port", "baud")
                }
                rs485 = RS485ProtocolConfig(**rs485_kwargs)

            inspire = InspireRealConfig(
                protocol=insp_src.get("protocol", "tcpip"),
                tcpip=tcpip,
                rs485=rs485,
                modbus_id=insp_src.get("modbus_id"),
            )
        elif hardware == "unitree_g1":
            g1_src = dict(interface_dict.get("g1", {}))
            g1 = G1RealConfig(
                protocol=g1_src.get("protocol", "dds"),
                network_interface=g1_src.get("network_interface", "enp2s0"),
                control_mode_pr=g1_src.get("control_mode_pr", 0),
                control_dt=g1_src.get("control_dt", 0.002),
                kp=g1_src.get("kp"),
                kd=g1_src.get("kd"),
            )
        else:
            raise ValueError(f"Unknown hardware for interface.real: {hardware}")

        real = RealInterfaceConfig(
            hardware=hardware, nova=nova, dh5=dh5, inspire=inspire, g1=g1
        )
        return InterfaceConfig(mode="real", real=real)

    raise ValueError("interface.mode must be 'sim' or 'real'")


def _dict_to_control_config(config_dict: dict[str, Any]) -> ControlNodeConfig:
    """Convert dictionary to ControlNodeConfig.

    Args:
        config_dict: Dictionary from YAML file

    Returns:
        ControlNodeConfig: Parsed configuration
    """

    robot_type = config_dict.get("robot_type")
    if not robot_type:
        raise ValueError("'robot_type' is required in configuration")

    subscriber_config = SubscriberConfig(**config_dict.get("subscriber", {}))
    interface_config = _parse_interface(config_dict.get("interface", {}), robot_type)
    control_config = ControlConfig(**config_dict.get("control", {}))

    dh5_config = None
    nova_config = None
    inspire_config = None
    g1_config = None

    if robot_type == "dh5" and "dh5" in config_dict:
        dh5_config = DH5Config(**config_dict["dh5"])
    elif robot_type == "nova" and "nova" in config_dict:
        nova_dict = dict(config_dict["nova"])

        if "base_offset" in nova_dict:
            base_offset_dict = nova_dict.pop("base_offset")
            base_offset = BaseOffsetConfig(**base_offset_dict)
            nova_dict["base_offset"] = base_offset

        nova_config = NovaConfig(**nova_dict)
    elif robot_type == "inspire" and "inspire" in config_dict:
        inspire_dict = config_dict["inspire"]
        inspire_config = InspireConfig(**inspire_dict)
    elif robot_type == "unitree_g1" and "g1" in config_dict:
        g1_dict = dict(config_dict["g1"])

        if "base_offset" in g1_dict:
            base_offset_dict = g1_dict.pop("base_offset")
            base_offset = BaseOffsetConfig(**base_offset_dict)
            g1_dict["base_offset"] = base_offset

        g1_config = G1Config(**g1_dict)

    return ControlNodeConfig(
        robot_type=robot_type,
        subscriber=subscriber_config,
        interface=interface_config,
        control=control_config,
        dh5=dh5_config,
        nova=nova_config,
        inspire=inspire_config,
        g1=g1_config,
    )


def _dict_to_hand_tracking_config(config_dict: dict[str, Any]) -> HandTrackingConfig:
    """Convert dictionary to HandTrackingConfig.

    Args:
        config_dict: Dictionary from YAML file

    Returns:
        HandTrackingConfig: Parsed configuration
    """

    node_type = config_dict.get("node_type")
    if node_type != "hand_tracking":
        raise ValueError(f"Expected node_type='hand_tracking', got '{node_type}'")

    node_id = config_dict.get("node_id")
    if not node_id:
        raise ValueError("'node_id' is required in configuration")

    camera_config = CameraConfig(**config_dict.get("camera", {}))
    mediapipe_config = MediaPipeConfig(**config_dict.get("mediapipe", {}))
    endpoints_config = EndpointsConfig(**config_dict.get("endpoints", {}))
    publish_config = PublishConfig(**config_dict.get("publish", {}))

    heartbeat_interval = config_dict.get("heartbeat_interval", 1.0)

    return HandTrackingConfig(
        node_type=node_type,
        node_id=node_id,
        camera=camera_config,
        mediapipe=mediapipe_config,
        endpoints=endpoints_config,
        publish=publish_config,
        heartbeat_interval=heartbeat_interval,
    )


def merge_config(
    base_config: ControlNodeConfig, overrides: dict[str, Any]
) -> ControlNodeConfig:
    """Merge CLI overrides into base configuration.

    Args:
        base_config: Base configuration from YAML
        overrides: Dictionary of overrides (e.g., from CLI args)

    Returns:
        ControlNodeConfig: Merged configuration

    Example:
        >>> config = load_config("config.yaml")
        >>> config = merge_config(config, {"interface.port": 9090})
    """

    logger.debug(f"Merging {len(overrides)} configuration overrides")

    for key, value in overrides.items():
        _set_nested_attr(base_config, key, value)
        logger.debug(f"Override: {key} = {value}")

    return base_config


def _set_nested_attr(obj: Any, key: str, value: Any):
    """Set nested attribute using dot notation.

    Args:
        obj: Object to modify
        key: Attribute path (e.g., "interface.port")
        value: Value to set
    """

    parts = key.split(".")

    for part in parts[:-1]:
        obj = getattr(obj, part)

    setattr(obj, parts[-1], value)


def validate_config(config: ControlNodeConfig | HandTrackingConfig) -> bool:
    """Validate configuration for common issues.

    Args:
        config: Configuration to validate (ControlNodeConfig or HandTrackingConfig)

    Returns:
        bool: True if valid

    Raises:
        ValueError: If configuration is invalid
    """

    if isinstance(config, HandTrackingConfig):
        if config.node_type != "hand_tracking":
            raise ValueError(f"Invalid node_type: {config.node_type}")

        if not config.node_id:
            raise ValueError("node_id cannot be empty")

        if config.heartbeat_interval <= 0:
            raise ValueError(f"Invalid heartbeat_interval: {config.heartbeat_interval}")

        logger.success("HandTrackingConfig validation passed")
        return True

    # Handle ControlNodeConfig
    if config.robot_type not in ["dh5", "nova", "inspire", "unitree_g1"]:
        raise ValueError(f"Invalid robot_type: {config.robot_type}")

    if config.control.rate_hz <= 0:
        raise ValueError(f"Invalid control rate: {config.control.rate_hz}")

    if config.interface.mode not in ["sim", "real"]:
        raise ValueError(f"Invalid interface mode: {config.interface.mode}")

    # Robot-specific validation
    if config.robot_type == "nova":
        if config.nova and len(config.nova.home_joints_deg) != 6:
            raise ValueError(
                f"Nova home_joints_deg must have 6 elements, got {len(config.nova.home_joints_deg)}"
            )
    if config.robot_type == "inspire":
        if config.inspire and config.inspire.side not in ["left", "right"]:
            raise ValueError(
                f"InspireConfig.side must be 'left' or 'right', got {config.inspire.side}"
            )
    if config.robot_type == "unitree_g1":
        if config.g1 and len(config.g1.home_joints_deg) != 14:
            raise ValueError(
                f"G1 home_joints_deg must have 14 elements, got {len(config.g1.home_joints_deg)}"
            )

    logger.success("Configuration validation passed")
    return True


def save_config(config: ControlNodeConfig, yaml_path: str):
    """Save configuration to YAML file.

    Args:
        config: Configuration to save
        yaml_path: Path to save YAML file
    """

    config_dict = config_to_dict(config)

    output_path = Path(yaml_path)
    output_path.parent.mkdir(exist_ok=True, parents=True)

    with open(output_path, "w") as f:
        yaml.dump(config_dict, f, default_flow_style=False, sort_keys=False)

    logger.success(f"Configuration saved to {yaml_path}")


def config_to_dict(config: ControlNodeConfig) -> dict[str, Any]:
    """Convert ControlNodeConfig to dictionary.

    Args:
        config: Configuration to convert

    Returns:
        Dictionary representation
    """

    return asdict(config)


# Alias for backward compatibility
_dict_to_config = _dict_to_control_config
_config_to_dict = config_to_dict
