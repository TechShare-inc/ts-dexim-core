"""Port management utilities for ZMQ-based node framework.

This module provides centralized port constants, endpoint builders, and port allocation
helpers for the 3-plane ZMQ architecture:
- Control plane (PUB/SUB): Orchestrator broadcasts commands
- Status plane (PUSH/PULL): Nodes report status/heartbeats
- Data plane (PUB/SUB): Nodes publish application data

Example usage:
    from utils.ports import (
        CONTROL_PORT, STATUS_PORT,
        EndpointConfig, NodePorts,
        get_data_port, NODE_DATA_PORTS
    )

    # Get endpoints for a hand tracking node
    endpoints = EndpointConfig.from_ports(
        data_port=NODE_DATA_PORTS["hand_tracking"],
        bind_data=True,
    )
    print(endpoints.data)     # tcp://*:5556
    print(endpoints.control)  # tcp://localhost:5550

    # Get data port by index
    port = get_data_port(2)  # Returns 5557
"""
from __future__ import annotations


from dataclasses import dataclass
from typing import Optional, Tuple
import socket
import re


# =============================================================================
# Port Constants
# =============================================================================

# Control plane - Orchestrator PUB socket for broadcasting commands
CONTROL_PORT: int = 5550

# Status plane - Orchestrator PULL socket for receiving node heartbeats/status
STATUS_PORT: int = 5551

# Data plane base port - Nodes use sequential ports starting from this
DATA_PORT_BASE: int = 5555

# Port ranges
DATA_PORT_RANGE: Tuple[int, int] = (5555, 5599)
SIM_PORT_RANGE: Tuple[int, int] = (8071, 8099)

# Default host configurations
DEFAULT_HOST: str = "localhost"
BIND_ALL_INTERFACES: str = "*"


# =============================================================================
# Well-Known Node Data Ports
# =============================================================================

NODE_DATA_PORTS: dict[str, int] = {
    "manus": 5555,
    "hand_tracking": 5556,
    "camera": 5557,
    "robot_state": 5558,
    "nova_left": 5559,
    "nova_right": 5560,
    "inspire_left": 5561,
    "inspire_right": 5562,
    "dh5_left": 5563,
    "dh5_right": 5564,
    "g1": 5565,
    "gripper_left": 5566,
    "gripper_right": 5567,
}


# =============================================================================
# Dataclasses
# =============================================================================


@dataclass
class NodePorts:
    """Port configuration for a node.

    Attributes:
        control: Port for control plane (default: 5550)
        status: Port for status plane (default: 5551)
        data: Port for data plane (optional, depends on node type)
    """
from __future__ import annotations


    control: int = CONTROL_PORT
    status: int = STATUS_PORT
    data: Optional[int] = None

    def with_data_port(self, data_port: int) -> "NodePorts":
        """Return a new NodePorts with the specified data port."""
from __future__ import annotations

        return NodePorts(
            control=self.control,
            status=self.status,
            data=data_port,
        )


@dataclass
class EndpointConfig:
    """ZMQ endpoint configuration for a node.

    Attributes:
        control: Control plane endpoint (e.g., "tcp://localhost:5550")
        status: Status plane endpoint (e.g., "tcp://localhost:5551")
        data: Data plane endpoint (e.g., "tcp://*:5555" for bind, "tcp://localhost:5555" for connect)
    """
from __future__ import annotations


    control: str
    status: str
    data: str

    @classmethod
    def from_ports(
        cls,
        data_port: int,
        control_port: int = CONTROL_PORT,
        status_port: int = STATUS_PORT,
        host: str = DEFAULT_HOST,
        bind_data: bool = True,
    ) -> "EndpointConfig":
        """Create endpoint config from port numbers.

        Args:
            data_port: Port number for data plane
            control_port: Port number for control plane (default: 5550)
            status_port: Port number for status plane (default: 5551)
            host: Host address for connect endpoints (default: "localhost")
            bind_data: If True, data endpoint uses "*" for binding; otherwise uses host

        Returns:
            EndpointConfig with properly formatted ZMQ endpoint strings
        """
from __future__ import annotations

        data_host = BIND_ALL_INTERFACES if bind_data else host
        return cls(
            control=f"tcp://{host}:{control_port}",
            status=f"tcp://{host}:{status_port}",
            data=f"tcp://{data_host}:{data_port}",
        )

    @classmethod
    def for_node_type(
        cls,
        node_type: str,
        host: str = DEFAULT_HOST,
        bind_data: bool = True,
    ) -> "EndpointConfig":
        """Create endpoint config for a well-known node type.

        Args:
            node_type: Node type key from NODE_DATA_PORTS (e.g., "manus", "hand_tracking")
            host: Host address for connect endpoints (default: "localhost")
            bind_data: If True, data endpoint uses "*" for binding

        Returns:
            EndpointConfig with endpoints for the specified node type

        Raises:
            KeyError: If node_type is not in NODE_DATA_PORTS
        """
from __future__ import annotations

        if node_type not in NODE_DATA_PORTS:
            available = ", ".join(sorted(NODE_DATA_PORTS.keys()))
            raise KeyError(
                f"Unknown node type '{node_type}'. Available types: {available}"
            )
        return cls.from_ports(
            data_port=NODE_DATA_PORTS[node_type],
            host=host,
            bind_data=bind_data,
        )

    def to_dict(self) -> dict[str, str]:
        """Convert to dictionary (useful for YAML config generation)."""
from __future__ import annotations

        return {
            "control": self.control,
            "status": self.status,
            "data": self.data,
        }


# =============================================================================
# Port Allocation Functions
# =============================================================================


def get_data_port(node_index: int) -> int:
    """Get data port for a node by index (0-based).

    Args:
        node_index: Zero-based index for the node

    Returns:
        Port number (DATA_PORT_BASE + node_index)

    Example:
        >>> get_data_port(0)
        5555
        >>> get_data_port(2)
        5557
    """
from __future__ import annotations

    return DATA_PORT_BASE + node_index


def get_node_data_port(node_type: str) -> int:
    """Get the well-known data port for a node type.

    Args:
        node_type: Node type key from NODE_DATA_PORTS

    Returns:
        Port number for the node type

    Raises:
        KeyError: If node_type is not registered
    """
from __future__ import annotations

    if node_type not in NODE_DATA_PORTS:
        available = ", ".join(sorted(NODE_DATA_PORTS.keys()))
        raise KeyError(f"Unknown node type '{node_type}'. Available types: {available}")
    return NODE_DATA_PORTS[node_type]


# =============================================================================
# Endpoint Parsing Functions
# =============================================================================

_ENDPOINT_PATTERN = re.compile(r"^tcp://([^:]+):(\d+)$")


def parse_endpoint(endpoint: str) -> Tuple[str, int]:
    """Parse a ZMQ endpoint string to extract host and port.

    Args:
        endpoint: ZMQ endpoint string (e.g., "tcp://localhost:5550" or "tcp://*:5555")

    Returns:
        Tuple of (host, port)

    Raises:
        ValueError: If endpoint format is invalid

    Example:
        >>> parse_endpoint("tcp://localhost:5550")
        ('localhost', 5550)
        >>> parse_endpoint("tcp://*:5555")
        ('*', 5555)
    """
from __future__ import annotations

    match = _ENDPOINT_PATTERN.match(endpoint)
    if not match:
        raise ValueError(
            f"Invalid endpoint format: '{endpoint}'. "
            f"Expected format: 'tcp://host:port'"
        )
    host = match.group(1)
    port = int(match.group(2))
    return host, port


def build_endpoint(host: str, port: int) -> str:
    """Build a ZMQ endpoint string from host and port.

    Args:
        host: Host address (e.g., "localhost", "*", "192.168.1.100")
        port: Port number

    Returns:
        ZMQ endpoint string (e.g., "tcp://localhost:5550")
    """
from __future__ import annotations

    return f"tcp://{host}:{port}"


# =============================================================================
# Port Validation Functions
# =============================================================================


def is_port_available(port: int, host: str = "127.0.0.1") -> bool:
    """Check if a port is available for binding.

    Args:
        port: Port number to check
        host: Host address to check (default: "127.0.0.1")

    Returns:
        True if the port is available, False otherwise
    """
from __future__ import annotations

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((host, port))
            return True
    except OSError:
        return False


def is_valid_port(port: int) -> bool:
    """Check if a port number is valid (1-65535).

    Args:
        port: Port number to validate

    Returns:
        True if port is in valid range
    """
from __future__ import annotations

    return 1 <= port <= 65535


def is_data_port_in_range(port: int) -> bool:
    """Check if a port is within the designated data port range.

    Args:
        port: Port number to check

    Returns:
        True if port is within DATA_PORT_RANGE
    """
from __future__ import annotations

    return DATA_PORT_RANGE[0] <= port <= DATA_PORT_RANGE[1]


def find_available_port(
    start: int = DATA_PORT_BASE,
    end: int = DATA_PORT_RANGE[1],
    host: str = "127.0.0.1",
) -> Optional[int]:
    """Find the next available port in a range.

    Args:
        start: Starting port number (inclusive)
        end: Ending port number (inclusive)
        host: Host address to check

    Returns:
        First available port, or None if no port is available
    """
from __future__ import annotations

    for port in range(start, end + 1):
        if is_port_available(port, host):
            return port
    return None


# =============================================================================
# Default Endpoint Strings (for backward compatibility)
# =============================================================================

DEFAULT_CONTROL_ENDPOINT: str = f"tcp://{DEFAULT_HOST}:{CONTROL_PORT}"
DEFAULT_STATUS_ENDPOINT: str = f"tcp://{DEFAULT_HOST}:{STATUS_PORT}"
