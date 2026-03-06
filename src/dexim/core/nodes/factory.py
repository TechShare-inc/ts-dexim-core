"""DEPRECATED: This module has been removed.

Interface factories have been moved to each robot node package:
- dh5_node.factory.create_dh5_interface
- nova_node.factory.create_nova_interface
- inspire_node.factory.create_inspire_interface

Import directly from the node packages instead:
    from dh5_node import create_dh5_interface
    from nova_node import create_nova_interface
    from inspire_node import create_inspire_interface

This file should be deleted. It only exists to provide a helpful error message
during the transition period.
"""
from __future__ import annotations


raise ImportError(
    "core_node_framework.factory has been removed. "
    "Use robot-specific factories instead:\n"
    "  - from dh5_node import create_dh5_interface\n"
    "  - from nova_node import create_nova_interface\n"
    "  - from inspire_node import create_inspire_interface"
)
