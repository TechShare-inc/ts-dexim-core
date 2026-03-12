"""DeviceNode — ManagedNode subclass with a formal device interface slot.

Imports ``DeviceInterface`` from ``dexim.core.robot_interface`` and re-exports
it for backward compatibility.  Adds a concrete ``on_shutdown()`` that
disconnects the interface before handing off to ``super()``, plus three
role-marker subclasses that express the node's communication direction.

Role subclasses
---------------
PublisherDeviceNode
    Publish-only devices (sensors/cameras), e.g. RealSense, Manus glove.
SubscriberDeviceNode
    Pure-actuator devices (command sinks only) — reserved for future use.
PubSubDeviceNode
    Bidirectional devices: read state *and* write commands, e.g. Inspire
    hand, Nova arm.
"""

from __future__ import annotations

from typing import Any

from dexim.core.nodes.managed import ManagedNode
from dexim.core.robot_interface import (
    DeviceInterface,
)  # re-exported for backward compat

__all__ = [
    "DeviceInterface",
    "DeviceNode",
    "PublisherDeviceNode",
    "SubscriberDeviceNode",
    "PubSubDeviceNode",
]


class DeviceNode(ManagedNode):
    """Abstract node that owns a hardware/simulation interface.

    Subclasses must still implement all remaining ``ManagedNode`` abstract
    methods (``on_start``, ``on_pause``, ``on_stop``, ``on_start_recording``,
    ``on_stop_recording``, ``_main_loop_iteration``).

    Attributes:
        interface: The device interface.  Must be assigned by the concrete
            subclass ``__init__`` before ``run()`` is called.  Typed as
            ``Any`` so both read-only (``SensorInterface``) and bidirectional
            (``DeviceInterface``) implementations are accepted without casts.
    """

    interface: Any

    def on_shutdown(self) -> None:
        """Disconnect the device interface, then delegate to super().

        Wraps ``interface.disconnect()`` in a broad ``except`` so that a
        hardware error during shutdown never prevents socket cleanup in
        ``ManagedNode.run()``.

        Raises:
            Nothing — all exceptions from ``disconnect()`` are swallowed and
            logged at DEBUG level.
        """
        try:
            self.interface.disconnect()
        except Exception as exc:  # noqa: BLE001
            # Intentionally broad: a failing disconnect must not block teardown.
            from loguru import logger

            logger.debug(f"{self.node_id} interface.disconnect() raised: {exc}")
        super().on_shutdown()


class PublisherDeviceNode(DeviceNode):
    """Role marker for publish-only device nodes (sensors / state sources).

    Examples: RealSense camera node, Manus glove node.
    """


class SubscriberDeviceNode(DeviceNode):
    """Role marker for subscribe-only device nodes (pure actuators).

    Reserved for future command-sink nodes that receive commands only.
    """


class PubSubDeviceNode(DeviceNode):
    """Role marker for bidirectional device nodes (read state + write commands).

    Examples: Inspire hand control node, Nova arm control node.
    """
