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
    hand, Tesollo hand, Nova arm.
"""

from __future__ import annotations

from abc import abstractmethod
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
    """Base class for bidirectional device nodes (read state + write commands).

    Concrete subclasses must implement :meth:`_run_pipeline` with the full
    per-iteration control logic.  The default :meth:`_main_loop_iteration`
    delegates to ``_run_pipeline()`` so subclasses only need to override one
    method.

    Examples: Inspire hand control node, Tesollo hand control node, Nova arm control node.
    """

    @abstractmethod
    def _run_pipeline(self) -> None:
        """Execute one iteration of the control pipeline.

        Called by :meth:`_main_loop_iteration` on every loop tick.
        Implement all per-cycle logic here: receive data, compute commands,
        send to interface, publish observations.
        """

    def _main_loop_iteration(self) -> None:
        """Delegate to :meth:`_run_pipeline`."""
        self._run_pipeline()
