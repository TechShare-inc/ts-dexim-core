"""DeviceNode — ManagedNode subclass with a formal device interface slot.

Defines ``DeviceInterface``, the minimal lifecycle protocol for any hardware
or simulation device (sensor, actuator, robot, camera, glove, …).  Adds a
concrete ``on_shutdown()`` that disconnects the interface before handing off
to ``super()``, plus three role-marker subclasses that express the node's
communication direction.

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

from typing import Protocol, runtime_checkable

from dexim.core.nodes.managed import ManagedNode


@runtime_checkable
class DeviceInterface(Protocol):
    """Minimal lifecycle protocol for any hardware or simulation device.

    Covers sensors (RealSense), input devices (Manus glove), and actuators
    (Inspire hand, Nova arm).  All three concrete interface families
    (``SensorInterface``, ``ActuatorInterface``, ``RobotInterface``) from
    ``dexim.core.robot_interface`` satisfy this protocol structurally.
    """

    def connect(self) -> None: ...
    def disconnect(self) -> None: ...


class DeviceNode(ManagedNode):
    """Abstract node that owns a hardware/simulation interface.

    Subclasses must still implement all remaining ``ManagedNode`` abstract
    methods (``on_start``, ``on_pause``, ``on_stop``, ``on_start_recording``,
    ``on_stop_recording``, ``_main_loop_iteration``).

    Attributes:
        interface: The device interface.  Must be assigned by the concrete
            subclass ``__init__`` before ``run()`` is called.  Declared here
            as a class-level annotation only; no default value is enforced so
            that subclasses may use any compatible type.
    """

    interface: DeviceInterface

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
