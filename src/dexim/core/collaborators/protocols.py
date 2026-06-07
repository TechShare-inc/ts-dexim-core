"""Shared protocols for collaborator classes.

Defines structural protocols used across robot control nodes so that
mock objects and real implementations can be substituted without coupling
to concrete types.
"""

from __future__ import annotations

from typing import Any, Protocol, runtime_checkable


@runtime_checkable
class ReceiverProtocol(Protocol):
    """Minimal protocol for any data receiver collaborator.

    Both :class:`~dexim.core.collaborators.SkeletonReceiver` (hands) and
    :class:`~dexim.core.collaborators.TrackerReceiver` (arms) implement this.
    """

    def receive(self) -> Any | None:
        """Return the latest data item, or None if nothing is available."""
        ...

    def close(self) -> None:
        """Release resources held by the receiver."""
        ...


@runtime_checkable
class WaitableSubscriberProtocol(Protocol):
    """Subscribers that support active sensor waiting.

    Extends generic sub-protocol with a blocking ``wait_for_sensor`` method.
    :class:`~dexim.core.collaborators.SkeletonReceiver` uses this to delegate
    the sensor-ready check to the subscriber implementation.
    """

    def wait_for_sensor(
        self,
        sensor_type: str,
        side: str,
        glove_id: str | int | None,
        timeout_sec: float,
        poll_interval_sec: float,
        require_stable_frames: int,
    ) -> bool:
        """Block until sensor data becomes available.

        Args:
            sensor_type: Type of sensor to wait for (e.g. ``"skeleton"``).
            side: ``"left"`` or ``"right"``.
            glove_id: Expected glove ID, or None.
            timeout_sec: Maximum wait time.
            poll_interval_sec: Polling interval.
            require_stable_frames: Consecutive matching frames required.

        Returns:
            True if sensor found within timeout, False otherwise.
        """
        ...
