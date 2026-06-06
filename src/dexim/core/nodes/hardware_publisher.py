"""HardwarePublisherNode base class.

Extends ManagedNode to provide a generic publisher loop for hardware sources
(e.g., cameras, robot state). Subclasses implement `get_data()` to provide
data objects and topics; this base handles timing, control messages, and
publishing packed messages on the Data Plane.
"""

from __future__ import annotations

import abc
import time
from typing import Any

import zmq
from dexim.core.messages import pack_data_message
from dexim.core.nodes.managed import ManagedNode
from loguru import logger


class HardwarePublisherNode(ManagedNode):
    """Base class for hardware-originating publisher nodes."""

    def __init__(
        self,
        node_id: str,
        data_endpoint: str,
        *,
        bind: bool = True,
        heartbeat_interval: float = 1.0,
        poll_timeout_ms: int = 1,
        rate_hz: float | None = None,
        control_endpoint: str | None = None,
        status_endpoint: str | None = None,
    ) -> None:
        super().__init__(
            node_id=node_id,
            control_endpoint=control_endpoint or "tcp://localhost:5550",
            status_endpoint=status_endpoint or "tcp://localhost:5551",
            heartbeat_interval=heartbeat_interval,
        )

        self._data_endpoint = data_endpoint
        self._bind = bool(bind)
        self._poll_timeout_ms = int(poll_timeout_ms)
        self._rate_hz = float(rate_hz) if rate_hz else None
        self._next_tick_ts: float = 0.0

        # PUB socket for data plane
        self._pub_data: zmq.Socket | None = None
        self._initialize_pub_socket()

        # Initialize rate limiter schedule
        if self._rate_hz and self._rate_hz > 0:
            self._next_tick_ts = time.time()

    # ----------------------
    # ZMQ setup/teardown
    # ----------------------
    def _initialize_pub_socket(self) -> None:
        if self._ctx is None:
            # ManagedNode ensures context initialization; guard for completeness
            self._ctx = zmq.Context.instance()
        pub = self._ctx.socket(zmq.PUB)
        # Avoid blocking on close; limit buffer to prevent unbounded memory growth
        pub.setsockopt(zmq.LINGER, 0)
        pub.setsockopt(zmq.SNDHWM, 100)
        if self._bind:
            pub.bind(self._data_endpoint)
        else:
            pub.connect(self._data_endpoint)
        self._pub_data = pub

    # ----------------------
    # ManagedNode hooks
    # ----------------------
    def on_standby(self) -> None:
        """Enter STANDBY: prepare tick schedule for fast START."""
        if self._rate_hz and self._rate_hz > 0:
            self._next_tick_ts = time.time()

    def on_start(self) -> None:
        # Reset tick schedule on start
        if self._rate_hz and self._rate_hz > 0:
            self._next_tick_ts = time.time()
        # Enable publishing on start
        self.is_publishing = True

    def on_pause(self) -> None:
        # Pause publishing but keep node alive
        self.is_publishing = False

    def on_start_recording(self) -> None:
        # Reset tick schedule when recording starts
        if self._rate_hz and self._rate_hz > 0:
            self._next_tick_ts = time.time()

    def on_stop_recording(self) -> None:
        # Default: no-op
        pass

    def on_stop(self) -> None:
        # Stop publishing (same as pause for hardware publishers)
        self.is_publishing = False

    def on_shutdown(self) -> None:
        # Disable publishing and close data PUB socket
        self.is_publishing = False
        try:
            if self._pub_data is not None:
                self._pub_data.close(linger=0)
        finally:
            self._pub_data = None

    def _main_loop_iteration(self) -> None:
        # Only publish when publishing is enabled
        if not self.is_publishing:
            return

        # Rate limiting: only proceed when the next tick is due
        if self._rate_hz and self._rate_hz > 0:
            now = time.time()
            if now < self._next_tick_ts:
                return
            # Schedule next tick
            period = 1.0 / self._rate_hz
            # Avoid drift by stepping in increments, not now+period
            self._next_tick_ts += period
            if self._next_tick_ts < now:
                # Catch up if we fell behind
                self._next_tick_ts = now + period

        # Fetch and publish data if available
        try:
            result = self.get_data()
        except Exception as e:
            # Skip this cycle on data acquisition error
            logger.error(f"Exception in get_data(): {e}")
            return

        if result is None:
            return

        # Support both single and batch returns for backward compatibility
        if isinstance(result, list):
            # Batch publishing: result is List[Tuple[bytes, Any]]
            for topic, data in result:
                self._send(topic, data)
        else:
            # Single publishing: result is Tuple[bytes, Any]
            topic, data = result
            self._send(topic, data)

    # ----------------------
    # Publishing helpers
    # ----------------------
    def _send(self, topic: bytes, data: Any) -> None:
        if self._pub_data is None:
            return
        try:
            topic_frame, payload_frame = self.pack_message(topic, data)
            self._pub_data.send_multipart(
                [topic_frame, payload_frame], flags=zmq.DONTWAIT
            )
        except zmq.Again:
            # DONTWAIT drop — expected when no subscribers or HWM reached; non-fatal.
            logger.debug(f"Dropped frame on topic {topic!r}: no subscriber or HWM")
        except Exception as e:
            logger.debug(f"Failed to send message on topic {topic!r}: {e}")

    def pack_message(self, topic: bytes, data: Any) -> tuple[bytes, bytes]:
        # Default packer: 2-frame (topic, msgpack payload with timestamp)
        topic_frame, payload_frame = pack_data_message(topic, time.time(), data)
        return topic_frame, payload_frame

    # ----------------------
    # Subclass API
    # ----------------------
    @abc.abstractmethod
    def get_data(self) -> tuple[bytes, Any] | list[tuple[bytes, Any]] | None:
        """Return (topic, data) to publish, List[(topic, data)] for batch, or None if no new data available."""
