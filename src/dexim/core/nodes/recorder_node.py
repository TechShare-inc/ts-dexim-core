"""RecorderNode - Base class for data recording nodes.

This module provides the RecorderNode abstract base class for nodes that
record teleoperation data for training/replay:

- Multi-endpoint ZMQ subscription
- Thread-safe data buffering
- Episode-based recording with START_REC/STOP_REC commands
- Background thread for non-blocking episode saving

Data recorder nodes extend this class and implement format-specific
episode writing (HDF5, LeRobot, etc.).

Example:
    from dexim.core.nodes import RecorderNode

    class DataRecorderNode(RecorderNode):
        def _create_episode_writer(self, buffers, episode_number):
            return EpisodeWriterThread(
                buffers=buffers,
                format=self._storage_format,
                episode=episode_number,
            )

    recorder = DataRecorderNode(
        node_id="recorder",
        data_endpoints=["tcp://localhost:5556", "tcp://localhost:5557"],
    )
    recorder.run()
"""

from __future__ import annotations

import copy
import threading
from abc import abstractmethod
from collections import defaultdict
from typing import Any

import zmq
from loguru import logger

from dexim.core.nodes.managed import ManagedNode
from dexim.core.messages import (
    CTRL_PUB_ENDPOINT,
    STATUS_PULL_ENDPOINT,
    unpack_data_message,
)


class RecorderNode(ManagedNode):
    """Abstract base class for data recording nodes.

    This class extends ManagedNode with recording-specific features:
    - Multi-endpoint ZMQ subscription
    - Thread-safe data buffering
    - Episode management (counter, start/stop)
    - Non-blocking episode saving via background threads

    Subclasses must implement:
    - _create_episode_writer(): Create thread for saving episode data
    - on_finalize(): Optional cleanup when node shuts down

    Attributes:
        data_endpoints: List of ZMQ endpoints to subscribe to
        storage_format: Format for saving episodes ("hdf5", "lerobot", etc.)
        output_dir: Directory for saving episode files
        buffers: Dict mapping topic bytes to list of (timestamp, data) tuples
        buffer_lock: Threading lock for buffer access
        episode_counter: Current episode number
    """

    def __init__(
        self,
        node_id: str,
        data_endpoints: list[str],
        storage_format: str = "hdf5",
        output_dir: str = "output",
        *,
        control_endpoint: str | None = None,
        status_endpoint: str | None = None,
        heartbeat_interval: float = 1.0,
    ) -> None:
        """Initialize recorder node.

        Args:
            node_id: Unique identifier for this node
            data_endpoints: List of ZMQ endpoints to subscribe to
            storage_format: Format for saving episodes ("hdf5", "lerobot", etc.)
            output_dir: Directory for saving episode files
            control_endpoint: Control plane endpoint (defaults to CTRL_PUB_ENDPOINT)
            status_endpoint: Status plane endpoint (defaults to STATUS_PULL_ENDPOINT)
            heartbeat_interval: Seconds between heartbeats
        """
        super().__init__(
            node_id=node_id,
            control_endpoint=control_endpoint or CTRL_PUB_ENDPOINT,
            status_endpoint=status_endpoint or STATUS_PULL_ENDPOINT,
            heartbeat_interval=heartbeat_interval,
        )

        if not data_endpoints:
            raise ValueError("data_endpoints must be a non-empty list")

        self._data_endpoints = list(data_endpoints)
        self._storage_format = storage_format.lower()
        self._output_dir = output_dir

        # Data plane SUB sockets (one per endpoint)
        self._sub_data_sockets: list[zmq.Socket] = []

        # Buffers: dict[topic: bytes, list[(timestamp: float, data: Any)]]
        self._buffers: dict[bytes, list[tuple[float, Any]]] = defaultdict(list)
        self._buffer_lock = threading.Lock()

        # Episode counter
        self._episode_counter = 0

        # Initialize data sockets
        self._initialize_data_sockets()

        logger.info(f"RecorderNode initialized: {node_id}")
        logger.info(f"  Data endpoints: {data_endpoints}")
        logger.info(f"  Storage format: {storage_format}")
        logger.info(f"  Output dir: {output_dir}")

    # ----------------------
    # ZMQ setup/teardown
    # ----------------------
    def _initialize_data_sockets(self) -> None:
        """Create SUB sockets for each data endpoint and subscribe to all topics."""
        if self._ctx is None:
            self._ctx = zmq.Context.instance()

        for endpoint in self._data_endpoints:
            sub = self._ctx.socket(zmq.SUB)
            sub.connect(endpoint)
            # Subscribe to all topics
            sub.setsockopt(zmq.SUBSCRIBE, b"")
            self._sub_data_sockets.append(sub)
            logger.debug(f"Subscribed to data endpoint: {endpoint}")

        # Register all data sockets with poller
        if self._poller is not None:
            for sock in self._sub_data_sockets:
                self._poller.register(sock, zmq.POLLIN)

    def _cleanup_zmq(self) -> None:
        """Clean up data sockets in addition to base cleanup."""
        # Close data sockets
        for sock in self._sub_data_sockets:
            try:
                sock.close(linger=0)
            except Exception:
                pass
        self._sub_data_sockets.clear()

        # Call base cleanup
        super()._cleanup_zmq()

    # ----------------------
    # Main loop
    # ----------------------
    def _main_loop_iteration(self) -> None:
        """Poll data sockets and buffer messages when recording."""
        if self._poller is None:
            return

        # Poll for messages (non-blocking)
        events = dict(self._poller.poll(timeout=10))  # 10ms timeout

        # Check data sockets
        for sock in self._sub_data_sockets:
            if sock in events:
                self._handle_data_message(sock)

    def _handle_data_message(self, sock: zmq.Socket) -> None:
        """Receive and buffer a data message from a socket."""
        if not self.is_recording:
            # Drain messages but don't buffer when not recording
            try:
                sock.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                pass
            return

        try:
            # Receive 2-frame message: [topic, payload]
            frames = sock.recv_multipart(flags=zmq.NOBLOCK)
            if len(frames) != 2:
                return

            # Unpack message
            message = unpack_data_message(frames)
            topic = message.get("topic", "").encode("utf-8")
            timestamp = message.get("timestamp")
            data = message.get("data")

            if timestamp is None or data is None:
                return

            # Append to buffer (thread-safe)
            with self._buffer_lock:
                self._buffers[topic].append((float(timestamp), data))

        except zmq.Again:
            # No message available
            pass
        except Exception as e:
            # Non-fatal: skip malformed messages
            logger.debug(f"Error handling data message: {e}")

    # ----------------------
    # ManagedNode hooks
    # ----------------------
    def on_start(self) -> None:
        """Called when START command is received.

        Clears buffers to prepare for new recording session.
        """
        with self._buffer_lock:
            self._buffers.clear()
        logger.info(f"{self.node_id} ready for recording")

    def on_pause(self) -> None:
        """Called when PAUSE command is received."""
        logger.info(f"{self.node_id} paused")

    def on_stop(self) -> None:
        """Called when STOP command is received."""
        logger.info(f"{self.node_id} stopped")

    def on_start_recording(self) -> None:
        """Called when START_REC command is received.

        Clears all buffers to start fresh recording.
        """
        with self._buffer_lock:
            self._buffers.clear()
        logger.info(f"{self.node_id} started recording (buffers cleared)")

    def on_stop_recording(self) -> None:
        """Called when STOP_REC command is received.

        Deep copies buffers and spawns episode writer thread.
        """
        # Deep copy buffers for writer thread
        with self._buffer_lock:
            if not self._buffers:
                logger.warning("No data to save - buffers empty")
                return
            episode_buffers = copy.deepcopy(self._buffers)
            self._buffers.clear()

        # Increment episode counter
        self._episode_counter += 1
        episode_number = self._episode_counter

        logger.info(f"Saving episode {episode_number} ({len(episode_buffers)} topics)")

        # Spawn writer thread (non-blocking)
        try:
            writer = self._create_episode_writer(episode_buffers, episode_number)
            writer.start()
            # Don't wait for thread - it runs in background
        except Exception as e:
            logger.error(f"Failed to create episode writer: {e}")

    def on_shutdown(self) -> None:
        """Called during shutdown sequence.

        Calls on_finalize() for format-specific cleanup.
        """
        logger.info(f"{self.node_id} shutting down")
        try:
            self.on_finalize()
        except Exception as e:
            logger.error(f"Error during finalization: {e}")

    # ----------------------
    # Abstract methods
    # ----------------------
    @abstractmethod
    def _create_episode_writer(
        self,
        episode_buffers: dict[bytes, list[tuple[float, Any]]],
        episode_number: int,
    ) -> threading.Thread:
        """Create a thread for saving episode data.

        Args:
            episode_buffers: Deep copy of data buffers
            episode_number: Episode number for filename

        Returns:
            Thread instance ready to start

        Subclasses should implement format-specific writer creation.
        """
        pass

    def on_finalize(self) -> None:
        """Called during shutdown for format-specific cleanup.

        Override in subclasses to finalize datasets, flush buffers, etc.
        """
        pass

    # ----------------------
    # Buffer access
    # ----------------------
    def get_buffer_stats(self) -> dict[str, int]:
        """Get current buffer statistics.

        Returns:
            Dict mapping topic names to message counts
        """
        with self._buffer_lock:
            return {
                topic.decode("utf-8", errors="replace"): len(messages)
                for topic, messages in self._buffers.items()
            }

    @property
    def episode_count(self) -> int:
        """Get total number of recorded episodes."""
        return self._episode_counter
