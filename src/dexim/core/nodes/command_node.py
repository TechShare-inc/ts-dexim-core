"""CommandNode base class for publishing commands to the data plane.

Extends ManagedNode to provide command publishing capabilities. This is useful
for nodes that need to send control messages or commands to other nodes via
the data plane (e.g., orchestrators, control systems).
"""

from __future__ import annotations

import time
from queue import Empty, Queue

import zmq
from dexim.core.messages import pack_data_message
from dexim.core.nodes.managed import ManagedNode
from loguru import logger


class CommandNode(ManagedNode):
    """Base class for nodes that publish commands to the data plane.

    This node manages a ZMQ PUB socket for publishing commands and provides
    thread-safe command queuing. Subclasses can queue commands via send_command()
    and they will be published automatically in the main loop.
    """

    def __init__(
        self,
        node_id: str,
        data_endpoint: str,
        topic: str,
        *,
        bind: bool = True,
        control_endpoint: str | None = None,
        status_endpoint: str | None = None,
        heartbeat_interval: float = 1.0,
    ) -> None:
        """
        Initialize command node.

        Args:
            node_id: Unique identifier for this node
            data_endpoint: ZMQ endpoint for data plane publishing
            topic: Topic to publish commands to
            bind: If True, bind to endpoint; if False, connect to endpoint
            control_endpoint: Control plane endpoint (default: tcp://localhost:5550)
            status_endpoint: Status plane endpoint (default: tcp://localhost:5551)
            heartbeat_interval: Heartbeat interval in seconds
        """
        super().__init__(
            node_id=node_id,
            control_endpoint=control_endpoint or "tcp://localhost:5550",
            status_endpoint=status_endpoint or "tcp://localhost:5551",
            heartbeat_interval=heartbeat_interval,
        )

        self._data_endpoint = data_endpoint
        self._topic = topic.encode("utf-8") if isinstance(topic, str) else topic
        self._bind = bind

        # Command queue for thread-safe communication
        self._command_queue: Queue[str | None] = Queue()

        # Data plane socket (initialized after ManagedNode sets up context)
        self._pub: zmq.Socket | None = None
        self._initialize_data_socket()

        logger.info(
            f"{self.__class__.__name__} initialized: "
            f"endpoint={data_endpoint}, topic={topic}, bind={bind}"
        )

    def _initialize_data_socket(self) -> None:
        """Initialize PUB socket for data plane."""
        if self._ctx is None:
            logger.warning("ZMQ context not available, skipping socket initialization")
            return

        pub = self._ctx.socket(zmq.PUB)
        pub.setsockopt(zmq.LINGER, 0)
        pub.setsockopt(zmq.SNDHWM, 100)

        if self._bind:
            pub.bind(self._data_endpoint)
            logger.info(f"Data PUB socket bound to {self._data_endpoint}")
        else:
            pub.connect(self._data_endpoint)
            logger.info(f"Data PUB socket connected to {self._data_endpoint}")

        self._pub = pub

    def send_command(self, command: str) -> bool:
        """
        Queue a command for publishing.

        This method is thread-safe and can be called from any thread.

        Args:
            command: Command string to publish

        Returns:
            True if command was queued successfully, False if command is empty
        """
        if not command or not command.strip():
            return False

        self._command_queue.put(command)
        return True

    def _publish_command(self, command: str) -> None:
        """
        Publish a command to the data plane.

        Args:
            command: Command string to publish
        """
        if self._pub is None:
            logger.warning("PUB socket not available, cannot publish")
            return

        try:
            topic_frame, payload_frame = pack_data_message(
                self._topic, time.time(), command
            )
            self._pub.send_multipart([topic_frame, payload_frame], flags=zmq.DONTWAIT)
            logger.debug(f"Published command: {command[:50]}...")
        except zmq.Again:
            pass  # Non-fatal: no subscriber or HWM reached
        except zmq.ZMQError as e:
            logger.error(f"Failed to publish command: {e}")
        except Exception as e:
            logger.error(f"Unexpected error publishing command: {e}")

    # ----------------------
    # ManagedNode lifecycle hooks
    # ----------------------
    def on_standby(self) -> None:
        """Called when entering STANDBY state."""
        logger.info(f"{self.node_id} entering standby")

    def on_start(self) -> None:
        """Called when START command is received."""
        logger.info(f"{self.node_id} started")

    def on_pause(self) -> None:
        """Called when PAUSE command is received."""
        logger.info(f"{self.node_id} paused")

    def on_start_recording(self) -> None:
        """Called when START_REC command is received."""
        logger.info(f"{self.node_id} started recording")

    def on_stop_recording(self) -> None:
        """Called when STOP_REC command is received."""
        logger.info(f"{self.node_id} stopped recording")

    def on_stop(self) -> None:
        """Called when STOP command is received."""
        logger.info(f"{self.node_id} stopped")

    def on_shutdown(self) -> None:
        """Called during shutdown sequence."""
        logger.info(f"{self.node_id} shutting down")
        if self._pub is not None:
            try:
                self._pub.close(linger=0)
            except Exception as e:
                logger.error(f"Error closing PUB socket: {e}")
            finally:
                self._pub = None

    def _main_loop_iteration(self) -> None:
        """Process queued commands and publish them."""
        # Only publish when enabled
        if not self.is_publishing:
            time.sleep(0.01)
            return

        # Check for queued commands (non-blocking)
        try:
            command = self._command_queue.get_nowait()
            if command is None:
                # Shutdown signal
                self.running = False
                return
            self._publish_command(command)
        except Empty:
            # No commands to process
            pass
