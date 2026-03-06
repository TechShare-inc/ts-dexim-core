"""ManagedNode abstract base class for orchestrator-controlled nodes.

Responsibilities:
- Initialize ZMQ context and sockets for Control and Status planes
- Poll for control commands (START, PAUSE, SHUTDOWN, START_REC, STOP_REC)
- Maintain is_recording state and heartbeat reporting
- Provide hooks for subclasses to implement their work and lifecycle

This is the core base class for all orchestrator-controlled nodes.
"""

from __future__ import annotations

import abc
import time

import zmq
from dexim_core.messages import (
    CTRL_PAUSE,
    CTRL_PAUSE_PUB,
    CTRL_PUB_ENDPOINT,
    CTRL_SHUTDOWN,
    CTRL_START,
    CTRL_START_PUB,
    CTRL_START_REC,
    CTRL_STOP,
    CTRL_STOP_PUB,
    CTRL_STOP_REC,
    STATUS_ERROR,
    STATUS_HEALTHY,
    STATUS_INITIALIZED,
    STATUS_PAUSED,
    STATUS_PULL_ENDPOINT,
    STATUS_SHUTTING_DOWN,
    STATUS_STARTED,
    TOPIC_CTRL,
    pack_status_message,
)


class ManagedNode(abc.ABC):
    """Abstract base class for ZMQ-managed nodes."""

    def __init__(
        self,
        node_id: str,
        control_endpoint: str = CTRL_PUB_ENDPOINT,
        status_endpoint: str = STATUS_PULL_ENDPOINT,
        heartbeat_interval: float = 1.0,
    ) -> None:
        self.node_id = node_id
        self.heartbeat_interval = float(heartbeat_interval)

        # Runtime state
        self.is_recording: bool = False
        self.is_publishing: bool = True
        self._teleop_active: bool = False  # Teleoperation control gating
        self.running: bool = False
        self._last_heartbeat_ts: float = 0.0

        # ZMQ context and sockets
        self._ctx: zmq.Context | None = None
        self._sub_control: zmq.Socket | None = None
        self._push_status: zmq.Socket | None = None
        self._poller: zmq.Poller | None = None

        # Endpoints
        self._control_endpoint = control_endpoint
        self._status_endpoint = status_endpoint

        # Initialize
        self._initialize_zmq()
        self.report_status(STATUS_INITIALIZED)

    # ----------------------
    # Public lifecycle
    # ----------------------
    def run(self) -> None:
        """Main loop: poll control, iterate subclass work, send heartbeats."""
        self.running = True
        self.report_status(STATUS_STARTED)

        try:
            while self.running:
                self._poll_once(timeout_ms=5)  # Reduced from 10ms for faster loop
                # logger.debug("ManagedNode main loop iteration")
                self._main_loop_iteration()
                # logger.debug("ManagedNode completed main loop iteration")
                self.send_heartbeat_if_needed()
                # logger.debug("ManagedNode heartbeat check complete")
                # time.sleep(0.0001)  # Minimal yield to prevent CPU spin (0.1ms)
        except Exception:
            # Report error but re-raise for caller visibility
            self.report_status(STATUS_ERROR)
            raise
        finally:
            try:
                self.on_shutdown()
            finally:
                self._cleanup_zmq()

    # ----------------------
    # Abstract hooks
    # ----------------------
    @abc.abstractmethod
    def on_start(self) -> None:
        """Called when START command is received (node lifecycle)."""

    @abc.abstractmethod
    def on_pause(self) -> None:
        """Called when PAUSE command is received (hold position, quick resume)."""

    @abc.abstractmethod
    def on_stop(self) -> None:
        """Called when STOP command is received (go to safe position)."""

    @abc.abstractmethod
    def on_shutdown(self) -> None:
        """Called during shutdown sequence (before sockets are closed)."""

    @abc.abstractmethod
    def on_start_recording(self) -> None:
        """Called when START_REC command is received (recording control)."""

    @abc.abstractmethod
    def on_stop_recording(self) -> None:
        """Called when STOP_REC command is received (recording control)."""

    # Deprecated hook for backward compatibility
    def on_stop_save(self) -> None:
        """Deprecated: Use on_stop_recording() instead."""
        self.on_stop_recording()

    @abc.abstractmethod
    def _main_loop_iteration(self) -> None:
        """Subclass work executed each loop iteration."""

    # ----------------------
    # ZMQ setup/teardown
    # ----------------------
    def _initialize_zmq(self) -> None:
        ctx = zmq.Context.instance()
        self._ctx = ctx

        # Control: SUB
        sub = ctx.socket(zmq.SUB)
        sub.connect(self._control_endpoint)
        sub.setsockopt(zmq.SUBSCRIBE, TOPIC_CTRL)
        self._sub_control = sub

        # Status: PUSH
        push = ctx.socket(zmq.PUSH)
        push.connect(self._status_endpoint)
        self._push_status = push

        # Poller for control
        poller = zmq.Poller()
        poller.register(sub, zmq.POLLIN)
        self._poller = poller

    def _cleanup_zmq(self) -> None:
        # Send shutting down status (best-effort)
        try:
            self.report_status(STATUS_SHUTTING_DOWN)
        except Exception:
            pass

        try:
            if self._sub_control is not None:
                self._sub_control.close(linger=0)
        finally:
            self._sub_control = None

        try:
            if self._push_status is not None:
                self._push_status.close(linger=0)
        finally:
            self._push_status = None

        try:
            if self._ctx is not None:
                # Do not terminate global instance to avoid impacting others
                # Allow GC to handle context when process exits
                pass
        finally:
            self._ctx = None

        self._poller = None

    # ----------------------
    # Control handling
    # ----------------------
    def _poll_once(self, timeout_ms: int) -> None:
        if self._poller is None:
            return
        events = dict(self._poller.poll(timeout=timeout_ms))
        if self._sub_control in events:
            self._handle_control_message()

    def _handle_control_message(self) -> None:
        if self._sub_control is None:
            return
        # Control messages are [topic, command]
        parts = self._sub_control.recv_multipart(flags=zmq.NOBLOCK)
        if not parts or len(parts) < 2:
            return
        topic, command = parts[0], parts[1]
        if topic != TOPIC_CTRL:
            return

        cmd = command.decode("utf-8", errors="ignore").strip().upper()

        # Teleoperation control
        if cmd == CTRL_START:
            # START: Begin teleoperation, capture reference pose
            # Call on_start() BEFORE setting _teleop_active to ensure
            # reference poses are captured before processing begins
            self.on_start()
            self._teleop_active = True
            self.is_publishing = True
            self.report_status(STATUS_STARTED)
            print(f"{self.node_id} started teleoperation")

        elif cmd == CTRL_PAUSE:
            # PAUSE: Pause teleoperation, hold current position (quick resume)
            self._teleop_active = False
            self.on_pause()
            self.report_status(STATUS_PAUSED)
            print(f"{self.node_id} paused")

        elif cmd == CTRL_STOP:
            # STOP: Stop teleoperation, go to safe position
            self._teleop_active = False
            self.on_stop()
            self.report_status(STATUS_PAUSED)
            print(f"{self.node_id} stopped (safe position)")

        elif cmd == CTRL_START_PUB:
            # START_PUB: Resume publishing
            self.is_publishing = True
            self.report_status(STATUS_HEALTHY)
            print(f"{self.node_id} publishing resumed")

        elif cmd == CTRL_PAUSE_PUB:
            # PAUSE_PUB: Temporarily pause publishing
            self.is_publishing = False
            self.report_status(STATUS_HEALTHY)
            print(f"{self.node_id} publishing paused")

        elif cmd == CTRL_STOP_PUB:
            # STOP_PUB: Stop publishing
            self.is_publishing = False
            self.report_status(STATUS_HEALTHY)
            print(f"{self.node_id} publishing stopped")

        elif cmd == CTRL_SHUTDOWN:
            # SHUTDOWN: Node stops and exits
            self.is_recording = False
            self.running = False
            # on_shutdown called in finally of run()
            print(f"{self.node_id} shutting down")

        # Recording control (independent of node state)
        elif cmd == CTRL_START_REC:
            # START_REC: Begin recording/data collection
            self.is_recording = True
            self.on_start_recording()
            self.report_status(STATUS_HEALTHY)
            print(f"{self.node_id} started recording")

        elif cmd == CTRL_STOP_REC:
            # STOP_REC: Stop recording/data collection
            self.is_recording = False
            self.on_stop_recording()
            self.report_status(STATUS_HEALTHY)
            print(f"{self.node_id} stopped recording")

        else:
            # Unknown command: ignore but remain healthy
            self.report_status(STATUS_HEALTHY)

    # ----------------------
    # Status/heartbeat
    # ----------------------
    def report_status(self, status: str, info: dict | None = None) -> None:
        if self._push_status is None:
            return
        payload = pack_status_message(
            node_id=self.node_id,
            status=status,
            is_recording=self.is_recording,
            timestamp=time.time(),
            info={"is_publishing": self.is_publishing, **(info or {})},
        )
        # Status plane is PUSH → PULL; single frame payload
        self._push_status.send(payload, flags=zmq.DONTWAIT)

    def send_heartbeat_if_needed(self) -> None:
        now = time.time()
        if now - self._last_heartbeat_ts >= self.heartbeat_interval:
            self.report_status(STATUS_HEALTHY)
            self._last_heartbeat_ts = now
