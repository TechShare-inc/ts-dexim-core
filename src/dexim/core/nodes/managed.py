"""ManagedNode abstract base class for orchestrator-controlled nodes.

Responsibilities:
- Initialize ZMQ context and sockets for Control and Status planes
- Poll for lifecycle, Control Epoch, and recording commands
- Maintain is_recording state and heartbeat reporting
- Provide hooks for subclasses to implement their work and lifecycle

This is the core base class for all orchestrator-controlled nodes.
"""

from __future__ import annotations

import abc
import json
import time
from typing import Any, Literal

import zmq

from dexim.core.messages import (
    CTRL_ACTIVATE,
    CTRL_DISCARD_REC,
    CTRL_PAUSE,
    CTRL_PAUSE_PUB,
    CTRL_PREPARE,
    CTRL_PUB_ENDPOINT,
    CTRL_SET_TASK,
    CTRL_SHUTDOWN,
    CTRL_STANDBY,
    CTRL_START,
    CTRL_START_PUB,
    CTRL_START_REC,
    CTRL_STOP,
    CTRL_STOP_PUB,
    CTRL_STOP_REC,
    STATUS_ARMED,
    STATUS_ERROR,
    STATUS_HEALTHY,
    STATUS_INITIALIZED,
    STATUS_PAUSED,
    STATUS_PREPARING,
    STATUS_PULL_ENDPOINT,
    STATUS_READY,
    STATUS_SHUTTING_DOWN,
    STATUS_STANDBY,
    STATUS_STARTED,
    STATUS_STARTING,
    TOPIC_CTRL,
    StatusInfo,
    pack_status_message,
)

# Poll timeout for the ZMQ control socket. Short enough for responsive command
# handling without wasting CPU cycles.
_CTRL_POLL_TIMEOUT_MS: int = 5
_PendingTransition = Literal["legacy", "arm", "grant"]


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

        # Countdown state (used by subclasses that want a start delay)
        self._countdown_duration: float = 0.0  # Seconds; 0 = no countdown
        self._countdown_end_ts: float = 0.0
        self._countdown_active: bool = False
        self._last_reported_second: int = -1
        self._prepared_epoch_id: str | None = None
        self._armed_epoch_id: str | None = None
        self._active_epoch_id: str | None = None
        self._pending_transition: _PendingTransition | None = None

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
        """Main loop: enter standby, poll control, iterate subclass work, send heartbeats."""
        self.running = True
        self.on_standby()
        self.report_status(STATUS_STANDBY)

        try:
            while self.running:
                self._poll_once(timeout_ms=_CTRL_POLL_TIMEOUT_MS)
                self._pre_loop_iteration()
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
    def on_standby(self) -> None:
        """Called when entering STANDBY state (on run() entry or CTRL_STANDBY command).

        The node is in a ready-but-idle state: interface connected, main loop
        running, but teleoperation is not active.  This is the default state
        after ``run()`` is called and before ``CTRL_START`` is received.

        Subclasses should connect hardware interfaces and perform one-time
        setup that does not require runtime data (e.g., tracker reference
        poses).  Publishing should remain deactivated.
        """

    @abc.abstractmethod
    def on_start(self) -> bool | None:
        """Prepare for a legacy START or orchestrated Control Epoch.

        Return ``False`` when runtime prerequisites are unavailable. The
        managed lifecycle then remains inactive; ``None`` preserves the
        historical successful-hook behavior for existing nodes.
        """

    def on_activate(self) -> bool | None:
        """Finalize preparation at the Control Epoch activation boundary.

        Subclasses override this for work that must observe the same boundary
        as motion authority, such as capturing a tracker reference. Returning
        ``False`` rejects activation while keeping teleoperation inactive.
        """
        return True

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

    def on_discard_recording(self) -> None:
        """Called when DISCARD_REC command is received. Default: no-op.

        Override to discard the current episode buffer without writing.
        """

    def on_set_task(self, task_info: dict[str, Any]) -> None:
        """Called when SET_TASK command is received. Default: no-op.

        Args:
            task_info: Dict with ``task_id`` and ``task_description`` keys.
        """

    # Deprecated hook for backward compatibility
    def on_stop_save(self) -> None:
        """Deprecated: Use on_stop_recording() instead."""
        self.on_stop_recording()

    @abc.abstractmethod
    def _main_loop_iteration(self) -> None:
        """Subclass work executed each loop iteration."""

    # ----------------------
    # Countdown helpers
    # ----------------------

    def _start_countdown(self, duration: float) -> None:
        """Begin a start countdown of *duration* seconds.

        During the countdown the main loop continues to run, control messages
        are still polled (so CTRL_STOP aborts the countdown), and status
        reports include ``STARTING`` with a ``countdown_remaining`` field.

        This helper preserves legacy ``START`` behavior. Coordinated Control
        Epochs use an orchestrator-provided wall-clock boundary instead.

        Args:
            duration: Countdown length in seconds.  Pass ``0.0`` to skip.
        """
        self._countdown_duration = duration
        self._countdown_end_ts = time.time() + duration
        self._countdown_active = duration > 0.0
        self._last_reported_second = -1
        self._pending_transition = "legacy"

    def _tick_countdown(self) -> bool:
        """Advance the countdown timer.

        Returns:
            ``True`` when the countdown has expired (or was never active).
        """
        if not self._countdown_active:
            return True
        if time.time() >= self._countdown_end_ts:
            self._countdown_active = False
            return True
        return False

    def _clear_control_epoch_state(self) -> None:
        """Clear pending and active Control Epoch authority."""
        self._countdown_active = False
        self._prepared_epoch_id = None
        self._armed_epoch_id = None
        self._active_epoch_id = None
        self._pending_transition = None

    def _schedule_epoch_transition(
        self,
        epoch_id: str,
        activation_time: float,
        transition: Literal["arm", "grant"],
    ) -> bool:
        """Schedule one shared boundary and report countdown truth.

        Returns:
            ``True`` when the transition is pending, or ``False`` when its
            boundary has already arrived and the caller should apply it now.
        """
        self._countdown_end_ts = activation_time
        self._countdown_active = activation_time > time.time()
        self._last_reported_second = -1
        self._pending_transition = transition
        if self._countdown_active:
            self.report_status(
                STATUS_STARTING,
                {
                    "control_epoch_id": epoch_id,
                    "activation_time": activation_time,
                    "countdown_remaining": max(0.0, activation_time - time.time()),
                },
            )
        return self._countdown_active

    def _arm_prepared_epoch(self, epoch_id: str | None) -> bool:
        """Run activation-time work without granting motion authority."""
        activation_accepted = self.on_activate()
        if activation_accepted is False:
            self._clear_control_epoch_state()
            self.is_publishing = False
            self.report_status(
                STATUS_PAUSED,
                {
                    "control_epoch_id": epoch_id,
                    "activation_rejected": True,
                },
            )
            print(f"{self.node_id} activation rejected -- prerequisites unavailable")
            return False

        self._countdown_active = False
        self._armed_epoch_id = epoch_id
        self._prepared_epoch_id = None
        self._pending_transition = None
        if epoch_id is not None:
            self.report_status(STATUS_ARMED, {"control_epoch_id": epoch_id})
        return True

    def _grant_armed_epoch(self, epoch_id: str | None) -> None:
        """Grant motion authority after every participant is armed."""
        self._countdown_active = False
        self._teleop_active = True
        self.is_publishing = True
        self._active_epoch_id = epoch_id
        self._prepared_epoch_id = None
        self._armed_epoch_id = None
        self._pending_transition = None
        status_info = {"control_epoch_id": epoch_id} if epoch_id is not None else None
        self.report_status(STATUS_STARTED, status_info)

    # ----------------------
    # Pre-loop hook + auto-prepare
    # ----------------------

    def _pre_loop_iteration(self) -> None:
        """Run before ``_main_loop_iteration()`` each tick.

        Responsibilities:
        1. Tick the current arm or activation countdown.
        2. Call ``_auto_prepare()`` when in STANDBY (no countdown, no teleop).
        """
        if self._countdown_active:
            if self._tick_countdown():
                transition = self._pending_transition
                if transition == "arm":
                    if self._arm_prepared_epoch(self._prepared_epoch_id):
                        print(f"{self.node_id} armed for shared activation")
                elif transition == "grant":
                    self._grant_armed_epoch(self._armed_epoch_id)
                    print(f"{self.node_id} shared activation complete")
                elif self._arm_prepared_epoch(None):
                    self._grant_armed_epoch(None)
                    print(f"{self.node_id} countdown complete -- teleoperation active")
            else:
                # Still counting down -- report progress at second boundaries.
                remaining = self._countdown_end_ts - time.time()
                second = int(remaining)
                if second != self._last_reported_second:
                    self._last_reported_second = second
                    self.report_status(
                        STATUS_STARTING,
                        {"countdown_remaining": round(remaining, 1)},
                    )
                    print(f"{self.node_id} starting in {max(remaining, 0.0):.0f}s")
            return  # Don't auto-prepare during countdown

        if not self._teleop_active:
            auto_cmd = self._auto_prepare()
            if auto_cmd == CTRL_START:
                # Auto-start: transition to RUNNING without countdown.
                start_accepted = self.on_start()
                if start_accepted is not False and self._arm_prepared_epoch(None):
                    self._grant_armed_epoch(None)
                    print(f"{self.node_id} auto-started -- {auto_cmd}")

    def _auto_prepare(self) -> str | None:
        """Override in subclasses to perform automatic preparation during STANDBY.

        Called each tick while the node is in STANDBY (no countdown active,
        teleop inactive).  Use this to check data availability, warm up
        subsystems, or log readiness status -- everything that should happen
        automatically without waiting for ``CTRL_START``.

        When the node is ready to run, return a control command string
        (e.g., ``CTRL_START``) to trigger an automatic transition.  The
        transition bypasses any countdown so the node moves directly to
        RUNNING.

        Returns:
            A control command to auto-apply, or ``None`` to stay in STANDBY.
        """
        return None

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
        push.setsockopt(zmq.SNDHWM, 10)  # Prevent unbounded buffering when no consumer
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
        # Control messages are [topic, command, optional JSON payload].
        parts = self._sub_control.recv_multipart(flags=zmq.NOBLOCK)
        if not parts or len(parts) < 2:
            return
        topic, command = parts[0], parts[1]
        if topic != TOPIC_CTRL:
            return

        cmd = command.decode("utf-8", errors="ignore").strip().upper()
        control_payload: dict[str, Any] = {}
        if len(parts) > 2:
            try:
                decoded = json.loads(parts[2])
            except (json.JSONDecodeError, UnicodeDecodeError):
                decoded = {}
            if isinstance(decoded, dict):
                control_payload = decoded

        # Teleoperation control
        if cmd == CTRL_PREPARE:
            epoch_id = control_payload.get("epoch_id")
            if not isinstance(epoch_id, str) or not epoch_id:
                return
            readiness = {
                "control_epoch_id": epoch_id,
                "countdown_duration": self._countdown_duration,
                "preparation_ready": True,
            }
            if self._prepared_epoch_id == epoch_id:
                self.report_status(STATUS_READY, readiness)
                return
            if self._armed_epoch_id == epoch_id:
                self.report_status(STATUS_ARMED, {"control_epoch_id": epoch_id})
                return
            if self._active_epoch_id == epoch_id:
                self.report_status(STATUS_STARTED, {"control_epoch_id": epoch_id})
                return
            if (
                self._prepared_epoch_id is not None
                or self._armed_epoch_id is not None
                or self._active_epoch_id is not None
                or self._countdown_active
                or self._teleop_active
            ):
                return

            self.report_status(
                STATUS_PREPARING,
                {"control_epoch_id": epoch_id, "preparation_ready": False},
            )
            start_accepted = self.on_start()
            if start_accepted is False:
                self.is_publishing = False
                self.report_status(
                    STATUS_PAUSED,
                    {
                        "control_epoch_id": epoch_id,
                        "start_rejected": True,
                        "preparation_ready": False,
                    },
                )
                return
            self._prepared_epoch_id = epoch_id
            self.report_status(STATUS_READY, readiness)

        elif cmd == CTRL_ACTIVATE:
            epoch_id = control_payload.get("epoch_id")
            activation_time = control_payload.get("activation_time")
            if (
                isinstance(epoch_id, str)
                and epoch_id
                and isinstance(activation_time, (int, float))
            ):
                if self._armed_epoch_id == epoch_id:
                    self.report_status(STATUS_ARMED, {"control_epoch_id": epoch_id})
                    return
                if self._prepared_epoch_id != epoch_id:
                    return

                if not self._schedule_epoch_transition(
                    epoch_id, float(activation_time), "arm"
                ):
                    self._arm_prepared_epoch(epoch_id)
                return

        elif cmd == CTRL_START:
            epoch_id = control_payload.get("epoch_id")
            activation_time = control_payload.get("activation_time")
            if (
                isinstance(epoch_id, str)
                and epoch_id
                and isinstance(activation_time, (int, float))
            ):
                if self._active_epoch_id == epoch_id:
                    self.report_status(STATUS_STARTED, {"control_epoch_id": epoch_id})
                    return
                if self._armed_epoch_id != epoch_id:
                    return

                if not self._schedule_epoch_transition(
                    epoch_id, float(activation_time), "grant"
                ):
                    self._grant_armed_epoch(epoch_id)
                return

            # Ignore duplicate START commands while the node is already
            # starting or running. This prevents repeated reference capture.
            if self._countdown_active or self._teleop_active:
                return

            # Legacy START: prepare, then use the node-local countdown.
            start_accepted = self.on_start()
            if start_accepted is False:
                self.is_publishing = False
                self.report_status(STATUS_PAUSED, {"start_rejected": True})
                print(f"{self.node_id} start rejected -- prerequisites unavailable")
                return
            if self._countdown_duration > 0.0:
                # Start countdown -- _pre_loop_iteration will transition
                # to RUNNING when it expires.
                self._start_countdown(self._countdown_duration)
                self.report_status(
                    STATUS_STARTING, {"countdown_remaining": self._countdown_duration}
                )
                print(
                    f"{self.node_id} start requested -- countdown {self._countdown_duration:.0f}s"
                )
            else:
                if self._arm_prepared_epoch(None):
                    self._grant_armed_epoch(None)
                    print(f"{self.node_id} started teleoperation")

        elif cmd == CTRL_PAUSE:
            # PAUSE: Pause teleoperation, hold current position (quick resume)
            self._teleop_active = False
            self._clear_control_epoch_state()
            self.on_pause()
            self.report_status(STATUS_PAUSED)
            print(f"{self.node_id} paused")

        elif cmd == CTRL_STOP:
            # STOP: Stop teleoperation, cancel countdown, go to safe position
            was_active = self._teleop_active
            self._teleop_active = False
            self._clear_control_epoch_state()
            if was_active:
                self.on_stop()
            self.report_status(STATUS_PAUSED)
            if was_active:
                print(f"{self.node_id} stopped (safe position)")
            else:
                print(f"{self.node_id} pending start cancelled")

        elif cmd == CTRL_STANDBY:
            # STANDBY: Return to ready-but-idle state (keep interface connected).
            if self._teleop_active or self._countdown_active:
                self._teleop_active = False
                self.is_publishing = False
            self._clear_control_epoch_state()
            self.on_standby()
            self.report_status(STATUS_STANDBY)
            print(f"{self.node_id} entered standby")

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
            # SHUTDOWN: Stop teleop safely, cancel countdown, then exit.
            if self._teleop_active or self._countdown_active:
                self._teleop_active = False
                self.on_stop()
            self._clear_control_epoch_state()
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

        elif cmd == CTRL_DISCARD_REC:
            # DISCARD_REC: Discard current episode without writing
            self.is_recording = False
            self.on_discard_recording()
            self.report_status(STATUS_HEALTHY)
            print(f"{self.node_id} discarded recording")

        elif cmd == CTRL_SET_TASK:
            # SET_TASK: Update active task metadata (optional JSON payload in frame 2)
            payload = parts[2] if len(parts) > 2 else b"{}"
            try:
                task_info = json.loads(payload)
            except json.JSONDecodeError:
                task_info = {}
            self.on_set_task(task_info)
            self.report_status(STATUS_HEALTHY)
            print(f"{self.node_id} task set to {task_info.get('task_id', '')}")

        else:
            # Unknown command: ignore but remain healthy
            self.report_status(STATUS_HEALTHY)

    # ----------------------
    # Status/heartbeat
    # ----------------------

    def get_status_info(self) -> StatusInfo:
        """Return a snapshot of the node's runtime state.

        Subclasses override this to add type-specific fields
        (robot variant, pipeline stats, hardware connection, etc.).
        The base implementation covers fields managed by
        ``ManagedNode`` itself.

        Returns:
            A ``StatusInfo`` with base fields populated.
        """
        countdown_remaining: float | None = None
        activation_time: float | None = None
        if self._countdown_active:
            remaining = self._countdown_end_ts - time.time()
            countdown_remaining = max(0.0, remaining)
            activation_time = self._countdown_end_ts

        control_epoch_id = (
            self._active_epoch_id
            or self._armed_epoch_id
            or self._prepared_epoch_id
            or ""
        )

        return StatusInfo(
            is_publishing=self.is_publishing,
            teleop_active=self._teleop_active,
            countdown_active=self._countdown_active,
            countdown_remaining=countdown_remaining,
            control_epoch_id=control_epoch_id,
            activation_time=activation_time,
            countdown_duration=self._countdown_duration,
            preparation_ready=self._prepared_epoch_id is not None,
        )

    def report_status(
        self,
        status: str,
        info: dict[str, Any] | StatusInfo | None = None,
    ) -> None:
        """Publish a status update on the ZMQ status plane.

        Args:
            status: One of the ``STATUS_*`` constants.
            info: Extra fields as a ``dict`` or ``StatusInfo``.
                When ``None``, ``get_status_info()`` is called
                automatically.
        """
        if self._push_status is None:
            return

        resolved_info: dict[str, Any] | StatusInfo
        if info is not None:
            # Caller-supplied info (e.g. countdown, error context)
            # takes precedence but still includes is_publishing.
            if isinstance(info, StatusInfo):
                resolved_info = info
            else:
                resolved_info = {
                    "is_publishing": self.is_publishing,
                    **info,
                }
        else:
            # No explicit info -- build from get_status_info().
            resolved_info = self.get_status_info()

        payload = pack_status_message(
            node_id=self.node_id,
            status=status,
            is_recording=self.is_recording,
            timestamp=time.time(),
            info=resolved_info,
        )
        # Status plane is PUSH -> PULL; single frame payload.
        # DONTWAIT + low SNDHWM means this drops silently when no
        # orchestrator is consuming status messages -- non-fatal.
        try:
            self._push_status.send(payload, flags=zmq.DONTWAIT)
        except zmq.Again:
            pass  # Expected when no PULL consumer or buffer is full
        except zmq.ZMQError:
            pass  # Socket may be in a bad state during shutdown

    def send_heartbeat_if_needed(self) -> None:
        if self._countdown_active:
            return  # Status reported by _pre_loop_iteration during countdown
        now = time.time()
        if now - self._last_heartbeat_ts >= self.heartbeat_interval:
            self.report_status(STATUS_HEALTHY, info=self.get_status_info())
            self._last_heartbeat_ts = now
