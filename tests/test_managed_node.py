from __future__ import annotations

import json
from typing import Any
from unittest.mock import Mock

import pytest

from dexim.core.messages import (
    CTRL_ACTIVATE,
    CTRL_PREPARE,
    CTRL_START,
    CTRL_STOP,
    STATUS_ARMED,
    STATUS_READY,
    STATUS_STARTED,
    STATUS_STARTING,
    TOPIC_CTRL,
)
from dexim.core.nodes.managed import ManagedNode


class ManagedNodeHarness(ManagedNode):
    def __init__(self) -> None:
        self.node_id = "test"
        control_socket = Mock()
        control_socket.recv_multipart.return_value = [
            TOPIC_CTRL,
            CTRL_START.encode(),
        ]
        self._sub_control = control_socket
        self._countdown_active = False
        self._teleop_active = False
        self._countdown_duration = 0.0
        self._countdown_end_ts = 0.0
        self._last_reported_second = -1
        self._prepared_epoch_id: str | None = None
        self._armed_epoch_id: str | None = None
        self._active_epoch_id: str | None = None
        self._pending_transition: str | None = None
        self.is_publishing = False
        self.start_count = 0
        self.stop_count = 0
        self.activate_count = 0
        self.accept_start = True
        self.accept_activate = True
        self.reported_statuses: list[tuple[str, Any]] = []

    def on_standby(self) -> None: ...

    def on_start(self) -> bool:
        self.start_count += 1
        return self.accept_start

    def on_pause(self) -> None: ...

    def on_activate(self) -> bool:
        self.activate_count += 1
        return self.accept_activate

    def on_stop(self) -> None:
        self.stop_count += 1

    def on_shutdown(self) -> None: ...

    def on_start_recording(self) -> None: ...

    def on_stop_recording(self) -> None: ...

    def _main_loop_iteration(self) -> None: ...

    def report_status(self, status: str, info: Any = None) -> None:
        self.reported_statuses.append((status, info))

    def receive_control(self, command: str, payload: dict[str, object] | None = None) -> None:
        frames = [TOPIC_CTRL, command.encode()]
        if payload is not None:
            frames.append(json.dumps(payload).encode())
        self._sub_control.recv_multipart.return_value = frames
        self._handle_control_message()


def test_prepare_reports_epoch_readiness_without_activating_motion() -> None:
    node = ManagedNodeHarness()

    node.receive_control(CTRL_PREPARE, {"epoch_id": "epoch-1"})
    node.receive_control(CTRL_PREPARE, {"epoch_id": "epoch-1"})

    assert node.start_count == 1
    assert not node._teleop_active
    assert not node.is_publishing
    assert node.reported_statuses[-1] == (
        STATUS_READY,
        {
            "control_epoch_id": "epoch-1",
            "countdown_duration": 0.0,
            "preparation_ready": True,
        },
    )


def test_prepared_node_arms_then_activates_at_shared_boundaries(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    now = 100.0
    monkeypatch.setattr("dexim.core.nodes.managed.time.time", lambda: now)
    node = ManagedNodeHarness()
    node._countdown_duration = 3.0
    node.receive_control(CTRL_PREPARE, {"epoch_id": "epoch-1"})

    node.receive_control(
        CTRL_ACTIVATE,
        {"epoch_id": "epoch-1", "activation_time": 103.0},
    )

    assert node.start_count == 1
    assert node.activate_count == 0
    assert not node._teleop_active
    assert node.reported_statuses[-1][0] == STATUS_STARTING

    now = 102.9
    node._pre_loop_iteration()
    assert node.activate_count == 0
    assert not node._teleop_active

    now = 103.0
    node._pre_loop_iteration()
    node._pre_loop_iteration()
    assert node.activate_count == 1
    assert not node._teleop_active
    assert node._armed_epoch_id == "epoch-1"
    assert node.reported_statuses[-1][0] == STATUS_ARMED

    node.receive_control(
        CTRL_START,
        {"epoch_id": "epoch-1", "activation_time": 103.25},
    )
    now = 103.25
    node._pre_loop_iteration()

    assert node.activate_count == 1
    assert node._teleop_active
    assert node._active_epoch_id == "epoch-1"
    assert node.reported_statuses[-1][0] == STATUS_STARTED


def test_stop_cancels_shared_countdown_without_device_motion() -> None:
    node = ManagedNodeHarness()
    node.receive_control(CTRL_PREPARE, {"epoch_id": "epoch-1"})
    node.receive_control(
        CTRL_ACTIVATE,
        {"epoch_id": "epoch-1", "activation_time": 10_000_000_000.0},
    )

    node.receive_control(CTRL_STOP)

    assert not node._countdown_active
    assert not node._teleop_active
    assert node.activate_count == 0
    assert node.stop_count == 0
    assert node._prepared_epoch_id is None


def test_activation_rejection_never_grants_motion_authority() -> None:
    node = ManagedNodeHarness()
    node.accept_activate = False
    node.receive_control(CTRL_PREPARE, {"epoch_id": "epoch-1"})

    node.receive_control(
        CTRL_ACTIVATE,
        {"epoch_id": "epoch-1", "activation_time": 0.0},
    )

    assert node.activate_count == 1
    assert not node._teleop_active
    assert not node.is_publishing
    assert node.reported_statuses[-1][1] == {
        "control_epoch_id": "epoch-1",
        "activation_rejected": True,
    }


def test_heartbeat_snapshot_retains_control_epoch_identity() -> None:
    node = ManagedNodeHarness()
    node.receive_control(CTRL_PREPARE, {"epoch_id": "epoch-1"})

    info = node.get_status_info()

    assert info.control_epoch_id == "epoch-1"
    assert info.countdown_duration == 0.0
    assert info.preparation_ready


@pytest.mark.parametrize(
    ("countdown_active", "teleop_active"),
    [(True, False), (False, True), (True, True)],
)
def test_duplicate_start_is_ignored(
    countdown_active: bool,
    teleop_active: bool,
) -> None:
    node = ManagedNodeHarness()
    node._countdown_active = countdown_active
    node._teleop_active = teleop_active

    node._handle_control_message()

    assert node.start_count == 0
    assert node.reported_statuses == []


def test_start_from_idle_runs_start_hook_once() -> None:
    node = ManagedNodeHarness()

    node._handle_control_message()

    assert node.start_count == 1
    assert node._teleop_active
    assert node.is_publishing


def test_rejected_start_remains_inactive() -> None:
    node = ManagedNodeHarness()
    node.accept_start = False

    node._handle_control_message()

    assert node.start_count == 1
    assert not node._teleop_active
    assert not node.is_publishing
    assert node.reported_statuses[-1][1] == {"start_rejected": True}


def test_stop_then_start_establishes_a_second_control_epoch() -> None:
    node = ManagedNodeHarness()
    node._handle_control_message()

    node._sub_control.recv_multipart.return_value = [
        TOPIC_CTRL,
        CTRL_STOP.encode(),
    ]
    node._handle_control_message()

    node._sub_control.recv_multipart.return_value = [
        TOPIC_CTRL,
        CTRL_START.encode(),
    ]
    node._handle_control_message()

    assert node.start_count == 2
    assert node.stop_count == 1
    assert node._teleop_active
