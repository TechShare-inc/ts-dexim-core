from __future__ import annotations

from typing import Any
from unittest.mock import Mock

import pytest

from dexim.core.messages import CTRL_START, TOPIC_CTRL
from dexim.core.nodes.managed import ManagedNode


class ManagedNodeHarness(ManagedNode):
    def __init__(self) -> None:
        self.node_id = "test"
        self._sub_control = Mock()
        self._sub_control.recv_multipart.return_value = [
            TOPIC_CTRL,
            CTRL_START.encode(),
        ]
        self._countdown_active = False
        self._teleop_active = False
        self._countdown_duration = 0.0
        self.is_publishing = False
        self.start_count = 0
        self.reported_statuses: list[tuple[str, Any]] = []

    def on_standby(self) -> None: ...

    def on_start(self) -> None:
        self.start_count += 1

    def on_pause(self) -> None: ...

    def on_stop(self) -> None: ...

    def on_shutdown(self) -> None: ...

    def on_start_recording(self) -> None: ...

    def on_stop_recording(self) -> None: ...

    def _main_loop_iteration(self) -> None: ...

    def report_status(self, status: str, info: Any = None) -> None:
        self.reported_statuses.append((status, info))


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
