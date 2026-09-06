from __future__ import annotations

from collections import deque

import pytest

from dexim.core.collaborators import tracker_receiver as tracker_module
from dexim.core.collaborators.tracker_receiver import TrackerReceiver
from dexim.core.messages import RigidPose


class FakeSubscriber:
    def __init__(self, samples: list[RigidPose | None]) -> None:
        self.samples = deque(samples)

    def read_latest(self) -> RigidPose | None:
        return self.samples.popleft()


def pose(timestamp: float) -> RigidPose:
    return RigidPose(
        tracker_id="tracker",
        tracker_type="left_hand",
        position=(0.0, 0.0, 0.0),
        rotation=(1.0, 0.0, 0.0, 0.0),
        timestamp=timestamp,
    )


def test_reference_capture_requires_consecutive_fresh_samples(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(tracker_module.time, "time", lambda: 10.0)
    monkeypatch.setattr(tracker_module.time, "sleep", lambda _seconds: None)
    receiver = TrackerReceiver(
        FakeSubscriber([pose(9.91), None, pose(9.92), pose(9.93), pose(9.94)]),
        "left_hand",
    )

    assert receiver.capture_reference(attempts=5, required_consecutive=3)
    assert receiver.reference_pose == pose(9.94)


def test_reference_capture_rejects_repeated_timestamp(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(tracker_module.time, "time", lambda: 10.0)
    monkeypatch.setattr(tracker_module.time, "sleep", lambda _seconds: None)
    receiver = TrackerReceiver(
        FakeSubscriber([pose(9.91), pose(9.91), pose(9.92)]),
        "left_hand",
    )

    assert not receiver.capture_reference(attempts=3, required_consecutive=3)
    assert receiver.reference_pose is None
