from __future__ import annotations

import time
from typing import Any

import pytest

from dexim.core.collaborators import ErgonomicsReceiver
from dexim.core.messages import ErgonomicsState


class FakeSubscriber:
    def __init__(self, batches: list[list[ErgonomicsState]]) -> None:
        self.batches = batches
        self.wait_kwargs: dict[str, Any] | None = None
        self.closed = False

    def read_latest_batch(self) -> list[ErgonomicsState]:
        return self.batches.pop(0) if self.batches else []

    def wait_for_sensor(self, **kwargs: Any) -> bool:
        self.wait_kwargs = kwargs
        return True

    def close(self) -> None:
        self.closed = True


def state(glove_id: int, timestamp: float | None = None) -> ErgonomicsState:
    return ErgonomicsState(
        glove_id=glove_id,
        side="left",
        timestamp=time.time() if timestamp is None else timestamp,
        values=[1.0],
    )


@pytest.mark.parametrize("configured_id", ["101", "0x65", 101])
def test_receive_matches_decimal_hexadecimal_and_integer_ids(
    configured_id: str | int,
) -> None:
    matching = state(101)
    subscriber = FakeSubscriber([[state(100), matching]])
    receiver = ErgonomicsReceiver(subscriber, "left", configured_id)

    assert receiver.receive() is matching


def test_receive_tracks_matching_data_and_missing_streak() -> None:
    matching = state(101, time.time() - 0.25)
    subscriber = FakeSubscriber([[], [state(100), matching]])
    receiver = ErgonomicsReceiver(subscriber, "left", 101)

    assert receiver.receive() is None
    assert receiver.no_data_streak == 1
    assert receiver.receive() is matching
    assert receiver.no_data_streak == 0
    assert receiver.last_data_age_sec == pytest.approx(0.25, abs=0.1)


def test_receive_without_configured_id_uses_first_state() -> None:
    first = state(100)
    receiver = ErgonomicsReceiver(FakeSubscriber([[first, state(101)]]), "left")

    assert receiver.receive() is first


def test_invalid_id_does_not_match() -> None:
    receiver = ErgonomicsReceiver(FakeSubscriber([[state(101)]]), "left", "invalid")

    assert receiver.receive() is None
    assert receiver.no_data_streak == 1


def test_wait_and_close_delegate_to_subscriber() -> None:
    subscriber = FakeSubscriber([])
    receiver = ErgonomicsReceiver(subscriber, "right", "0x65")

    assert receiver.wait_for_sensor(
        timeout_sec=2.0,
        poll_interval_sec=0.1,
        require_stable_frames=4,
    )
    assert subscriber.wait_kwargs == {
        "sensor_type": "ergonomics",
        "side": "right",
        "glove_id": "0x65",
        "timeout_sec": 2.0,
        "poll_interval_sec": 0.1,
        "require_stable_frames": 4,
    }

    receiver.close()
    assert subscriber.closed
