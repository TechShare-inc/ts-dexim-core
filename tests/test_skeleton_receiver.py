from __future__ import annotations

from dexim.core.collaborators import SkeletonReceiver
from dexim.core.messages import HandState


class FakeSubscriber:
    def __init__(self, batch: list[HandState]) -> None:
        self.batch = batch

    def read_latest_batch(self) -> list[HandState]:
        return self.batch


def test_bare_skeleton_glove_id_remains_hexadecimal() -> None:
    decimal = HandState(glove_id=101, side="left", timestamp=0.0)
    hexadecimal = HandState(glove_id=0x101, side="left", timestamp=0.0)
    receiver = SkeletonReceiver(FakeSubscriber([decimal, hexadecimal]), "left", "101")

    assert receiver.receive() is hexadecimal
