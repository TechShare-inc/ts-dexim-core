"""Product-neutral HandState receiver."""

from __future__ import annotations

from typing import Any

from dexim.core.collaborators._glove_state_receiver import _GloveStateReceiver
from dexim.core.messages import HandState


class SkeletonReceiver(_GloveStateReceiver[HandState]):
    """Receive hand skeleton states for one configured hand.

    Args:
        subscriber: Object implementing ``read_latest_batch()``.
        side: ``"left"`` or ``"right"``.
        glove_id: Expected glove ID as a hexadecimal string, integer, or
            ``None`` to accept the first state. Bare strings remain
            hexadecimal for backward compatibility.
    """

    def __init__(
        self,
        subscriber: Any,
        side: str,
        glove_id: str | int | None = None,
    ) -> None:
        super().__init__(
            subscriber,
            side,
            glove_id,
            sensor_type="skeleton",
            state_label="HandState",
            bare_string_base=16,
        )
