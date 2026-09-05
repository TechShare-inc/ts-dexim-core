"""Product-neutral ErgonomicsState receiver."""

from __future__ import annotations

from typing import Any

from dexim.core.collaborators._glove_state_receiver import _GloveStateReceiver
from dexim.core.messages import ErgonomicsState


class ErgonomicsReceiver(_GloveStateReceiver[ErgonomicsState]):
    """Receive ergonomics states for one configured hand.

    Args:
        subscriber: Object implementing ``read_latest_batch()``.
        side: Hand side used for diagnostics and sensor waiting.
        glove_id: Expected glove identifier as a ``0x``-prefixed hexadecimal
            string, decimal string, integer, or ``None`` to accept the first
            state.
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
            sensor_type="ergonomics",
            state_label="ErgonomicsState",
            bare_string_base=10,
        )
