"""ErgonomicsState subscription and glove-ID filtering."""

from __future__ import annotations

import time
from typing import Any

from loguru import logger

from dexim.core.collaborators.protocols import WaitableSubscriberProtocol
from dexim.core.messages import ErgonomicsState


class ErgonomicsReceiver:
    """Receive ergonomics states for one configured hand.

    Args:
        subscriber: Object implementing ``read_latest_batch()``.
        side: Hand side used for diagnostics and sensor waiting.
        glove_id: Expected glove identifier as a hexadecimal string, decimal
            string, integer, or ``None`` to accept the first state.
    """

    def __init__(
        self,
        subscriber: Any,
        side: str,
        glove_id: str | int | None = None,
    ) -> None:
        self._subscriber = subscriber
        self._side = side
        self._expected_glove_id = glove_id
        self._no_data_streak = 0
        self._last_data_age_sec: float | None = None
        self._warning_logged = False

    @property
    def last_data_age_sec(self) -> float | None:
        """Age of the last matching state in seconds."""
        return self._last_data_age_sec

    @property
    def no_data_streak(self) -> int:
        """Number of consecutive reads without a matching state."""
        return self._no_data_streak

    def receive(self) -> ErgonomicsState | None:
        """Return the newest state matching the configured glove."""
        result: ErgonomicsState | None = None
        try:
            batch: list[ErgonomicsState] = self._subscriber.read_latest_batch()
        except Exception as exc:
            logger.error(f"Error reading ergonomics data: {exc}")
        else:
            if batch:
                expected_id = self._resolve_glove_id()
                if expected_id is None and self._expected_glove_id is None:
                    if not self._warning_logged:
                        logger.warning(
                            f"[{self._side}] No glove_id configured; "
                            "using first ErgonomicsState in batch."
                        )
                        self._warning_logged = True
                    result = batch[0]
                elif expected_id is not None:
                    result = next(
                        (state for state in batch if state.glove_id == expected_id),
                        None,
                    )
                    if result is not None:
                        self._warning_logged = False
                    elif not self._warning_logged:
                        logger.warning(
                            f"[{self._side}] No ErgonomicsState matched glove_id "
                            f"0x{expected_id:X} ({expected_id}) in batch of "
                            f"{len(batch)}"
                        )
                        self._warning_logged = True

        if result is not None:
            self._last_data_age_sec = time.time() - result.timestamp
            self._no_data_streak = 0
        else:
            self._no_data_streak += 1

        return result

    def drain(self) -> None:
        """Drain buffered states without processing them."""
        try:
            self._subscriber.read_latest_batch()
        except Exception:
            pass

    def wait_for_sensor(
        self,
        timeout_sec: float = 5.0,
        poll_interval_sec: float = 0.2,
        require_stable_frames: int = 2,
    ) -> bool:
        """Wait for matching ergonomics data when the subscriber supports it."""
        if not isinstance(self._subscriber, WaitableSubscriberProtocol):
            return True

        return self._subscriber.wait_for_sensor(
            sensor_type="ergonomics",
            side=self._side,
            glove_id=self._expected_glove_id,
            timeout_sec=timeout_sec,
            poll_interval_sec=poll_interval_sec,
            require_stable_frames=require_stable_frames,
        )

    def close(self) -> None:
        """Close the underlying subscriber."""
        try:
            self._subscriber.close()
        except Exception as exc:
            logger.error(f"Error closing subscriber: {exc}")

    def _resolve_glove_id(self) -> int | None:
        """Normalize the configured glove identifier."""
        if self._expected_glove_id is None:
            return None
        try:
            if isinstance(self._expected_glove_id, str):
                value = self._expected_glove_id.strip()
                base = 16 if value.lower().startswith("0x") else 10
                return int(value, base)
            return int(self._expected_glove_id)
        except (ValueError, TypeError) as exc:
            logger.error(
                f"[{self._side}] Invalid glove_id {self._expected_glove_id!r}: {exc}"
            )
            return None
