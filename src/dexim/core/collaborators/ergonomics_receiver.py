"""ErgonomicsReceiver — ErgonomicsJoint subscription and glove-ID filtering."""

from __future__ import annotations

import time
from typing import Any

from dexim.core.collaborators.protocols import WaitableSubscriberProtocol
from dexim.core.messages import HandState, ErgonomicsJoint
from loguru import logger


class ErgonomicsReceiver:
    """Receives and filters Ergonomics messages from a ergonomics subscriber.

    Args:
        subscriber: Object implementing ``read_latest_batch()``
            (TopicSubscriber[Ergonomics] or a compatible mock).
        handedness: ``"left"`` or ``"right"``.
        glove_id: Expected glove ID for filtering (hex str, int, or None).
    """

    def __init__(
        self,
        subscriber: Any,
        handedness: str,
        glove_id: str | int | None = None,
    ) -> None:
        self._subscriber = subscriber
        self._handedness = handedness
        self._expected_glove_id = glove_id
        self._no_data_streak: int = 0
        self._last_data_age_sec: float | None = None
        self._warning_logged: bool = False

    @property
    def last_data_age_sec(self) -> float | None:
        """Age of the last received data frame in seconds."""
        return self._last_data_age_sec

    @property
    def no_data_streak(self) -> int:
        """Consecutive frames with no matching data."""
        return self._no_data_streak

    def receive(self) -> ErgonomicsJoint | None:
        """Read the latest ErgonomicsJoint matching this hand's glove_id.

        Drains queued messages and uses only the newest one so the control
        loop always acts on fresh data.

        Returns:
            Matching ErgonomicsJoint, or None if no data or no match.
        """
        result: ErgonomicsJoint | None = None
        try:
            batch: list[ErgonomicsJoint] = self._subscriber.read_latest_batch()
        except Exception as exc:
            logger.error(f"Error reading ergonomics data: {exc}")
        else:
            if batch:
                if self._expected_glove_id is None:
                    if not self._warning_logged:
                        logger.warning(
                            f"[{self._handedness}] No glove_id configured; "
                            "using first ErgonomicsJoint in batch."
                        )
                        self._warning_logged = True
                    result = batch[0]
                else:
                    exp_id = self._resolve_glove_id()
                    if exp_id is not None:
                        for state in batch:
                            if state.glove_id == exp_id:
                                self._warning_logged = False
                                result = state
                                break
                        if result is None and not self._warning_logged:
                            logger.warning(
                                f"[{self._handedness}] No ErgonomicsJoint matched glove_id "
                                f"0x{exp_id:X} ({exp_id}) in batch of {len(batch)}"
                            )
                            self._warning_logged = True

        if result is not None:
            data_age = time.time() - result.timestamp
            self._last_data_age_sec = data_age
            self._no_data_streak = 0
            logger.debug(
                f"[{self._handedness}] ergonomics data_age={data_age * 1000:.1f}ms"
            )
        else:
            self._no_data_streak += 1
            if self._no_data_streak in (5, 10, 30, 60):
                logger.warning(
                    f"[{self._handedness}] No ergonomics data for "
                    f"{self._no_data_streak} consecutive frames"
                )

        return result

    def drain(self) -> None:
        """Drain all buffered messages without processing (used during pause)."""
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
        """Wait for ergonomics data to appear in the subscriber stream.

        Args:
            timeout_sec: Maximum wait time.
            poll_interval_sec: Polling interval.
            require_stable_frames: Consecutive matching frames required.

        Returns:
            True if ergonomics found, False on timeout.
        """
        if not isinstance(self._subscriber, WaitableSubscriberProtocol):
            logger.debug("Subscriber doesn't support wait_for_sensor, assuming ready")
            return True

        return self._subscriber.wait_for_sensor(
            sensor_type="ergonomics",
            handedness=self._handedness,
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
        try:
            if isinstance(self._expected_glove_id, str):
                try:
                    return int(self._expected_glove_id, 16)
                except ValueError:
                    return int(self._expected_glove_id, 10)
            return int(self._expected_glove_id)  # type: ignore[arg-type]
        except (ValueError, TypeError) as exc:
            logger.error(
                f"[{self._handedness}] Invalid glove_id "
                f"'{self._expected_glove_id}': {exc}"
            )
            return None
