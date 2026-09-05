"""Shared implementation for glove-state batch receivers."""

from __future__ import annotations

import time
from typing import Any, Generic, Protocol, TypeVar

from loguru import logger

from dexim.core.collaborators.protocols import WaitableSubscriberProtocol


class _GloveState(Protocol):
    glove_id: int
    timestamp: float


_StateT = TypeVar("_StateT", bound=_GloveState)


class _GloveStateReceiver(Generic[_StateT]):
    """Filter batches of timestamped glove states by configured glove ID."""

    def __init__(
        self,
        subscriber: Any,
        side: str,
        glove_id: str | int | None,
        *,
        sensor_type: str,
        state_label: str,
        bare_string_base: int,
    ) -> None:
        self._subscriber = subscriber
        self._side = side
        self._expected_glove_id = glove_id
        self._sensor_type = sensor_type
        self._state_label = state_label
        self._bare_string_base = bare_string_base
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

    def receive(self) -> _StateT | None:
        """Return the newest state matching the configured glove."""
        result: _StateT | None = None
        try:
            batch: list[_StateT] = self._subscriber.read_latest_batch()
        except Exception as exc:
            logger.error(f"Error reading {self._sensor_type} data: {exc}")
        else:
            if batch:
                expected_id = self._resolve_glove_id()
                if expected_id is None and self._expected_glove_id is None:
                    if not self._warning_logged:
                        logger.warning(
                            f"[{self._side}] No glove_id configured; "
                            f"using first {self._state_label} in batch."
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
                            f"[{self._side}] No {self._state_label} matched glove_id "
                            f"0x{expected_id:X} ({expected_id}) in batch of "
                            f"{len(batch)}"
                        )
                        self._warning_logged = True

        if result is not None:
            data_age = time.time() - result.timestamp
            self._last_data_age_sec = data_age
            self._no_data_streak = 0
            logger.debug(
                f"[{self._side}] {self._sensor_type} data_age={data_age * 1000:.1f}ms"
            )
        else:
            self._no_data_streak += 1
            if self._no_data_streak in (5, 10, 30, 60):
                logger.warning(
                    f"[{self._side}] No {self._sensor_type} data for "
                    f"{self._no_data_streak} consecutive frames"
                )

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
        """Wait for matching data when the subscriber supports it."""
        if not isinstance(self._subscriber, WaitableSubscriberProtocol):
            logger.debug("Subscriber doesn't support wait_for_sensor, assuming ready")
            return True

        return self._subscriber.wait_for_sensor(
            sensor_type=self._sensor_type,
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
                base = 16 if value.lower().startswith("0x") else self._bare_string_base
                return int(value, base)
            return int(self._expected_glove_id)
        except (ValueError, TypeError) as exc:
            logger.error(
                f"[{self._side}] Invalid glove_id {self._expected_glove_id!r}: {exc}"
            )
            return None
