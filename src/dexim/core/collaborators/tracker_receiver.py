"""TrackerReceiver — RigidPose subscription, type filtering, and reference capture."""

from __future__ import annotations

import time
from typing import Any

from dexim.core.messages import RigidPose
from loguru import logger

# Reject data older than this to avoid stale buffered poses.
# At 60 Hz a frame is produced every ~17 ms, so 100 ms ≈ 6 dropped frames.
_STALE_TRACKER_THRESHOLD_SEC: float = 0.1


class TrackerReceiver:
    """Receives and filters RigidPose messages by tracker type.

    Designed for arm control nodes that need 6-DOF rigid-body tracker data.
    Drains the subscriber queue on each call to ``receive()`` so the control
    loop always acts on the most recent pose.

    Reference pose capture (``capture_reference()``) is called once on
    START to record the home position of both the tracker and the
    end-effector.

    Args:
        subscriber: Object implementing ``read()`` that returns a dict with
            a ``"trackers"`` key containing a list of objects that have
            ``tracker_type`` and ``timestamp`` attributes.  Typically a
            ``DataSubscriber`` (Manus or compatible mock).
        tracker_type: Tracker type string to match (e.g. ``"left_hand"``).
        stale_threshold_sec: Data older than this is rejected.
    """

    def __init__(
        self,
        subscriber: Any,
        tracker_type: str,
        stale_threshold_sec: float = _STALE_TRACKER_THRESHOLD_SEC,
    ) -> None:
        self._subscriber = subscriber
        self._tracker_type = tracker_type
        self._stale_threshold_sec = stale_threshold_sec

        self._reference_pose: RigidPose | None = None
        self._last_data_age_sec: float | None = None
        self._no_data_streak: int = 0

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def tracker_type(self) -> str:
        """Tracker type string used for filtering."""
        return self._tracker_type

    @property
    def reference_pose(self) -> RigidPose | None:
        """Reference pose captured on START, or None if not yet captured."""
        return self._reference_pose

    @property
    def last_data_age_sec(self) -> float | None:
        """Age of the last successfully received pose in seconds."""
        return self._last_data_age_sec

    @property
    def no_data_streak(self) -> int:
        """Consecutive receive() calls that returned None."""
        return self._no_data_streak

    # ------------------------------------------------------------------
    # Core receive
    # ------------------------------------------------------------------

    def receive(self) -> RigidPose | None:
        """Return the latest non-stale RigidPose matching ``tracker_type``.

        Reads one data frame from the subscriber and searches for the
        configured tracker type.  Stale data (older than
        ``stale_threshold_sec``) is silently skipped.

        Returns:
            Matching RigidPose, or None if unavailable or stale.
        """
        result: RigidPose | None = None
        try:
            data = self._subscriber.read()
            trackers = data.get("trackers", [])
        except Exception as exc:
            logger.error(f"[{self._tracker_type}] Error reading tracker data: {exc}")
            self._no_data_streak += 1
            return None

        now = time.time()
        for tracker in trackers:
            if tracker.tracker_type != self._tracker_type:
                continue

            # Reject stale data
            if hasattr(tracker, "timestamp") and tracker.timestamp > 0:
                age = now - tracker.timestamp
                if age > self._stale_threshold_sec:
                    logger.debug(
                        f"[{self._tracker_type}] Skipping stale data "
                        f"(age={age * 1000:.1f}ms)"
                    )
                    continue

            result = tracker
            break

        if result is not None:
            self._last_data_age_sec = (
                now - result.timestamp if result.timestamp > 0 else None
            )
            self._no_data_streak = 0
            if self._last_data_age_sec is not None:
                logger.debug(
                    f"[{self._tracker_type}] "
                    f"data_age={self._last_data_age_sec * 1000:.1f}ms"
                )
        else:
            self._no_data_streak += 1
            if self._no_data_streak in (5, 10, 30, 60):
                logger.warning(
                    f"[{self._tracker_type}] No tracker data for "
                    f"{self._no_data_streak} consecutive frames"
                )

        return result

    # ------------------------------------------------------------------
    # Reference capture (called on START)
    # ------------------------------------------------------------------

    def capture_reference(self, attempts: int = 20) -> bool:
        """Poll until a fresh pose is received and store it as reference.

        Called once on node START so that relative motion can be computed
        from the captured home position.  Attempts up to *attempts* reads,
        discarding stale frames and sleeping briefly between tries.

        Args:
            attempts: Maximum number of read attempts.

        Returns:
            True if a reference pose was captured, False on failure.
        """
        for attempt in range(attempts):
            try:
                data = self._subscriber.read()
                trackers = data.get("trackers", [])
            except Exception as exc:
                logger.debug(
                    f"[{self._tracker_type}] Reference capture attempt "
                    f"{attempt + 1}/{attempts} failed: {exc}"
                )
                time.sleep(0.05)
                continue

            now = time.time()
            found: RigidPose | None = None
            for tracker in trackers:
                if tracker.tracker_type != self._tracker_type:
                    continue

                if hasattr(tracker, "timestamp") and tracker.timestamp > 0:
                    age = now - tracker.timestamp
                    if age > self._stale_threshold_sec:
                        logger.debug(
                            f"[{self._tracker_type}] Attempt {attempt + 1}: "
                            f"skipping stale data (age={age * 1000:.1f}ms)"
                        )
                        # Don't sleep — drain queue quickly
                        continue

                found = tracker
                break

            if found is not None:
                self._reference_pose = found
                logger.info(
                    f"[{self._tracker_type}] Reference pose captured "
                    f"(attempt {attempt + 1})"
                )
                return True

            logger.debug(
                f"[{self._tracker_type}] Reference capture attempt "
                f"{attempt + 1}/{attempts}: no fresh data, retrying…"
            )
            time.sleep(0.05)

        logger.warning(
            f"[{self._tracker_type}] Failed to capture reference pose after "
            f"{attempts} attempts"
        )
        return False

    # ------------------------------------------------------------------
    # Drain / close
    # ------------------------------------------------------------------

    def drain(self) -> None:
        """Read and discard one data frame (used while paused)."""
        try:
            self._subscriber.read()
        except Exception:
            pass

    def close(self) -> None:
        """Close the underlying subscriber."""
        try:
            self._subscriber.close()
        except Exception as exc:
            logger.error(f"[{self._tracker_type}] Error closing subscriber: {exc}")
