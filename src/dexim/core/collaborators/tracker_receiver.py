"""TrackerReceiver -- RigidPose subscription and reference capture."""

from __future__ import annotations

import time
from typing import Any

from loguru import logger

from dexim.core.messages import RigidPose

# Reject data older than this to avoid stale buffered poses.
# At 60 Hz a frame is produced every ~17 ms, so 100 ms ~= 6 dropped frames.
_STALE_TRACKER_THRESHOLD_SEC: float = 0.1


class TrackerReceiver:
    """Receives a single-tracker RigidPose from a per-tracker topic subscriber.

    The subscriber must be constructed with the tracker-type-specific topic
    (e.g. ``observation/manus/rigid_pose/left_hand``) so that each
    ``TrackerReceiver`` deals with exactly one physical sensor -- no runtime
    filtering is performed here.

    Reference pose capture (``capture_reference()``) is called once on
    START to record the home position of both the tracker and the
    end-effector.

    Args:
        subscriber: Object implementing ``read()`` that returns a
            ``RigidPose | None``.  Typically a
            ``TopicSubscriber[RigidPose]``.
        tracker_type: Tracker type label used only for log messages
            (e.g. ``"left_hand"``).
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
        """Tracker type label used for logging."""
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
        """Return the latest non-stale RigidPose from the subscriber.

        Reads the most recent frame from the topic-specific subscriber,
        draining any queued messages. Stale data (older than
        ``stale_threshold_sec``) is silently skipped.

        Returns:
            RigidPose, or None if unavailable or stale.
        """
        try:
            pose: RigidPose | None = self._subscriber.read_latest()
        except Exception as exc:
            logger.error(f"[{self._tracker_type}] Error reading tracker data: {exc}")
            self._no_data_streak += 1
            return None

        if pose is None:
            self._no_data_streak += 1
            if self._no_data_streak in (5, 10, 30, 60):
                logger.warning(
                    f"[{self._tracker_type}] No tracker data for "
                    f"{self._no_data_streak} consecutive frames"
                )
            return None

        now = time.time()
        if hasattr(pose, "timestamp") and pose.timestamp > 0:
            age = now - pose.timestamp
            if age > self._stale_threshold_sec:
                logger.debug(
                    f"[{self._tracker_type}] Skipping stale data "
                    f"(age={age * 1000:.1f}ms)"
                )
                self._no_data_streak += 1
                return None
            self._last_data_age_sec = age
            logger.debug(f"[{self._tracker_type}] data_age={age * 1000:.1f}ms")
        else:
            self._last_data_age_sec = None

        self._no_data_streak = 0
        return pose

    # ------------------------------------------------------------------
    # Reference capture (called on START)
    # ------------------------------------------------------------------

    def capture_reference(
        self,
        attempts: int = 20,
        required_consecutive: int = 1,
    ) -> bool:
        """Poll until a fresh pose is received and store it as reference.

        Called once on node START so that relative motion can be computed
        from the captured home position.  Attempts up to *attempts* reads,
        discarding stale frames and sleeping briefly between tries.

        Args:
            attempts: Maximum number of read attempts.
            required_consecutive: Distinct fresh samples required. Missing or
                stale samples reset the consecutive-sample count.

        Returns:
            True if a reference pose was captured, False on failure.
        """
        if required_consecutive < 1:
            raise ValueError("required_consecutive must be at least 1")
        if attempts < required_consecutive:
            raise ValueError("attempts must cover required_consecutive samples")

        consecutive = 0
        previous_timestamp: float | None = None
        for attempt in range(attempts):
            try:
                pose: RigidPose | None = self._subscriber.read_latest()
            except Exception as exc:
                logger.debug(
                    f"[{self._tracker_type}] Reference capture attempt "
                    f"{attempt + 1}/{attempts} failed: {exc}"
                )
                consecutive = 0
                previous_timestamp = None
                time.sleep(0.05)
                continue

            if pose is None:
                logger.debug(
                    f"[{self._tracker_type}] Reference capture attempt "
                    f"{attempt + 1}/{attempts}: no fresh data, retrying\u2026"
                )
                consecutive = 0
                previous_timestamp = None
                time.sleep(0.05)
                continue

            if getattr(pose, "tracking_valid", True) is False:
                consecutive = 0
                previous_timestamp = None
                time.sleep(0.05)
                continue

            now = time.time()
            if hasattr(pose, "timestamp") and pose.timestamp > 0:
                age = now - pose.timestamp
                if age > self._stale_threshold_sec:
                    logger.debug(
                        f"[{self._tracker_type}] Attempt {attempt + 1}: "
                        f"skipping stale data (age={age * 1000:.1f}ms)"
                    )
                    consecutive = 0
                    previous_timestamp = None
                    continue

                if (
                    previous_timestamp is not None
                    and pose.timestamp <= previous_timestamp
                ):
                    logger.debug(
                        f"[{self._tracker_type}] Attempt {attempt + 1}: "
                        "waiting for a newer tracker sample"
                    )
                    time.sleep(0.05)
                    continue
                previous_timestamp = pose.timestamp

            consecutive += 1
            if consecutive < required_consecutive:
                time.sleep(0.05)
                continue

            self._reference_pose = pose
            logger.info(
                f"[{self._tracker_type}] Reference pose captured "
                f"after {consecutive} consecutive fresh samples "
                f"(attempt {attempt + 1})"
            )
            return True

        logger.warning(
            f"[{self._tracker_type}] Failed to capture reference pose after "
            f"{attempts} attempts"
        )
        return False

    def clear_reference(self) -> None:
        """Invalidate the reference captured for the preceding control epoch."""
        self._reference_pose = None

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
