"""Structured status-info dataclass for node status reporting.

Defines ``StatusInfo`` -- a flat, typed dataclass that every
``ManagedNode`` subtype populates via ``get_status_info()`` and
sends over the ZMQ status plane as part of the ``info`` dict.

The same type is used by ``LaunchManager`` and CLI consumers
(``dexim system status``, TUI Status Pane) for type-safe access
to per-node runtime details.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class StatusInfo:
    """Flat snapshot of a node's runtime state sent via the status plane.

    Consumers should handle missing / zero / empty fields gracefully.
    Not every node type fills every field (e.g. a recorder node has no
    ``robot_variant``, a camera node has no ``dof_count``).

    Process-level fields (``pid``, ``memory_mb``, ``cpu_percent``) are
    populated by the ``LaunchManager`` on the consumer side -- nodes
    cannot report their own OS process stats.
    """

    # ------------------------------------------------------------------
    # Base fields (ManagedNode)
    # ------------------------------------------------------------------

    is_publishing: bool = False
    """Whether the node is publishing on the data plane."""

    teleop_active: bool = False
    """Whether teleoperation control is active."""

    countdown_remaining: float | None = None
    """Seconds remaining in the start countdown (``None`` if not active)."""

    countdown_active: bool = False
    """Whether a start countdown is in progress."""

    # ------------------------------------------------------------------
    # Process fields (populated by LaunchManager, NOT by the node)
    # ------------------------------------------------------------------

    pid: int = 0
    """OS process ID (set by ``LaunchManager`` after spawn)."""

    memory_mb: float = 0.0
    """Resident memory in MiB (via ``psutil``, if available)."""

    cpu_percent: float = 0.0
    """CPU usage percent (via ``psutil``, if available)."""

    # ------------------------------------------------------------------
    # Hardware connection
    # ------------------------------------------------------------------

    interface_mode: str = ""
    """Interface mode string: ``"mock"``, ``"sim"``, ``"real"``,
    ``"serial"``, ``"dds"``, ``"usb"``, ``"realsense"``, etc."""

    hardware_connected: bool | None = None
    """``True`` if hardware is connected, ``False`` if not,
    ``None`` if the concept is not applicable to this node type."""

    # ------------------------------------------------------------------
    # Publishing / data-plane statistics
    # ------------------------------------------------------------------

    data_rate_hz: float = 0.0
    """Actual publishing rate in Hz (from rate limiter or measured)."""

    topics_published: int = 0
    """Number of distinct ZMQ topics the node publishes."""

    data_age_sec: float | None = None
    """Age of the most recent input data sample, in seconds.
    ``None`` if no data has been received yet."""

    no_data_streak: int = 0
    """Number of consecutive iterations with no input data."""

    # ------------------------------------------------------------------
    # Robot-specific
    # ------------------------------------------------------------------

    robot_variant: str = ""
    """Human-readable variant name (e.g. ``"Nova 5"``, ``"Inspire"``)."""

    handedness: str = ""
    """``"left"``, ``"right"``, or empty if not applicable."""

    dof_count: int = 0
    """Number of degrees of freedom (actuated joints)."""

    pipeline_stages: dict[str, float] = field(default_factory=dict)
    """Per-stage maximum duration in milliseconds.
    Keys are stage names (``"recv"``, ``"pose_map"``, ``"ik"``, etc.).
    Values are the maximum time observed for that stage."""

    rate_limiter_stats: dict[str, float] = field(default_factory=dict)
    """Rate-limiter statistics: ``actual_rate``, ``target_rate``,
    ``jitter``, ``overtime_percent``, etc."""

    # ------------------------------------------------------------------
    # Recorder-specific
    # ------------------------------------------------------------------

    episode_counter: int = 0
    """Total number of episodes recorded since node start."""

    active_task: str = ""
    """Task ID of the currently active task."""

    buffer_topics: int = 0
    """Number of distinct topics in the current recording buffer."""

    buffer_frames: int = 0
    """Total number of buffered data frames across all topics."""

    writer_queue_depth: int = 0
    """Number of pending write jobs in the writer queue."""

    # ------------------------------------------------------------------
    # Manus-specific
    # ------------------------------------------------------------------

    trackers_detected: int = 0
    """Number of Vive trackers detected by Manus Core."""

    gloves_detected: int = 0
    """Number of gloves detected by Manus Core."""

    calibration_loaded: bool = False
    """Whether a world calibration transform is loaded."""

    publish_success_count: int = 0
    """Total number of frames successfully published."""

    publish_skip_count: int = 0
    """Total number of frames skipped (no change detected)."""

    error_count: int = 0
    """Total number of errors encountered since node start."""

    # ------------------------------------------------------------------
    # Serialization helpers
    # ------------------------------------------------------------------

    def to_flat_dict(self, *, skip_empty: bool = True) -> dict[str, Any]:
        """Serialize to a flat JSON-serialisable dict.

        Args:
            skip_empty: If ``True`` (default), omit fields with their
                default / zero / None / empty value.  This keeps the
                status payload compact.

        Returns:
            A ``dict`` suitable for ``json.dumps``.
        """
        result: dict[str, Any] = {}
        for field_name in self._field_names():
            value = getattr(self, field_name)
            if skip_empty and _is_empty(value):
                continue
            result[field_name] = value
        return result

    @classmethod
    def from_flat_dict(cls, data: dict[str, Any]) -> StatusInfo:
        """Deserialize from a flat dict (e.g. parsed from ZMQ JSON).

        Unknown keys are silently ignored.  Missing keys use the
        dataclass default.

        Args:
            data: Flat dict as produced by ``to_flat_dict()`` or
                received over the ZMQ status plane.

        Returns:
            A new ``StatusInfo`` instance.
        """
        valid_fields = set(cls._field_names())
        filtered = {k: v for k, v in data.items() if k in valid_fields}
        return cls(**filtered)

    @staticmethod
    def _field_names() -> tuple[str, ...]:
        """Return the ordered field names for this dataclass."""
        return (
            "is_publishing",
            "teleop_active",
            "countdown_remaining",
            "countdown_active",
            "pid",
            "memory_mb",
            "cpu_percent",
            "interface_mode",
            "hardware_connected",
            "data_rate_hz",
            "topics_published",
            "data_age_sec",
            "no_data_streak",
            "robot_variant",
            "handedness",
            "dof_count",
            "pipeline_stages",
            "rate_limiter_stats",
            "episode_counter",
            "active_task",
            "buffer_topics",
            "buffer_frames",
            "writer_queue_depth",
            "trackers_detected",
            "gloves_detected",
            "calibration_loaded",
            "publish_success_count",
            "publish_skip_count",
            "error_count",
        )


def _is_empty(value: Any) -> bool:
    """Return ``True`` if *value* is a default / empty sentinel.

    We treat ``None``, ``0`` (int), ``0.0`` (float), ``False`` (bool),
    empty ``str``, and empty ``dict`` / ``list`` as empty.
    """
    if value is None:
        return True
    if isinstance(value, bool):
        return not value  # False is the default
    if isinstance(value, (int, float)):
        return value == 0
    if isinstance(value, (str, dict, list, tuple)):
        return len(value) == 0  # type: ignore[arg-type]
    return False


__all__ = ["StatusInfo"]
