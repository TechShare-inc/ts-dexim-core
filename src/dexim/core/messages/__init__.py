"""Messaging contracts and topic helpers for dexim_core nodes."""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from typing import Any, cast

import msgpack
from dexim.core.messages.types import (
    FrameObservation,
    HandState,
    RigidPose,
    SkeletonJoint,
    ErgonomicsJoint,
)

# Control/status endpoints
CTRL_PUB_ENDPOINT = "tcp://localhost:5550"
STATUS_PULL_ENDPOINT = "tcp://localhost:5551"

# Control topic and commands
TOPIC_CTRL = b"control"
CTRL_START = "START"
CTRL_PAUSE = "PAUSE"
CTRL_STOP = "STOP"
CTRL_SHUTDOWN = "SHUTDOWN"
CTRL_START_REC = "START_REC"
CTRL_STOP_REC = "STOP_REC"
CTRL_DISCARD_REC = "DISCARD_REC"
CTRL_SET_TASK = "SET_TASK"
CTRL_START_PUB = "START_PUB"
CTRL_PAUSE_PUB = "PAUSE_PUB"
CTRL_STOP_PUB = "STOP_PUB"

# Node status values
STATUS_INITIALIZED = "INITIALIZED"
STATUS_STARTED = "STARTED"
STATUS_PAUSED = "PAUSED"
STATUS_HEALTHY = "HEALTHY"
STATUS_ERROR = "ERROR"
STATUS_SHUTTING_DOWN = "SHUTTING_DOWN"


class TopicBuilder:
    """Build canonical topic names for action/observation streams."""

    @dataclass(frozen=True)
    class _ActionTopics:
        def joint_cmd(self, device_id: str) -> bytes:
            return f"action/{device_id}/joint_cmd".encode()

    @dataclass(frozen=True)
    class _ObservationTopics:
        def joint_state(self, device_id: str) -> bytes:
            return f"observation/{device_id}/joint_state".encode()
        
        def ergonomics_state(self, device_id: str) -> bytes:
            return f"observation/{device_id}/ergonomics_state".encode()

        def hand_state(self, device_id: str) -> bytes:
            return f"observation/{device_id}/hand_state".encode()

        def rigid_pose(self, device_id: str, tracker_type: str | None = None) -> bytes:
            if tracker_type is not None:
                return f"observation/{device_id}/rigid_pose/{tracker_type}".encode()
            return f"observation/{device_id}/rigid_pose".encode()

        def video_frame(self, device_id: str) -> bytes:
            return f"observation/{device_id}/video_frame".encode()

    def __init__(self) -> None:
        self.action = self._ActionTopics()
        self.observation = self._ObservationTopics()


# ---------------------------------------------------------------------------
# Backward-compatibility topic constants
#
# These constants resolve the undefined-import errors in packages that still
# import them directly.  New code should use TopicBuilder instead.
# ---------------------------------------------------------------------------

#: Deprecated – use ``TopicBuilder().observation.hand_state(node_id)``
TOPIC_MANUS_RAW_SKELETONS = b"observation/manus/manus_raw_skeletons"

#: Deprecated – use ``TopicBuilder().observation.rigid_pose(node_id)``
TOPIC_MANUS_TRACKERS = b"observation/manus/manus_trackers"

#
TOPIC_MANUS_ERGONOMICS = b"observation/manus/manus_ergonomics"


class TopicValidator:
    """Validate identifiers used for topic naming."""

    _DEVICE_ID_PATTERN = re.compile(r"^[a-zA-Z0-9_-]+$")

    def validate_device_id(self, device_id: str) -> tuple[bool, str]:
        if not device_id:
            return False, "device_id must not be empty"
        if not self._DEVICE_ID_PATTERN.match(device_id):
            return False, "device_id must only contain letters, numbers, '-' or '_'"
        return True, ""

    def validate_camera_config(
        self,
        mode: str,
        device_index: int,
        serial_number: str | None,
    ) -> tuple[bool, str]:
        if mode not in {"usb", "realsense"}:
            return False, "camera mode must be 'usb' or 'realsense'"
        if mode == "usb" and device_index < 0:
            return False, "usb camera device_index must be >= 0"
        if mode == "realsense" and not serial_number:
            return False, "realsense mode requires serial_number"
        return True, ""


def pack_data_message(
    topic: bytes | str, timestamp: float, data: Any
) -> tuple[bytes, bytes]:
    """Pack a data-plane multipart message into [topic, payload] frames."""
    topic_frame = topic.encode("utf-8") if isinstance(topic, str) else bytes(topic)
    payload_frame = cast(
        bytes,
        msgpack.dumps(
            {"timestamp": float(timestamp), "data": data},
            use_bin_type=True,
        ),
    )
    return topic_frame, payload_frame


def unpack_data_message(frames: list[bytes] | tuple[bytes, ...]) -> dict[str, Any]:
    """Unpack a data-plane multipart message from [topic, payload] frames."""
    if len(frames) != 2:
        raise ValueError("data message must contain exactly 2 frames")

    topic_frame, payload_frame = frames
    payload = msgpack.unpackb(payload_frame, raw=False)
    return {
        "topic": topic_frame.decode("utf-8", errors="ignore"),
        "timestamp": payload.get("timestamp"),
        "data": payload.get("data"),
    }


def pack_status_message(
    *,
    node_id: str,
    status: str,
    is_recording: bool,
    timestamp: float,
    info: dict[str, Any] | None = None,
) -> bytes:
    """Pack node status payload for PUSH/PULL status plane."""
    payload = {
        "node_id": node_id,
        "status": status,
        "is_recording": bool(is_recording),
        "timestamp": float(timestamp),
        "info": info or {},
    }
    return json.dumps(payload, separators=(",", ":")).encode("utf-8")


__all__ = [
    # Endpoints
    "CTRL_PUB_ENDPOINT",
    "STATUS_PULL_ENDPOINT",
    # Control topics & commands
    "TOPIC_CTRL",
    "CTRL_START",
    "CTRL_PAUSE",
    "CTRL_STOP",
    "CTRL_SHUTDOWN",
    "CTRL_START_REC",
    "CTRL_STOP_REC",
    "CTRL_DISCARD_REC",
    "CTRL_SET_TASK",
    "CTRL_START_PUB",
    "CTRL_PAUSE_PUB",
    "CTRL_STOP_PUB",
    # Node status values
    "STATUS_INITIALIZED",
    "STATUS_STARTED",
    "STATUS_PAUSED",
    "STATUS_HEALTHY",
    "STATUS_ERROR",
    "STATUS_SHUTTING_DOWN",
    # Topic helpers
    "TopicBuilder",
    "TopicValidator",
    # Backward-compat topic constants (deprecated)
    "TOPIC_MANUS_RAW_SKELETONS",
    "TOPIC_MANUS_TRACKERS",
    "TOPIC_MANUS_ERGONOMICS",
    # Serialization helpers
    "pack_data_message",
    "unpack_data_message",
    "pack_status_message",
    # Message types
    "FrameObservation",
    "HandState",
    "RigidPose",
    "SkeletonJoint",
    "ErgonomicsJoint",
]
