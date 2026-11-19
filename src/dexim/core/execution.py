"""Immutable device execution metadata and provenance-aware joint streams."""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import asdict, dataclass

import numpy as np


def digest(value: object) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return "sha256:" + hashlib.sha256(encoded.encode()).hexdigest()


def position_vector(value, size: int) -> np.ndarray:
    q = np.asarray(value, dtype=float)
    if q.shape != (size,) or not np.isfinite(q).all():
        raise ValueError(
            f"Expected {size} finite positions in a one-dimensional vector"
        )
    return q


@dataclass(frozen=True)
class VisualAttachment:
    parent: str
    parent_frame: str
    child: str
    child_frame: str
    translation_m: tuple[float, ...]
    quaternion_wxyz: tuple[float, ...]
    purpose: str

    def __post_init__(self):
        for key in ("parent", "parent_frame", "child", "child_frame"):
            if not isinstance(getattr(self, key), str) or not getattr(self, key):
                raise ValueError(
                    "Explicit attachment device and frame names are required"
                )
        if self.parent == self.child or self.purpose != "visual":
            raise ValueError(
                "Kinematic attachments must be visual between distinct devices"
            )
        for key, count in (("translation_m", 3), ("quaternion_wxyz", 4)):
            value = position_vector(getattr(self, key), count)
            object.__setattr__(self, key, tuple(float(v) for v in value))
        if abs(np.linalg.norm(self.quaternion_wxyz) - 1) > 1e-8:
            raise ValueError("Attachment quaternion must be normalized wxyz")


@dataclass(frozen=True)
class ExecutionDescriptor:
    device_id: str
    model_id: str
    model_digest: str
    joint_names: tuple[str, ...]
    lower: tuple[float, ...]
    upper: tuple[float, ...]
    backend: str = "kinematic"
    frame_names: tuple[str, ...] = ()
    mapping_revision: str = "canonical-radians-position-v1"

    def __post_init__(self):
        for key in ("joint_names", "lower", "upper", "frame_names"):
            object.__setattr__(self, key, tuple(getattr(self, key)))
        if self.backend not in {"kinematic", "hardware", "mujoco", "isaac"}:
            raise ValueError("Unsupported execution backend")
        if not all(
            isinstance(v, str) and v and len(v) <= 256
            for v in (
                self.device_id,
                self.model_id,
                self.model_digest,
                *self.joint_names,
            )
        ):
            raise ValueError("Device, model and joint identities are required")
        if self.mapping_revision != "canonical-radians-position-v1":
            raise ValueError("Unsupported canonical joint mapping")
        n = len(self.joint_names)
        if not n or len(set(self.joint_names)) != n:
            raise ValueError("Unique ordered joint names are required")
        position_vector(self.lower, n)
        position_vector(self.upper, n)
        if any(lo >= hi for lo, hi in zip(self.lower, self.upper)):
            raise ValueError("Joint limits must have a nonempty interval")

    @property
    def identity(self) -> str:
        return digest(asdict(self))

    @property
    def origin(self) -> str:
        return {
            "kinematic": "synthetic",
            "hardware": "measured",
            "mujoco": "simulated",
            "isaac": "simulated",
        }[self.backend]

    def apply_limits(self, q) -> np.ndarray:
        return np.clip(
            position_vector(q, len(self.joint_names)), self.lower, self.upper
        )


@dataclass(frozen=True)
class ExecutionSample:
    """Version 6 uses host monotonic time; physics clocks need a later schema."""

    session_id: str
    participant_id: str
    instance_id: str
    device_id: str
    descriptor_id: str
    backend: str
    origin: str
    epoch_id: str | None
    reference_id: str | None
    kind: str
    sequence: int
    source_at: float
    received_at: float
    q: tuple[float, ...]
    input_sequence: int | None = None
    input_at: float | None = None
    requested_q: tuple[float, ...] = ()
    version: int = 6
    clock_id: str = "host-monotonic"
    qd: tuple[float, ...] | None = None
    tau: tuple[float, ...] | None = None

    def __post_init__(self):
        for key in ("q", "requested_q"):
            object.__setattr__(self, key, tuple(getattr(self, key)))
        for key in (
            "session_id",
            "participant_id",
            "instance_id",
            "device_id",
            "descriptor_id",
        ):
            value = getattr(self, key)
            if not isinstance(value, str) or not value or len(value) > 256:
                raise ValueError(f"Invalid {key}")
        if self.version != 6 or type(self.version) is not int:
            raise ValueError("Unsupported execution stream version")
        if self.backend != "kinematic" or self.origin != "synthetic":
            raise ValueError(
                "Version 6 currently admits synthetic kinematic state only"
            )
        if self.clock_id != "host-monotonic":
            raise ValueError("Version 6 requires host monotonic time")
        if self.kind not in {"action", "observed"}:
            raise ValueError("Invalid execution stream kind")
        if (self.epoch_id is None) != (self.reference_id is None):
            raise ValueError("Epoch and reference must travel together")
        for value in (self.epoch_id, self.reference_id):
            if value is not None and (not isinstance(value, str) or not value):
                raise ValueError("Invalid epoch/reference identity")
        if self.kind == "action" and self.epoch_id is None:
            raise ValueError("Actions require an epoch")
        if type(self.sequence) is not int or self.sequence < 1:
            raise ValueError("Invalid stream sequence")
        if (
            any(
                type(v) not in (int, float) or not math.isfinite(v)
                for v in (self.source_at, self.received_at, *self.q, *self.requested_q)
            )
            or not 0 <= self.source_at <= self.received_at
        ):
            raise ValueError("Invalid finite stream data or timing")
        if not self.q:
            raise ValueError("An ordered joint vector is required")
        if self.qd is not None or self.tau is not None:
            raise ValueError("Kinematic velocity and effort are unavailable")
        if self.kind == "action":
            if type(self.input_sequence) is not int or self.input_sequence < 1:
                raise ValueError("Actions require an input sequence")
            if len(self.requested_q) != len(self.q):
                raise ValueError("Actions require the original requested target")
            if (
                type(self.input_at) not in (int, float)
                or not math.isfinite(self.input_at)
                or not 0 <= self.input_at <= self.source_at
            ):
                raise ValueError("Actions require a host-monotonic input timestamp")
        elif (
            self.input_sequence is not None
            or self.input_at is not None
            or self.requested_q
        ):
            raise ValueError("Observed state cannot contain a requested target")

    def validate_descriptor(self, descriptor: ExecutionDescriptor) -> None:
        if (
            self.device_id != descriptor.device_id
            or self.descriptor_id != descriptor.identity
            or self.backend != descriptor.backend
            or self.origin != descriptor.origin
            or len(self.q) != len(descriptor.joint_names)
        ):
            raise ValueError("Execution descriptor mismatch")
        if any(
            q < lo - 1e-10 or q > hi + 1e-10
            for q, lo, hi in zip(self.q, descriptor.lower, descriptor.upper)
        ):
            raise ValueError("State outside execution limits")

    def to_dict(self) -> dict:
        return asdict(self)
