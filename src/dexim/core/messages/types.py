"""Shared message dataclasses for the DexImitate data plane.

These types form the lingua franca between input device packages (e.g., dexim-manus)
and robot controller packages (e.g., dexim-nova, dexim-inspire, dexim-dh5).

Design principles:
- Hardware-agnostic: no device names in type definitions
- Serialization-friendly: plain Python types (tuples, lists, ints, floats)
- Round-trip safe: to_dict() / from_dict() are inverse operations via msgpack
- Protocol-compatible: RigidPose satisfies TrackerDataProtocol,
  HandState satisfies SkeletonDataProtocol (structurally)

Message vocabulary:
    JointCommand  — controller → robot arm/hand (desired configuration)
    JointState    — robot → system (observed joint configuration)
    HandState     — input device → hand controller (observed hand skeleton)
    RigidPose     — input device → arm controller (observed 6-DOF rigid body pose)
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any


@dataclass
class SkeletonJoint:
    """A single joint in a hand skeleton.

    Attributes:
        id: Joint index as defined by the source device.
        position: (x, y, z) position in metres.
        rotation: (w, x, y, z) unit quaternion.
        scale: (x, y, z) scale factors (typically (1, 1, 1)).
    """

    id: int
    position: tuple[float, float, float]
    rotation: tuple[float, float, float, float]  # wxyz
    scale: tuple[float, float, float] = (1.0, 1.0, 1.0)

    def to_dict(self) -> dict[str, Any]:
        """Serialize to a msgpack-compatible dict.

        Returns:
            Dict with keys 'id', 'position', 'rotation', 'scale'.
        """
        return {
            "id": self.id,
            "position": list(self.position),
            "rotation": list(self.rotation),
            "scale": list(self.scale),
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> SkeletonJoint:
        """Deserialize from a msgpack-decoded dict.

        Args:
            d: Dict with keys 'id', 'position', 'rotation', and optionally 'scale'.

        Returns:
            SkeletonJoint instance.
        """
        pos = d["position"]
        rot = d["rotation"]
        sc = d.get("scale", [1.0, 1.0, 1.0])
        return cls(
            id=int(d["id"]),
            position=(float(pos[0]), float(pos[1]), float(pos[2])),
            rotation=(float(rot[0]), float(rot[1]), float(rot[2]), float(rot[3])),
            scale=(float(sc[0]), float(sc[1]), float(sc[2])),
        )


@dataclass
class HandState:
    """Observed hand skeleton from an input device.

    This is the hardware-agnostic counterpart to raw skeleton dicts published
    by device nodes (e.g., ManusNode). Robot controller packages depend on
    this type rather than on device-specific subscribers.

    Attributes:
        glove_id: Source device identifier (integer glove/hand ID).
        side: 'left' or 'right'.
        timestamp: Capture time in seconds (Unix epoch).
        joints: Ordered list of skeleton joints.
    """

    glove_id: int
    side: str  # "left" | "right"
    timestamp: float
    joints: list[SkeletonJoint] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        """Serialize to a msgpack-compatible dict.

        Returns:
            Dict suitable for use as the ``data`` argument to
            ``pack_data_message()``.
        """
        return {
            "glove_id": self.glove_id,
            "side": self.side,
            "timestamp": self.timestamp,
            "joints": [j.to_dict() for j in self.joints],
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> HandState:
        """Deserialize from a msgpack-decoded dict.

        Args:
            d: Dict produced by ``to_dict()`` (or the msgpack payload's
               ``data`` field).

        Returns:
            HandState instance.
        """
        return cls(
            glove_id=int(d["glove_id"]),
            side=str(d["side"]),
            timestamp=float(d["timestamp"]),
            joints=[SkeletonJoint.from_dict(j) for j in d.get("joints", [])],
        )

    # ------------------------------------------------------------------
    # Helpers for downstream processing
    # ------------------------------------------------------------------

    def joint_by_id(self, joint_id: int) -> SkeletonJoint | None:
        """Look up a joint by its id.

        Args:
            joint_id: The joint index to look up.

        Returns:
            Matching SkeletonJoint, or None.
        """
        for j in self.joints:
            if j.id == joint_id:
                return j
        return None

    def to_legacy_nodes_dict(self) -> dict[str, Any]:
        """Convert to the legacy raw-dict format expected by ManusFeatureExtractor.

        Returns:
            Dict with key 'nodes' mapping to a list of node dicts, compatible
            with the external ``manus_subscriber`` feature extractor.
        """
        return {"nodes": [j.to_dict() for j in self.joints]}


@dataclass
class RigidPose:
    """Observed 6-DOF rigid body pose from an input device.

    This is the hardware-agnostic counterpart to raw tracker dicts published
    by device nodes (e.g., ManusNode). Robot arm controller packages depend on
    this type rather than on device-specific subscribers.

    Attributes:
        tracker_id: Unique string identifier for the tracker.
        tracker_type: Semantic tracker role, e.g. 'left_hand', 'right_hand',
            'hmd', 'left_elbow'.  Snake_case, hardware-neutral naming.
        position: (x, y, z) position in metres (world frame).
        rotation: (w, x, y, z) unit quaternion (world frame).
        timestamp: Capture time in seconds (Unix epoch).
        is_hmd: True if this tracker is a head-mounted display.
        user_id: User/session identifier from the tracking system.
        system_type: Optional tracking system name (e.g. 'SteamVR').
    """

    tracker_id: str
    tracker_type: str
    position: tuple[float, float, float]
    rotation: tuple[float, float, float, float]  # wxyz
    timestamp: float
    is_hmd: bool = False
    user_id: int = 0
    system_type: str = ""

    # ------------------------------------------------------------------
    # Serialization
    # ------------------------------------------------------------------

    def to_dict(self) -> dict[str, Any]:
        """Serialize to a msgpack-compatible dict.

        Returns:
            Dict suitable for use as the ``data`` element inside a tracker
            list published by ``pack_data_message()``.
        """
        return {
            "tracker_id": self.tracker_id,
            "tracker_type": self.tracker_type,
            "position": list(self.position),
            "rotation": list(self.rotation),
            "timestamp": self.timestamp,
            "is_hmd": self.is_hmd,
            "user_id": self.user_id,
            "system_type": self.system_type,
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> RigidPose:
        """Deserialize from a msgpack-decoded dict.

        Args:
            d: Dict produced by ``to_dict()`` (or the msgpack payload's
               ``data`` field element).

        Returns:
            RigidPose instance.
        """
        pos = d["position"]
        rot = d["rotation"]
        return cls(
            tracker_id=str(d["tracker_id"]),
            tracker_type=str(d["tracker_type"]),
            position=(float(pos[0]), float(pos[1]), float(pos[2])),
            rotation=(float(rot[0]), float(rot[1]), float(rot[2]), float(rot[3])),
            timestamp=float(d.get("timestamp", time.time())),
            is_hmd=bool(d.get("is_hmd", False)),
            user_id=int(d.get("user_id", 0)),
            system_type=str(d.get("system_type", "")),
        )

    # ------------------------------------------------------------------
    # Protocol compatibility helpers
    # ------------------------------------------------------------------

    @property
    def side(self) -> str | None:
        """Infer side from tracker_type.

        Returns:
            'left', 'right', or None if not a hand tracker.
        """
        t = self.tracker_type.lower()
        if "left" in t:
            return "left"
        if "right" in t:
            return "right"
        return None

    @property
    def transform(self) -> Any:
        """6-DOF pose as Transform3D, satisfying TrackerDataProtocol.

        Returns:
            Transform3D built from position and rotation.

        Raises:
            ImportError: If dexim.core.spatial is not available.
        """
        return self.to_transform3d()

    def to_transform3d(self) -> Any:
        """Convert to a Transform3D spatial object.

        Returns:
            Transform3D instance built from position and rotation.

        Raises:
            ImportError: If dexim.core.spatial is not available.
        """
        import numpy as np
        from dexim.core.spatial import Transform3D

        return Transform3D(
            position=np.array(self.position),
            w_quat=np.array(self.rotation),
        )


@dataclass
class FrameObservation:
    """Observed color/depth frame payload from a sensor device.

    Attributes:
        device_id: Source camera identifier (node/device id).
        timestamp: Capture time in seconds (Unix epoch).
        color: JPEG-compressed color frame bytes.
        depth: Raw depth frame bytes (uint16 little-endian).
        width: Frame width in pixels.
        height: Frame height in pixels.
        depth_scale: Meters-per-unit scale for depth values.
        intrinsics: Camera intrinsics dictionary.
    """

    device_id: str
    timestamp: float
    color: bytes
    depth: bytes
    width: int
    height: int
    depth_scale: float
    intrinsics: dict[str, float] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        """Serialize to a msgpack-compatible dict."""
        return {
            "device_id": self.device_id,
            "timestamp": self.timestamp,
            "color": self.color,
            "depth": self.depth,
            "width": self.width,
            "height": self.height,
            "depth_scale": self.depth_scale,
            "intrinsics": self.intrinsics,
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> FrameObservation:
        """Deserialize from a msgpack-decoded dict."""
        intrinsics = d.get("intrinsics", {})
        return cls(
            device_id=str(d["device_id"]),
            timestamp=float(d["timestamp"]),
            color=bytes(d["color"]),
            depth=bytes(d["depth"]),
            width=int(d["width"]),
            height=int(d["height"]),
            depth_scale=float(d["depth_scale"]),
            intrinsics={
                str(k): float(v) for k, v in intrinsics.items() if isinstance(k, str)
            },
        )


__all__ = [
    "FrameObservation",
    "HandState",
    "RigidPose",
    "SkeletonJoint",
]
