"""
BaseHandModel: Abstract base class for hand robot kinematics models.

This module provides an abstract base class that defines the common interface
for all hand robot models (DH5, Inspire, Generic, etc.).
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

import numpy.typing as npt

try:
    import pinocchio as pin
except ImportError as _err:
    raise ImportError(
        "pinocchio is required by dexim.core.model but is not installed. "
        "Install it via conda: conda install pinocchio -c conda-forge"
    ) from _err

if TYPE_CHECKING:
    from dexim.core.model.optimizer import VectorOptimizer


class BaseHandModel(ABC):
    """
    Abstract base class for hand robot kinematics models.

    This class defines the common interface that all hand models must implement.
    It represents "what the robot is" - its structure, geometry, and mathematical
    properties without any I/O or hardware control.

    All hand models should provide:
    - Pinocchio Model/Data for kinematics computations
    - Frame management (end-effector frames for fingers)
    - Forward kinematics computation
    - Joint limits, names, and model properties
    - Keypoint target information for optimization

    Attributes:
        model: Pinocchio Model containing robot structure
        data: Pinocchio Data for kinematics computations
        geometry_model: Optional geometry model for visualization
        nq: Number of position variables
        nv: Number of velocity variables
        valid_frames: Dictionary of valid frames
        tip_frames: Dictionary of tip frames
        valid_frame_names: List of valid frame names
        tip_frame_names: List of tip frame names
    """

    # Core Pinocchio structures - must be initialized by subclasses
    model: pin.Model
    data: pin.Data
    geometry_model: pin.GeometryModel | None

    # Dimensions
    nq: int  # Number of position variables
    nv: int  # Number of velocity variables

    # Frame management
    valid_frames: dict[str, pin.Frame]
    tip_frames: dict[str, pin.Frame]
    valid_frame_names: list[str]
    tip_frame_names: list[str]
    remaining_frame_names: list[str]
    ee_frame_names: list[str]
    optimizer: VectorOptimizer | None = None

    @abstractmethod
    def __init__(self, **kwargs):
        """Initialize the hand model. Must be implemented by subclasses."""

        pass

    @abstractmethod
    def get_frame_id(self, frame_name: str) -> int:
        """
        Get the frame ID for a given frame name.

        Args:
            frame_name: Name of the frame

        Returns:
            Frame ID in the model

        Raises:
            ValueError: If frame name is not found
        """

        pass

    @abstractmethod
    def get_keypoint_targets(self) -> list[tuple[str, str]]:
        """
        Get the list of keypoint target pairs for optimization.

        Returns:
            List of tuples containing (source_frame, destination_frame) pairs
            representing the finger segments to track (e.g., MCP to tip)
        """

        pass

    def get_frame_pose(self, frame_name: str) -> pin.SE3:
        """
        Get the pose of a frame.

        Args:
            frame_name: Name of the frame

        Returns:
            SE3 pose of the frame
        """

        frame_id = self.get_frame_id(frame_name)
        return self.data.oMf[frame_id]

    def get_frame_position(self, frame_name: str) -> npt.NDArray:
        """
        Get the position of a frame.

        Args:
            frame_name: Name of the frame

        Returns:
            3D position vector
        """

        return self.get_frame_pose(frame_name).translation

    def get_joint_names(self) -> list[str]:
        """
        Get list of joint names.

        Returns:
            List of joint names
        """

        return [self.model.names[i] for i in range(1, self.model.njoints)]

    def get_frame_names(self) -> list[str]:
        """
        Get list of frame names.

        Returns:
            List of frame names
        """

        return self.valid_frame_names

    def get_joint_limits(self) -> tuple[npt.NDArray, npt.NDArray]:
        """
        Get joint position limits.

        Returns:
            Tuple of (lower_limits, upper_limits) in radians
        """

        return (
            self.model.lowerPositionLimit.copy(),
            self.model.upperPositionLimit.copy(),
        )

    def print_model_info(self) -> None:
        """Print detailed information about the model."""

        print(f"\n=== {self.__class__.__name__} Information ===")
        print(f"Degrees of freedom: {self.model.nq}")
        print(f"Number of joints: {self.model.njoints}")
        print(f"Number of frames: {self.model.nframes}")
        if self.geometry_model:
            print(
                f"Number of geometry objects: {len(self.geometry_model.geometryObjects)}"
            )

        print("\nJoint names:")
        for i, name in enumerate(self.get_joint_names()):
            print(f"  {i}: {name}")

        print("\nValid frame names:")
        for name in self.valid_frame_names:
            print(f"  {name}")

        print("\nTip frame names:")
        for name in self.tip_frame_names:
            print(f"  {name}")
