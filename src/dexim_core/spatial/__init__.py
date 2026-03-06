"""
ts-spatial: A lightweight Python library for 3D spatial transformations.

This package provides utilities for working with 3D spatial transformations,
including position and rotation representations, transform composition, and
conversion between various formats (matrices, quaternions, etc.).

Main Components:
    - Transform3D: Class for representing and manipulating 3D transformations
    - rigid_transform: Function for computing optimal rigid transformations
    - Calibration utilities for tracker-to-world frame registration
    - Custom exceptions for transformation errors

Example:
    >>> from dexim_core.spatial import Transform3D
    >>> import numpy as np
    >>>
    >>> # Create a transform
    >>> pos = np.array([1.0, 2.0, 3.0])
    >>> quat = np.array([1.0, 0.0, 0.0, 0.0])  # w, x, y, z
    >>> transform = Transform3D(position=pos, w_quat=quat)
    >>>
    >>> # Get homogeneous matrix
    >>> matrix = transform.homogeneous
    >>>
    >>> # Compose transforms
    >>> result = transform * transform.inverse()  # Should be identity
"""

from .transform import (
    Transform3D,
    rigid_transform,
    SrcDstSizeMismatchError,
    InvalidPointDimError,
    NotEnoughPointsError,
    RankDeficiencyError,
)

from .calibration import (
    calibrate_world,
    save_world_calibration,
    load_world_calibration,
    calibrate_world_interactive,
    calibrate_world_iter,
    # Backward compatibility aliases
    calibrate_nova_tracker,
    save_calibration,
    load_calibration,
    calibrate_nova_tracker_interactive,
)

__version__ = "0.1.0"
__author__ = "Haoyan Li"

__all__ = [
    # Main classes and functions
    "Transform3D",
    "rigid_transform",
    # Calibration functions
    "calibrate_world",
    "save_world_calibration",
    "load_world_calibration",
    "calibrate_world_interactive",
    "calibrate_world_iter",
    # Backward compatibility aliases
    "calibrate_nova_tracker",
    "save_calibration",
    "load_calibration",
    "calibrate_nova_tracker_interactive",
    # Exceptions
    "SrcDstSizeMismatchError",
    "InvalidPointDimError",
    "NotEnoughPointsError",
    "RankDeficiencyError",
    # Metadata
    "__version__",
]
