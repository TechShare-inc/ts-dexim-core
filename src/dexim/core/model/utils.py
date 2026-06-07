"""
URDF loading utilities for robot models.

This module provides unified functions for loading URDF models with Pinocchio,
handling package:// URL replacement automatically.
"""

from __future__ import annotations

import os
import tempfile
from pathlib import Path

try:
    import pinocchio as pin
except ImportError as _err:
    raise ImportError(
        "pinocchio is required by dexim.core.model but is not installed. "
        "Install it via conda: conda install pinocchio -c conda-forge"
    ) from _err

from .helpers import replace_package_url_in_content


def load_urdf_model(
    packages_dir: str | Path,
    description_name: str,
    urdf_name: str,
    mimic: bool = False,
    verbose: bool = False,
) -> pin.Model:
    """
    Load a Pinocchio Model from a URDF file.

    This function handles package:// URL replacement and temporary file creation
    when needed. It provides a unified interface for loading URDF models across
    the codebase.

    Args:
        packages_dir: Base directory containing robot description packages
        description_name: Name of the robot description package/folder
        urdf_name: Name of the URDF file (e.g., "robot.urdf")
        mimic: Whether to parse mimic joints (default: False)
        verbose: Whether to print verbose output (default: False)

    Returns:
        Pinocchio Model loaded from URDF

    Raises:
        FileNotFoundError: If constructed paths do not exist
        ValueError: If package directories do not exist or URDF loading fails

    Example:
        >>> model = load_urdf_model("assets", "inspire_hand_l", "inspire_hand_l.urdf")
    """

    # Convert to Path for consistent handling
    packages_dir = Path(packages_dir)

    # Construct paths following ROS package structure
    description_dir = packages_dir / description_name
    urdf_path = description_dir / "urdf" / urdf_name

    if not packages_dir.exists():
        raise ValueError(f"Packages directory not found: {packages_dir}")

    if not description_dir.exists():
        raise ValueError(f"Description directory not found: {description_dir}")

    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF file not found: {urdf_path}")

    # Read URDF content
    with open(urdf_path) as f:
        urdf_content = f.read()

    # Replace package:// URLs with absolute paths
    # Use packages_dir as the base for resolving package:// references
    modified_content = replace_package_url_in_content(
        urdf_content, str(packages_dir), description_name
    )

    # Create temporary file with modified content
    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".urdf", delete=False
    ) as temp_file:
        temp_file.write(modified_content)
        temp_urdf_path = temp_file.name

    try:
        # Load model from temporary file
        model = pin.buildModelFromUrdf(temp_urdf_path, mimic=mimic)  # type: ignore
        if verbose:
            print(f"✅ Model loaded from {urdf_path} (via temp file)")
    finally:
        # Clean up temporary file
        os.unlink(temp_urdf_path)

    return model


def load_urdf_models(
    packages_dir: str | Path,
    description_name: str,
    urdf_name: str,
    mimic: bool = False,
    verbose: bool = False,
) -> tuple[pin.Model, pin.GeometryModel, pin.GeometryModel]:
    """
    Load Pinocchio Model and GeometryModels from a URDF file.

    This function handles package:// URL replacement and temporary file creation
    when needed. It returns the model along with collision and visual geometry models.

    Args:
        packages_dir: Base directory containing robot description packages
        description_name: Name of the robot description package/folder
        urdf_name: Name of the URDF file (e.g., "robot.urdf")
        mimic: Whether to parse mimic joints (default: False)
        verbose: Whether to print verbose output (default: False)

    Returns:
        Tuple of (model, collision_model, visual_model)
        If geometry models are not present in URDF, empty GeometryModel objects are returned.

    Raises:
        FileNotFoundError: If constructed paths do not exist
        ValueError: If package directories do not exist or URDF loading fails

    Example:
        >>> model, collision, visual = load_urdf_models(
        ...     "assets",
        ...     "inspire_hand_l",
        ...     "inspire_hand_l.urdf",
        ...     mimic=True
        ... )
    """

    # Convert to Path for consistent handling
    packages_dir = Path(packages_dir)

    # Construct paths following ROS package structure
    description_dir = packages_dir / description_name
    urdf_path = description_dir / "urdf" / urdf_name

    if not packages_dir.exists():
        raise ValueError(f"Packages directory not found: {packages_dir}")

    if not description_dir.exists():
        raise ValueError(f"Description directory not found: {description_dir}")

    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF file not found: {urdf_path}")

    # Read URDF content
    with open(urdf_path) as f:
        urdf_content = f.read()

    # Replace package:// URLs with absolute paths
    # Use packages_dir as the base for resolving package:// references
    modified_content = replace_package_url_in_content(
        urdf_content, str(packages_dir), description_name
    )

    # Create temporary file with modified content
    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".urdf", delete=False
    ) as temp_file:
        temp_file.write(modified_content)
        temp_urdf_path = temp_file.name

    try:
        # Load models from temporary file
        model, collision_model, visual_model = pin.buildModelsFromUrdf(
            temp_urdf_path,
            verbose=verbose,
            mimic=mimic,  # type: ignore
        )
        if verbose:
            print(f"✅ Models loaded from {urdf_path} (via temp file)")
    finally:
        # Clean up temporary file
        os.unlink(temp_urdf_path)

    return model, collision_model, visual_model


__all__ = ["load_urdf_model", "load_urdf_models"]
