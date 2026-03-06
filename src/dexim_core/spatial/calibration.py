"""
Calibration utilities for tracker-to-world frame registration.

This module provides functions for:
- 3-point rigid registration calibration for world frame
- Saving/loading calibration data
- Interactive calibration workflow
- Iterator-based calibration for TUI applications

The world frame calibration establishes a transformation between the tracker
coordinate system and the robot world frame, enabling consistent spatial
mapping regardless of the number of robots or trackers in the workspace.

Author: Haoyan Li
Date: November 19, 2025
"""

import json
from collections.abc import Generator
from datetime import datetime
from pathlib import Path
from typing import (
    Any,
    Protocol,
    runtime_checkable,
)

import numpy as np
import numpy.typing as npt
from loguru import logger

from .transform import Transform3D, rigid_transform


@runtime_checkable
class TrackerDataProtocol(Protocol):
    """Protocol for tracker data from parsing."""

    transform: Transform3D


@runtime_checkable
class DataSubscriberProtocol(Protocol):
    """Protocol for data subscribers (e.g., ManusSubscriber)."""

    def receive(self) -> Any:
        """Receive data from subscriber."""
        ...

    def receive_latest(self) -> Any:
        """Receive latest data from subscriber."""
        ...


@runtime_checkable
class TrackerParserProtocol(Protocol):
    """Protocol for tracker data parsers."""

    def parse_tracker_by_type(
        self, data: Any, tracker_type: str
    ) -> TrackerDataProtocol | None:
        """Parse tracker data by type."""
        ...


def calibrate_world(
    origin: Transform3D, x_point: Transform3D, y_point: Transform3D
) -> npt.NDArray:
    """
    Calibrate world frame using 3-point rigid registration.

    This function normalizes the measured points by using their actual distances
    to define the reference coordinate system, then uses ts_spatial.rigid_transform
    to compute the optimal transformation.

    Args:
        origin: Tracker pose at world origin
        x_point: Tracker pose at +X axis point
        y_point: Tracker pose at +Y axis point

    Returns:
        4x4 transformation matrix (world to tracker base)
    """
    # Measured points in tracker frame
    measured_origin = origin.position
    measured_x = x_point.position
    measured_y = y_point.position

    # Calculate actual distances from measured points (normalization)
    x_distance = np.linalg.norm(measured_x - measured_origin)
    y_distance = np.linalg.norm(measured_y - measured_origin)

    logger.info(f"Measured X-axis distance: {x_distance:.4f}m")
    logger.info(f"Measured Y-axis distance: {y_distance:.4f}m")

    # Reference points using MEASURED distances (normalized coordinate system)
    ref_origin = np.array([0.0, 0.0, 0.0])
    ref_x = np.array([x_distance, 0.0, 0.0])
    ref_y = np.array([0.0, y_distance, 0.0])

    # Stack points for rigid registration (N x 3 format)
    measured_points = np.vstack([measured_origin, measured_x, measured_y])
    reference_points = np.vstack([ref_origin, ref_x, ref_y])

    # Use ts_spatial.rigid_transform for robust rigid transformation
    # This computes: reference_points = R @ measured_points + t
    R_matrix, t_vector, scale = rigid_transform(
        measured_points, reference_points, calc_scale=False
    )

    logger.info("✅ Transformation matrix calculated using ts_spatial.rigid_transform")
    logger.debug(f"Rotation matrix determinant: {np.linalg.det(R_matrix):.6f}")
    logger.debug(f"Translation vector: {t_vector.flatten()}")

    # Build 4x4 homogeneous transformation matrix
    wM_base = np.eye(4, dtype=np.float64)
    wM_base[:3, :3] = R_matrix
    wM_base[:3, 3] = t_vector.flatten()

    return wM_base


def save_world_calibration(
    matrix: npt.NDArray,
    output_dir: Path = Path("calibrations"),
    reference_points: dict | None = None,
    measured_points: dict | None = None,
    error_mm: float | None = None,
) -> Path:
    """
    Save world calibration matrix to JSON file.

    Args:
        matrix: 4x4 transformation matrix
        output_dir: Directory to save calibration file
        reference_points: Reference points used for calibration
        measured_points: Measured points used for calibration
        error_mm: Calibration error in millimeters

    Returns:
        Path to saved calibration file
    """
    # Create output directory
    output_dir.mkdir(parents=True, exist_ok=True)

    # Generate filename with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"world_calibration_{timestamp}.json"
    filepath = output_dir / filename

    # Prepare calibration data
    calibration_data = {
        "world_to_tracker": matrix.tolist(),
        "timestamp": timestamp,
    }

    if error_mm is not None:
        calibration_data["error_mm"] = error_mm

    if reference_points is not None:
        calibration_data["reference_points"] = reference_points

    if measured_points is not None:
        calibration_data["measured_points"] = measured_points

    # Save to JSON
    with open(filepath, "w") as f:
        json.dump(calibration_data, f, indent=2)

    logger.success(f"World calibration saved to: {filepath}")
    return filepath


def load_world_calibration(calib_dir: Path = Path("calibrations")) -> npt.NDArray:
    """
    Load world calibration matrix from JSON file.

    Args:
        calib_dir: Directory containing calibration files (default: "calibrations")

    Returns:
        4x4 transformation matrix

    Raises:
        FileNotFoundError: If calibration file not found
    """
    if not calib_dir.exists():
        raise FileNotFoundError(f"Calibration directory not found: {calib_dir}")

    calib_files = sorted(calib_dir.glob("world_calibration_*.json"))
    if not calib_files:
        raise FileNotFoundError(f"No world calibration files found in {calib_dir}")

    filepath = calib_files[-1]
    logger.info(f"Loading world calibration: {filepath}")

    with open(filepath) as f:
        data = json.load(f)

    matrix = np.array(data["world_to_tracker"])
    return matrix


def calibrate_world_interactive(
    subscriber: DataSubscriberProtocol,
    parser: TrackerParserProtocol,
    tracker_type: str = "left_hand",
) -> tuple[npt.NDArray, dict, dict]:
    """
    Interactive world frame calibration workflow.

    Args:
        subscriber: Data subscriber instance (e.g., ManusSubscriber)
        parser: Tracker parser instance (e.g., ManusTrackerParser)
        tracker_type: Tracker type to use

    Returns:
        Tuple of (calibration_matrix, reference_points, measured_points)
    """
    logger.info("Starting interactive world frame calibration...")
    logger.info("You will be asked to position the tracker at 3 reference points.")

    # Point 1: Origin
    input("\nPress Enter when tracker is at ORIGIN (world origin)...")
    data = subscriber.receive_latest()
    origin_data = parser.parse_tracker_by_type(data, tracker_type)
    if origin_data is None:
        raise ValueError("Failed to get tracker data at origin")
    origin = origin_data.transform
    logger.success(f"Origin captured: {origin.position}")

    # Point 2: X-axis
    input("\nPress Enter when tracker is at +X AXIS (10cm along X)...")
    data = subscriber.receive_latest()
    x_data = parser.parse_tracker_by_type(data, tracker_type)
    if x_data is None:
        raise ValueError("Failed to get tracker data at X point")
    x_point = x_data.transform
    logger.success(f"X-axis captured: {x_point.position}")

    # Point 3: Y-axis
    input("\nPress Enter when tracker is at +Y AXIS (10cm along Y)...")
    data = subscriber.receive_latest()
    y_data = parser.parse_tracker_by_type(data, tracker_type)
    if y_data is None:
        raise ValueError("Failed to get tracker data at Y point")
    y_point = y_data.transform
    logger.success(f"Y-axis captured: {y_point.position}")

    # Compute calibration
    logger.info("Computing calibration...")
    wM_base = calibrate_world(origin, x_point, y_point)

    # Compute error
    # Calculate actual distances from measured points (normalization)
    x_distance = np.linalg.norm(x_point.position - origin.position)
    y_distance = np.linalg.norm(y_point.position - origin.position)

    ref_points = {
        "origin": [0.0, 0.0, 0.0],
        "x_axis": [x_distance, 0.0, 0.0],
        "y_axis": [0.0, y_distance, 0.0],
    }

    meas_points = {
        "origin": origin.position.tolist(),
        "x_axis": x_point.position.tolist(),
        "y_axis": y_point.position.tolist(),
    }

    # Compute reprojection error
    ref_array = np.array(
        [ref_points["origin"], ref_points["x_axis"], ref_points["y_axis"]]
    )
    meas_array = np.array(
        [meas_points["origin"], meas_points["x_axis"], meas_points["y_axis"]]
    )

    ref_homogeneous = np.hstack([ref_array, np.ones((3, 1))])
    transformed = (wM_base @ ref_homogeneous.T).T[:, :3]
    error = np.linalg.norm(transformed - meas_array, axis=1)
    error_mm = float(np.mean(error) * 1000.0)

    logger.success(f"Calibration complete! Average error: {error_mm:.2f} mm")

    return wM_base, ref_points, meas_points


def calibrate_world_iter(
    subscriber: DataSubscriberProtocol,
    parser: TrackerParserProtocol,
    tracker_type: str = "left_hand",
) -> Generator[tuple[str, Transform3D | None], None, tuple[npt.NDArray, dict, dict]]:
    """
    Iterator-based world frame calibration for TUI applications.

    This generator yields calibration steps and waits for tracker data input,
    enabling integration with interactive TUI frameworks.

    Yields:
        Tuple of (step_description, captured_transform or None)
            - step_description: Human-readable step instruction
            - captured_transform: Transform3D if data captured, None if waiting

    Returns:
        Tuple of (calibration_matrix, reference_points, measured_points) on completion

    Example:
        >>> calibrator = calibrate_world_iter(subscriber, parser, "left_hand")
        >>> for step_desc, transform in calibrator:
        ...     if transform is None:
        ...         # Display step_desc, wait for user input
        ...         user_ready = await get_user_input(step_desc)
        ...         calibrator.send(user_ready)
        ...     else:
        ...         # Display captured transform
        ...         print(f"Captured: {transform.position}")
    """
    logger.info("Starting iterator-based world frame calibration...")

    # Step 1: Capture origin
    yield ("Position tracker at ORIGIN (world origin) and confirm", None)
    data = subscriber.receive()
    origin_data = parser.parse_tracker_by_type(data, tracker_type)
    if origin_data is None:
        raise ValueError("Failed to get tracker data at origin")
    origin = origin_data.transform
    logger.success(f"Origin captured: {origin.position}")
    yield ("Origin captured", origin)

    # Step 2: Capture X-axis point
    yield ("Position tracker at +X AXIS (10cm along X) and confirm", None)
    data = subscriber.receive()
    x_data = parser.parse_tracker_by_type(data, tracker_type)
    if x_data is None:
        raise ValueError("Failed to get tracker data at X point")
    x_point = x_data.transform
    logger.success(f"X-axis captured: {x_point.position}")
    yield ("X-axis captured", x_point)

    # Step 3: Capture Y-axis point
    yield ("Position tracker at +Y AXIS (10cm along Y) and confirm", None)
    data = subscriber.receive()
    y_data = parser.parse_tracker_by_type(data, tracker_type)
    if y_data is None:
        raise ValueError("Failed to get tracker data at Y point")
    y_point = y_data.transform
    logger.success(f"Y-axis captured: {y_point.position}")
    yield ("Y-axis captured", y_point)

    # Compute calibration
    logger.info("Computing calibration...")
    wM_base = calibrate_world(origin, x_point, y_point)

    # Prepare results
    ref_points = {
        "origin": [0.0, 0.0, 0.0],
        "x_axis": [0.1, 0.0, 0.0],
        "y_axis": [0.0, 0.1, 0.0],
    }

    meas_points = {
        "origin": origin.position.tolist(),
        "x_axis": x_point.position.tolist(),
        "y_axis": y_point.position.tolist(),
    }

    # Compute reprojection error
    ref_array = np.array(
        [ref_points["origin"], ref_points["x_axis"], ref_points["y_axis"]]
    )
    meas_array = np.array(
        [meas_points["origin"], meas_points["x_axis"], meas_points["y_axis"]]
    )

    ref_homogeneous = np.hstack([ref_array, np.ones((3, 1))])
    transformed = (wM_base @ ref_homogeneous.T).T[:, :3]
    error = np.linalg.norm(transformed - meas_array, axis=1)
    error_mm = np.mean(error) * 1000

    logger.success(f"Calibration complete! Average error: {error_mm:.2f} mm")

    return wM_base, ref_points, meas_points


# ============================================================================
# Backward compatibility aliases for Nova tracker calibration
# ============================================================================


def calibrate_nova_tracker(
    origin: Transform3D, x_point: Transform3D, y_point: Transform3D
) -> npt.NDArray:
    """
    Calibrate Nova tracker using 3-point rigid registration.

    DEPRECATED: Use calibrate_world() instead for consistent naming.
    This function is maintained for backward compatibility.

    Args:
        origin: Tracker pose at robot base origin
        x_point: Tracker pose at +X axis point
        y_point: Tracker pose at +Y axis point

    Returns:
        4x4 transformation matrix (world to tracker base)
    """
    import warnings

    warnings.warn(
        "calibrate_nova_tracker is deprecated, use calibrate_world instead",
        DeprecationWarning,
        stacklevel=2,
    )
    return calibrate_world(origin, x_point, y_point)


def save_calibration(
    matrix: npt.NDArray,
    output_dir: Path = Path("calibrations"),
    reference_points: dict | None = None,
    measured_points: dict | None = None,
    error_mm: float | None = None,
) -> Path:
    """
    Save calibration matrix to JSON file (Nova naming convention).

    DEPRECATED: Use save_world_calibration() for new code.
    This function saves with 'nova_calibration_*.json' naming for backward compatibility.

    Args:
        matrix: 4x4 transformation matrix
        output_dir: Directory to save calibration file
        reference_points: Reference points used for calibration
        measured_points: Measured points used for calibration
        error_mm: Calibration error in millimeters

    Returns:
        Path to saved calibration file
    """
    import warnings

    warnings.warn(
        "save_calibration is deprecated, use save_world_calibration instead",
        DeprecationWarning,
        stacklevel=2,
    )

    # Create output directory
    output_dir.mkdir(parents=True, exist_ok=True)

    # Generate filename with timestamp (Nova convention)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"nova_calibration_{timestamp}.json"
    filepath = output_dir / filename

    # Prepare calibration data
    calibration_data = {
        "world_to_tracker": matrix.tolist(),
        "timestamp": timestamp,
    }

    if error_mm is not None:
        calibration_data["error_mm"] = error_mm

    if reference_points is not None:
        calibration_data["reference_points"] = reference_points

    if measured_points is not None:
        calibration_data["measured_points"] = measured_points

    # Save to JSON
    with open(filepath, "w") as f:
        json.dump(calibration_data, f, indent=2)

    logger.success(f"Calibration saved to: {filepath}")
    return filepath


def load_calibration(calib_dir: Path = Path("calibrations")) -> npt.NDArray:
    """
    Load calibration matrix from JSON file (Nova naming convention).

    DEPRECATED: Use load_world_calibration() for new code.
    This function looks for 'nova_calibration_*.json' files for backward compatibility.

    Args:
        calib_dir: Directory containing calibration files (default: "calibrations")

    Returns:
        4x4 transformation matrix

    Raises:
        FileNotFoundError: If calibration file not found
    """
    import warnings

    warnings.warn(
        "load_calibration is deprecated, use load_world_calibration instead",
        DeprecationWarning,
        stacklevel=2,
    )

    if not calib_dir.exists():
        raise FileNotFoundError(f"Calibration directory not found: {calib_dir}")

    calib_files = sorted(calib_dir.glob("nova_calibration_*.json"))
    if not calib_files:
        raise FileNotFoundError(f"No calibration files found in {calib_dir}")

    filepath = calib_files[-1]
    logger.info(f"Loading calibration: {filepath}")

    with open(filepath) as f:
        data = json.load(f)

    matrix = np.array(data["world_to_tracker"])
    return matrix


def calibrate_nova_tracker_interactive(
    subscriber: DataSubscriberProtocol,
    parser: TrackerParserProtocol,
    tracker_type: str = "left_hand",
) -> tuple[npt.NDArray, dict, dict]:
    """
    Interactive calibration workflow (Nova naming convention).

    DEPRECATED: Use calibrate_world_interactive() for new code.
    Maintained for backward compatibility.

    Args:
        subscriber: Data subscriber instance
        parser: Tracker parser instance
        tracker_type: Tracker type to use

    Returns:
        Tuple of (calibration_matrix, reference_points, measured_points)
    """
    import warnings

    warnings.warn(
        "calibrate_nova_tracker_interactive is deprecated, use calibrate_world_interactive instead",
        DeprecationWarning,
        stacklevel=2,
    )
    return calibrate_world_interactive(subscriber, parser, tracker_type)
