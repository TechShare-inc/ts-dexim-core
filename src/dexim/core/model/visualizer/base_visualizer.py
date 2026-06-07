"""
Base visualizer class for robot visualization with Viser.

This module provides a common base class for robot visualizers,
containing shared functionality for visibility control, server management,
and common visualization patterns.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np
from loguru import logger

try:
    import pinocchio as pin
except ImportError as _err:
    raise ImportError(
        "pinocchio is required by dexim.core.model.visualizer but is not installed. "
        "Install it via conda: conda install pinocchio -c conda-forge"
    ) from _err

try:
    import viser
    from viser import ViserServer
except ImportError as _err:
    raise ImportError(
        "viser is required by dexim.core.model.visualizer but is not installed. "
        "Install it via pip: pip install dexim-core[viz]"
    ) from _err


class BaseRobotVisualizer(ABC):
    """
    Abstract base class for robot visualizers using Viser.

    This class provides common functionality for robot visualization including:
    - Viser server management
    - Visibility control for different visualization components
    - Configuration update interface
    - Context manager support
    - Common utility methods

    Subclasses must implement:
    - _create_geometry_objects(): Create robot geometry visualization
    - _create_frame_visualizations(): Create frame visualizations
    """

    def __init__(
        self,
        server: ViserServer | None = None,
        port: int = 8080,
        verbose: bool = True,
        show_frames: bool = True,
        show_geometry: bool = True,
        show_ee_spheres: bool = True,
    ):
        """
        Initialize the base visualizer.

        Args:
            server: Optional existing Viser server instance
            port: Port for the Viser server (if creating new server)
            verbose: Whether to print status messages
            show_frames: Whether to show end-effector frame axes
            show_geometry: Whether to show robot geometry (links/joints)
            show_ee_spheres: Whether to show end-effector spheres
        """

        self.verbose = verbose
        self.logger = logger

        # Visibility flags
        self._show_frames = show_frames
        self._show_geometry = show_geometry
        self._show_ee_spheres = show_ee_spheres

        # Initialize Viser server
        if server is None:
            self.server = viser.ViserServer(port=port)
            self.owns_server = True
        else:
            self.server = server
            self.owns_server = False

        # Visualization components (to be populated by subclasses)
        self.frame_handles: dict[str, viser.SceneNodeHandle] = {}
        self.geometry_handles: dict[str, viser.SceneNodeHandle] = {}
        self.feature_handles: dict[
            str, tuple[viser.LineSegmentsHandle, viser.IcosphereHandle]
        ] = {}

    @abstractmethod
    def _create_geometry_objects(self) -> None:
        """
        Create visual representations of robot geometry.

        This method must be implemented by subclasses to create
        appropriate visual representations for their specific robot type.
        """

        pass

    @abstractmethod
    def _create_frame_visualizations(self) -> None:
        """
        Create interactive frame visualizations.

        This method must be implemented by subclasses to create
        frame visualizations specific to their robot type.
        """

        pass

    @abstractmethod
    def update_configuration(self, q: np.ndarray) -> None:
        """
        Update robot configuration and refresh visualization.

        Args:
            q: Joint configuration vector

        This method must be implemented by subclasses to update
        their specific robot model and visualization.
        """

        pass

    @abstractmethod
    def get_current_configuration(self) -> np.ndarray:
        """
        Get the current joint configuration.

        Returns:
            Current joint configuration vector
        """

        pass

    @abstractmethod
    def get_current_ee_poses(self) -> dict[str, pin.SE3]:
        """
        Get current end-effector poses.

        Returns:
            Dictionary mapping frame names to SE3 poses
        """

        pass

    @abstractmethod
    def reset_to_neutral(self) -> None:
        """Reset robot to neutral configuration."""

        pass

    # Visibility control properties and methods
    @property
    def show_frames(self) -> bool:
        """Whether end-effector frames are visible."""

        return self._show_frames

    @show_frames.setter
    def show_frames(self, visible: bool) -> None:
        """Set visibility of end-effector frames."""

        if self._show_frames == visible:
            return

        self._show_frames = visible
        for name, handle in self.frame_handles.items():
            if not name.startswith("sphere_"):  # Don't affect spheres
                handle.visible = visible

        if self.verbose:
            state = "visible" if visible else "hidden"
            self.logger.info(f"End-effector frames are now {state}")

    @property
    def show_geometry(self) -> bool:
        """Whether robot geometry is visible."""

        return self._show_geometry

    @show_geometry.setter
    def show_geometry(self, visible: bool) -> None:
        """Set visibility of robot geometry."""

        if self._show_geometry == visible:
            return

        self._show_geometry = visible
        for name, handle in self.geometry_handles.items():
            if not name.startswith("ee_sphere_"):  # Don't affect end-effector spheres
                handle.visible = visible

        if self.verbose:
            state = "visible" if visible else "hidden"
            self.logger.info(f"Robot geometry is now {state}")

    @property
    def show_ee_spheres(self) -> bool:
        """Whether end-effector spheres are visible."""

        return self._show_ee_spheres

    @show_ee_spheres.setter
    def show_ee_spheres(self, visible: bool) -> None:
        """Set visibility of end-effector spheres."""

        if self._show_ee_spheres == visible:
            return

        self._show_ee_spheres = visible
        # Subclasses should override this to properly handle their sphere visibility
        if self.verbose:
            state = "visible" if visible else "hidden"
            self.logger.info(f"End-effector spheres are now {state}")

    def toggle_frames(self) -> None:
        """Toggle visibility of end-effector frames."""

        self.show_frames = not self.show_frames

    def toggle_geometry(self) -> None:
        """Toggle visibility of robot geometry."""

        self.show_geometry = not self.show_geometry

    def toggle_ee_spheres(self) -> None:
        """Toggle visibility of end-effector spheres."""

        self.show_ee_spheres = not self.show_ee_spheres

    def show_all(self) -> None:
        """Show all visualization components."""

        self.show_frames = True
        self.show_geometry = True
        self.show_ee_spheres = True

        if self.verbose:
            self.logger.info("All visualization components are now visible")

    def hide_all(self) -> None:
        """Hide all visualization components."""

        self.show_frames = False
        self.show_geometry = False
        self.show_ee_spheres = False

        if self.verbose:
            self.logger.info("All visualization components are now hidden")

    def get_visibility_status(self) -> dict[str, bool]:
        """Get current visibility status of all components."""

        return {
            "frames": self._show_frames,
            "geometry": self._show_geometry,
            "ee_spheres": self._show_ee_spheres,
        }

    @abstractmethod
    def update_feature_vectors(self, feature_vectors: dict[str, np.ndarray]) -> None:
        """Update the visualization of input feature vectors.

        Args:
            feature_vectors: Dictionary mapping frame names to 3D feature vectors.
        """

    def close(self) -> None:
        """Close the visualizer and clean up resources."""

        # Clear handles
        self.frame_handles.clear()
        self.geometry_handles.clear()
        self.feature_handles.clear()

        if self.owns_server:
            # Note: Viser doesn't have a direct close method,
            # but the server will clean up when the object is destroyed
            pass

        if self.verbose:
            self.logger.info("Visualizer closed")

    def __enter__(self):
        """Context manager entry."""

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""

        self.close()

    @abstractmethod
    def __repr__(self) -> str:
        """String representation of the visualizer."""

        pass


__all__ = ["BaseRobotVisualizer"]
