"""ArmTeleopNode - Base class for arm teleoperation using tracker data.

This module provides the ArmTeleopNode abstract base class for arm robots
that use tracker data for teleoperation with IK-based control:

- Tracker reference pose capture on START command
- World calibration transformation support
- Relative pose computation for IK
- Home configuration setup (moves robot to home position)

Arm robots (Nova, etc.) extend this class and implement robot-specific IK.

Note: Tracker data is NOT required during initialization. The node will:
1. During setup: Move to home position without tracker dependency
2. On START command: Move to home, then capture tracker reference pose

Example:
    from dexim_core.nodes import ArmTeleopNode

    class NovaControlNode(ArmTeleopNode):
        def setup(self):
            self.subscriber = ManusSubscriber(...)
            self.model = NovaModel(...)
            self.interface = NovaInterface(...)

        def process_data(self, data):
            # Get relative pose and solve IK
            target_pose = self._compute_target_pose(tracker_pose)
            return self.model.retarget(target_pose=target_pose)

    # setup_home_configuration() is called automatically after setup()
    with NovaControlNode("nova_left", config) as node:
        node.run()
"""

from __future__ import annotations

from abc import abstractmethod

import numpy as np
from loguru import logger
from dexim_core.spatial import Transform3D

from dexim_core.nodes.protocols import (
    DEFAULT_SENSOR_WAIT_CONFIG,
    ControlNodeConfig,
    RobotModelProtocol,
    SensorWaitConfig,
    TrackerDataProtocol,
)
from dexim_core.nodes.teleop_node import TeleopNode


class ArmTeleopNode(TeleopNode):
    """Base class for arm teleoperation using tracker data.

    This class extends TeleopNode with arm-specific features:
    - Tracker reference pose capture for relative motion (on START command)
    - World calibration transformation
    - Home configuration setup (moves robot to home position)
    - IK-based control pipeline

    Subclasses must implement:
    - setup(): Initialize subscriber, model, interface
    - process_data(): Compute IK from tracker data
    - get_safe_position(): Return home joint configuration

    Attributes:
        tracker_pose_home: Reference tracker pose captured on START
        ee_pose_home: End-effector pose at home configuration
        q_home: Home joint configuration
        q_previous: Previous joint configuration (for IK warm start)
        wM_base: World-to-base calibration matrix (4x4)
        model: Robot kinematic model (RobotModelProtocol)
        handedness: Tracker handedness ("left" or "right")
    """

    def __init__(self, node_id: str, config: ControlNodeConfig):
        """Initialize arm teleop node.

        Args:
            node_id: Unique identifier (e.g., "nova_left", "nova_right")
            config: Control node configuration
        """
        super().__init__(node_id=node_id, config=config)

        # Home poses (set by setup_home_configuration)
        self.tracker_pose_home: Transform3D | None = None
        self.ee_pose_home: Transform3D | None = None

        # Joint configurations
        self.q_home: np.ndarray | None = None
        self.q_previous: np.ndarray | None = None

        # Calibration transformation (identity by default)
        self.wM_base: np.ndarray = np.eye(4)

        # Robot model (set by setup)
        self.model: RobotModelProtocol

        # Tracker configuration
        self.handedness: str = "left"  # Override in subclass

        # Sensor wait configuration (can be overridden by subclass or config)
        self.sensor_wait_config: SensorWaitConfig = DEFAULT_SENSOR_WAIT_CONFIG

    @property
    def tracker_type(self) -> str:
        """Get tracker type string based on handedness."""
        return f"{self.handedness}_hand"

    # ----------------------
    # Home configuration
    # ----------------------
    def setup_home_configuration(self) -> None:
        """Set up home configuration for teleoperation.

        This method is called automatically after setup() by TeleopNode._post_setup().
        It:
        1. Sets q_home from config or neutral
        2. Computes end-effector home pose via FK
        3. Moves smoothly to home configuration

        Note: Tracker reference pose is captured later in on_start() when the user
        presses START, not during initial setup. This allows the node to initialize
        without requiring tracker data to be available.

        Subclasses may override to add robot-specific logic.

        Raises:
            RuntimeError: If smooth movement to home position fails
        """
        logger.info("Setting up home configuration...")

        # Step 1: Set q_home from config or neutral
        self.q_home = self._get_home_joints()
        logger.info("Using home joint configuration")

        # Step 2: Compute end-effector home pose via FK
        self.ee_pose_home = self._compute_ee_pose(self.q_home)
        logger.info(f"End-effector home position: {self.ee_pose_home.position}")

        # Initialize previous configuration for IK warm start
        self.q_previous = self.q_home.copy()

        # Step 3: Move to home configuration smoothly
        logger.info("Moving to home configuration...")
        success = self._move_to_position_safely(
            self.q_home,
            timeout_sec=10.0,
            max_velocity_rad_s=1.0,  # Gentle for initial homing
        )

        if not success:
            raise RuntimeError(
                "Failed to move to home configuration: movement timed out"
            )

        logger.success("Home configuration setup complete")

    def _find_tracker_by_type(
        self, tracker_list: list[TrackerDataProtocol], tracker_type: str
    ) -> TrackerDataProtocol | None:
        """Find tracker by type from list of TrackerData objects.

        Args:
            tracker_list: List of TrackerData objects
            tracker_type: Type to find (e.g., "left_hand")

        Returns:
            Matching TrackerData or None
        """
        for tracker in tracker_list:
            if tracker.tracker_type == tracker_type:
                return tracker
        return None

    def _apply_calibration(self, tracker_transform: Transform3D) -> Transform3D:
        """Apply world calibration to tracker transform.

        Args:
            tracker_transform: Raw tracker Transform3D

        Returns:
            Calibrated Transform3D
        """
        wM_base_transform = Transform3D.from_matrix(self.wM_base)
        return wM_base_transform * tracker_transform

    @abstractmethod
    def _get_home_joints(self) -> np.ndarray:
        """Get home joint configuration from config or neutral.

        Returns:
            Home joint configuration array

        Subclasses should implement based on their config structure.
        """
        pass

    @abstractmethod
    def _compute_ee_pose(self, q: np.ndarray) -> Transform3D:
        """Compute end-effector pose via forward kinematics.

        Args:
            q: Joint configuration

        Returns:
            End-effector Transform3D
        """
        pass

    # ----------------------
    # Calibration
    # ----------------------
    def load_world_calibration(self, calibration_path: str | None = None) -> None:
        """Load world-to-base calibration transformation.

        Args:
            calibration_path: Path to calibration file. If None, uses identity.

        Note:
            Subclasses may override to implement robot-specific calibration loading.
        """
        if calibration_path is None:
            logger.info("No calibration path provided, using identity transform")
            self.wM_base = np.eye(4)
            return

        try:
            # Try to load calibration - subclasses may provide specific loaders
            import json
            from pathlib import Path

            cal_path = Path(calibration_path)
            if cal_path.exists():
                with open(cal_path) as f:
                    cal_data = json.load(f)
                    if "wM_base" in cal_data:
                        self.wM_base = np.array(cal_data["wM_base"])
                        logger.success(f"Loaded calibration from {calibration_path}")
                        return

            logger.warning(f"Calibration file not found: {calibration_path}")
            self.wM_base = np.eye(4)

        except Exception as e:
            logger.warning(f"Failed to load calibration: {e}, using identity")
            self.wM_base = np.eye(4)

    # ----------------------
    # Reference pose capture
    # ----------------------
    def _capture_reference_pose(self, max_attempts: int = 20) -> bool:
        """Capture current tracker pose as reference for relative control.

        Called by on_start() to capture reference pose. Retries a few times
        to ensure tracker data is available and fresh (not stale buffered data).

        Args:
            max_attempts: Maximum number of read attempts (default: 20)

        Returns:
            True if successful, False otherwise
        """
        import time

        # Threshold for stale data - reject data older than this (60Hz tracker = ~17ms between frames)
        STALE_THRESHOLD_SEC = 0.1  # 100ms

        for attempt in range(max_attempts):
            try:
                current_time = time.time()
                data = self.subscriber.read()
                trackers = data.get("trackers", [])

                if not trackers:
                    logger.debug(
                        f"Attempt {attempt + 1}/{max_attempts}: No tracker data yet, waiting..."
                    )
                    time.sleep(0.05)
                    continue

                tracker = self._find_tracker_by_type(trackers, self.tracker_type)
                if tracker is None:
                    logger.debug(
                        f"Attempt {attempt + 1}/{max_attempts}: Tracker '{self.tracker_type}' not found, waiting..."
                    )
                    time.sleep(0.05)
                    continue

                # Check for stale data (buffered from before pause)
                if hasattr(tracker, "timestamp") and tracker.timestamp > 0:
                    data_age = current_time - tracker.timestamp
                    if data_age > STALE_THRESHOLD_SEC:
                        logger.debug(
                            f"Attempt {attempt + 1}/{max_attempts}: Skipping stale data (age: {data_age:.3f}s)"
                        )
                        # Don't sleep - consume queue quickly
                        continue

                # Apply calibration and update home pose
                self.tracker_pose_home = self._apply_calibration(tracker.transform)

                # Update EE home pose via FK at current position
                try:
                    current_state = self.interface.read()
                    q_home = current_state.q.copy()
                    self.q_home = q_home
                    self.ee_pose_home = self._compute_ee_pose(q_home)
                    self.q_previous = q_home.copy()
                except Exception as e:
                    logger.warning(f"Could not read current state: {e}")

                logger.info(
                    f"Reference pose captured: {self.tracker_pose_home.position}"
                )
                return True

            except Exception as e:
                logger.error(
                    f"Error during reference capture attempt {attempt + 1}: {e}"
                )
                time.sleep(0.1)

        logger.warning(
            f"Failed to capture reference pose after {max_attempts} attempts. "
            f"Tracker type: {self.tracker_type}"
        )
        return False

    # ----------------------
    # Relative pose computation
    # ----------------------
    def compute_target_pose(self, current_tracker_pose: Transform3D) -> Transform3D:
        """Compute target end-effector pose from current tracker pose.

        Uses relative motion from reference (GLOBAL FRAME approach):
        - Δp = p_tracker - p_tracker_home
        - p_target = p_ee_home + Δp
        - ΔR_global = R_tracker * R_tracker_home⁻¹ (delta in global frame)
        - R_target = ΔR_global * R_ee_home (pre-multiply for global rotation)

        Args:
            current_tracker_pose: Current tracker Transform3D

        Returns:
            Target end-effector Transform3D
        """
        if self.tracker_pose_home is None or self.ee_pose_home is None:
            raise RuntimeError(
                "Home configuration not set. Call setup_home_configuration() first."
            )

        try:
            from dexim_core.spatial import Transform3D

            # Relative position: Δp = p_tracker - p_tracker_home
            delta_position = (
                current_tracker_pose.position - self.tracker_pose_home.position
            )

            # Target position: p_target = p_ee_home + Δp
            target_position = self.ee_pose_home.position + delta_position

            # Relative rotation in GLOBAL frame:
            # ΔR_global = R_tracker_current * R_tracker_home^(-1)
            # R_target = ΔR_global * R_ee_home
            #          = R_tracker_current * R_tracker_home^(-1) * R_ee_home
            #
            # This computes the rotation change in the world/global frame,
            # then applies that global rotation to the EE home orientation.
            tracker_home_rot_inv = self.tracker_pose_home.inverse().rotation_matrix
            delta_rotation_global = (
                current_tracker_pose.rotation_matrix @ tracker_home_rot_inv
            )

            # Target rotation: R_target = ΔR_global * R_ee_home (pre-multiply for global frame)
            target_rotation = delta_rotation_global @ self.ee_pose_home.rotation_matrix
            return Transform3D(
                position=target_position, rotation_matrix=target_rotation
            )

        except ImportError:
            logger.error("ts_spatial not available for pose computation")
            raise

    # ----------------------
    # Lifecycle hooks
    # ----------------------
    def on_start(self) -> None:
        """Called when START command is received.

        Captures tracker reference pose at current position (home) for relative control.
        The robot is already at home from setup_home_configuration().
        """
        super().on_start()

        # Capture reference pose at current position (should be home)
        if not self._capture_reference_pose():
            logger.warning("Failed to capture reference pose on START")

    def get_safe_position(self) -> np.ndarray | None:
        """Get safe position (home configuration).

        Returns:
            Home joint configuration, or None if not set
        """
        return self.q_home
