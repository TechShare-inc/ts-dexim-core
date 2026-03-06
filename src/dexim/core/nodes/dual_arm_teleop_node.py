"""DualArmTeleopNode - Base class for dual-arm teleoperation using tracker data.

This module provides the DualArmTeleopNode abstract base class for dual-arm robots
(like Unitree G1 humanoid) that use two trackers for bimanual teleoperation:

- Dual tracker reference pose capture on START command (left and right)
- Relative pose computation for both arms
- Dual-arm IK solving
- Home configuration setup (moves robot to home position)

Note: Tracker data is NOT required during initialization. The node will:
1. During setup: Move to home position without tracker dependency
2. On START command: Move to home, then capture tracker reference poses

Example:
    from dexim.core.nodes import DualArmTeleopNode

    class G1ControlNode(DualArmTeleopNode):
        def setup(self):
            self.subscriber = ManusSubscriber(...)
            self.model = G1Model(...)
            self.interface = G1Interface(...)

        def process_data(self, data):
            # Get relative poses and solve dual-arm IK
            left_target = self.compute_target_pose_left(left_tracker)
            right_target = self.compute_target_pose_right(right_tracker)
            return self.model.retarget(left=left_target, right=right_target)

    # setup_home_configuration() is called automatically after setup()
    with G1ControlNode("g1_sim", config) as node:
        node.run()
"""

from __future__ import annotations

import time
from abc import abstractmethod
from collections.abc import Sequence

import numpy as np
from loguru import logger
from dexim.core.spatial import Transform3D

from dexim.core.nodes.protocols import (
    DEFAULT_SENSOR_WAIT_CONFIG,
    ControlNodeConfig,
    DualArmRobotModelProtocol,
    SensorWaitConfig,
    TrackerDataProtocol,
)
from dexim.core.nodes.teleop_node import TeleopNode


class DualArmTeleopNode(TeleopNode):
    """Base class for dual-arm teleoperation using tracker data.

    This class extends TeleopNode with dual-arm-specific features:
    - Dual tracker reference pose capture (on START command)
    - Relative pose computation for both arms
    - Home configuration setup (moves robot to home position)
    - Dual-arm IK-based control pipeline

    Subclasses must implement:
    - setup(): Initialize subscriber, model, interface
    - process_data(): Compute dual-arm IK from tracker data
    - get_safe_position(): Return home joint configuration
    - _get_home_joints(): Get home joints from config
    - _compute_ee_pose_left/right(): FK for each arm

    Attributes:
        tracker_pose_home_left: Reference tracker pose for left arm (captured on START)
        tracker_pose_home_right: Reference tracker pose for right arm (captured on START)
        ee_pose_home_left: End-effector pose at home for left arm
        ee_pose_home_right: End-effector pose at home for right arm
        q_home: Home joint configuration (both arms)
        q_previous: Previous joint configuration (for IK warm start)
        model: Dual-arm robot kinematic model
    """

    def __init__(self, node_id: str, config: ControlNodeConfig):
        """Initialize dual-arm teleop node.

        Args:
            node_id: Unique identifier (e.g., "g1_sim", "g1_real")
            config: Control node configuration
        """
        super().__init__(node_id=node_id, config=config)

        # Home poses for both arms
        self.tracker_pose_home_left: Transform3D | None = None
        self.tracker_pose_home_right: Transform3D | None = None
        self.ee_pose_home_left: Transform3D | None = None
        self.ee_pose_home_right: Transform3D | None = None

        # Joint configurations
        self.q_home: np.ndarray | None = None
        self.q_previous: np.ndarray | None = None

        # Robot model (set by setup)
        self.model: DualArmRobotModelProtocol

        # Sensor wait configuration (can be overridden by subclass or config)
        self.sensor_wait_config: SensorWaitConfig = DEFAULT_SENSOR_WAIT_CONFIG

    # ----------------------
    # Home configuration
    # ----------------------
    def setup_home_configuration(self) -> None:
        """Set up home configuration for dual-arm teleoperation.

        This method is called automatically after setup() by TeleopNode._post_setup().
        It:
        1. Sets q_home from config or neutral
        2. Computes end-effector home poses for both arms via FK
        3. Moves smoothly to home configuration

        Note: Tracker reference poses are captured later in on_start() when the user
        presses START, not during initial setup. This allows the node to initialize
        without requiring tracker data to be available.

        Subclasses may override to add robot-specific logic.

        Raises:
            RuntimeError: If smooth movement to home position fails
        """
        logger.info("Setting up home configuration for dual arms...")

        # Step 1: Set q_home from config or neutral
        self.q_home = self._get_home_joints()
        logger.info("Using home joint configuration")

        # Step 2: Compute end-effector home poses for both arms via FK
        self.ee_pose_home_left = self._compute_ee_pose_left(self.q_home)
        self.ee_pose_home_right = self._compute_ee_pose_right(self.q_home)

        logger.info(f"Left EE home position: {self.ee_pose_home_left.position}")
        logger.info(f"Right EE home position: {self.ee_pose_home_right.position}")

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

        logger.success("Home configuration setup complete for dual arms")

    def _find_tracker_by_type(
        self, tracker_list: Sequence[TrackerDataProtocol], tracker_type: str
    ) -> TrackerDataProtocol | None:
        """Find tracker by type from list of TrackerData objects.

        Args:
            tracker_list: List of TrackerData objects
            tracker_type: Type to find (e.g., "left_hand", "right_hand")

        Returns:
            Matching TrackerData or None
        """
        for tracker in tracker_list:
            if tracker.tracker_type == tracker_type:
                return tracker
        return None

    @abstractmethod
    def _get_home_joints(self) -> np.ndarray:
        """Get home joint configuration from config or neutral.

        Returns:
            Home joint configuration array for both arms
        """
        pass

    @abstractmethod
    def _compute_ee_pose_left(self, q: np.ndarray) -> Transform3D:
        """Compute left end-effector pose via forward kinematics.

        Args:
            q: Joint configuration

        Returns:
            Left end-effector Transform3D
        """
        pass

    @abstractmethod
    def _compute_ee_pose_right(self, q: np.ndarray) -> Transform3D:
        """Compute right end-effector pose via forward kinematics.

        Args:
            q: Joint configuration

        Returns:
            Right end-effector Transform3D
        """
        pass

    def _extract_model_config_from_interface(
        self, interface_state_q: np.ndarray
    ) -> np.ndarray:
        """Extract model-relevant DOF from interface state.

        When the interface has more DOF than the model (e.g., 29 DOF robot state
        but 14 DOF arm model), extract only the arm joint positions.

        Args:
            interface_state_q: Full joint state from interface

        Returns:
            Joint positions for the model (typically arm-only DOF)
        """
        # If interface has a mapping function, use it
        if hasattr(self.interface, "_map_interface_to_model_config"):
            mapped = self.interface._map_interface_to_model_config(interface_state_q)
            if mapped is not None:
                return mapped

        # Fallback: assume last N DOF are arm DOF (won't be used for G1 due to mapping above)
        model_nq = len(self.q_home)
        if len(interface_state_q) >= model_nq:
            return interface_state_q[-model_nq:]

        return interface_state_q

    def _move_to_position_safely(
        self,
        target_joints: np.ndarray,
        timeout_sec: float = 5.0,
        max_velocity_rad_s: float | None = None,
    ) -> bool:
        """Move to target position with velocity limiting.

        Overrides parent to handle DOF conversion when interface has more DOF than model.

        Args:
            target_joints: Target joint configuration (model DOF)
            timeout_sec: Maximum time to spend on movement
            max_velocity_rad_s: Optional velocity limit (rad/s)

        Returns:
            True if reached target, False if timed out
        """
        try:
            # Read current position
            current_state = self.interface.read()
            # Convert from interface DOF to model DOF
            current_joints = self._extract_model_config_from_interface(current_state.q)
        except Exception as e:
            logger.warning(
                f"Cannot read current position for safe movement: {e}. "
                "Attempting direct movement."
            )
            self._send_command(target_joints)
            return True

        # Calculate distance and required velocity
        delta = target_joints - current_joints
        max_joint_delta = np.max(np.abs(delta))

        # If already at target (within small tolerance), done
        if max_joint_delta < 1e-4:
            logger.debug("Already at target position")
            return True

        # Determine velocity limit with priority hierarchy
        if max_velocity_rad_s is not None:
            velocity_limit = max_velocity_rad_s
        elif self.config.control.safe_position_max_velocity_rad_s is not None:
            velocity_limit = self.config.control.safe_position_max_velocity_rad_s
        else:
            velocity_limit = self.config.control.max_joint_velocity_rad_s

        # Calculate number of steps needed based on max velocity
        max_delta_per_step = velocity_limit * self.dt
        num_steps = int(np.ceil(max_joint_delta / max_delta_per_step))

        # Add safety margin and cap at timeout
        num_steps = min(num_steps + 5, int(timeout_sec / self.dt))

        logger.info(
            f"Moving to safe position: {num_steps} steps "
            f"(~{num_steps * self.dt:.2f}s, max delta: {max_joint_delta:.3f} rad)"
        )

        start_time = time.time()
        for step in range(num_steps):
            # Check timeout
            if (time.time() - start_time) > timeout_sec:
                logger.warning(
                    f"Movement timeout: reached step {step}/{num_steps}, "
                    f"delta still: {max_joint_delta:.3f} rad"
                )
                return False

            # Interpolate towards target
            progress = (step + 1) / num_steps
            interpolated = current_joints + (delta * progress)

            # Send command
            self._send_command(interpolated)

            # Read actual position
            try:
                current_state = self.interface.read()
                current_joints = self._extract_model_config_from_interface(current_state.q)
                delta = target_joints - current_joints
                max_joint_delta = np.max(np.abs(delta))
            except Exception as e:
                logger.warning(f"Cannot read current position during movement: {e}")

            # Small sleep to avoid busy-waiting
            time.sleep(self.dt * 0.9)

        logger.success("Successfully reached safe position")
        return True

    # ----------------------
    # Reference pose capture
    # ----------------------
    def _capture_reference_pose(self, max_attempts: int = 20) -> bool:
        """Capture current tracker poses as reference for relative control.

        Called by on_start() to capture reference poses for both arms. Retries
        a few times to ensure tracker data is available and fresh (not stale buffered data).

        Args:
            max_attempts: Maximum number of read attempts (default: 20)

        Returns:
            True if successful, False otherwise
        """
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

                left_tracker = self._find_tracker_by_type(trackers, "left_hand")
                right_tracker = self._find_tracker_by_type(trackers, "right_hand")

                if left_tracker is None or right_tracker is None:
                    logger.debug(
                        f"Attempt {attempt + 1}/{max_attempts}: "
                        f"left_hand={'found' if left_tracker else 'MISSING'}, "
                        f"right_hand={'found' if right_tracker else 'MISSING'}, waiting..."
                    )
                    time.sleep(0.05)
                    continue

                # Check for stale data (buffered from before pause)
                # Check both trackers - use the older timestamp for validation
                left_age = float('inf')
                right_age = float('inf')
                if hasattr(left_tracker, "timestamp") and left_tracker.timestamp > 0:
                    left_age = current_time - left_tracker.timestamp
                if hasattr(right_tracker, "timestamp") and right_tracker.timestamp > 0:
                    right_age = current_time - right_tracker.timestamp

                max_age = max(left_age, right_age)
                if max_age < float('inf') and max_age > STALE_THRESHOLD_SEC:
                    logger.debug(
                        f"Attempt {attempt + 1}/{max_attempts}: Skipping stale data "
                        f"(left: {left_age:.3f}s, right: {right_age:.3f}s)"
                    )
                    # Don't sleep - consume queue quickly
                    continue

                # Update home poses
                self.tracker_pose_home_left = left_tracker.transform
                self.tracker_pose_home_right = right_tracker.transform

                # Update EE home poses via FK at current position
                try:
                    current_state = self.interface.read()
                    q_home = current_state.q.copy()
                    self.q_home = q_home
                    self.ee_pose_home_left = self._compute_ee_pose_left(q_home)
                    self.ee_pose_home_right = self._compute_ee_pose_right(q_home)
                    self.q_previous = q_home.copy()
                except Exception as e:
                    logger.warning(f"Could not read current state: {e}")

                logger.info(
                    f"Dual reference poses captured: "
                    f"left={self.tracker_pose_home_left.position}, "
                    f"right={self.tracker_pose_home_right.position}"
                )
                return True

            except Exception as e:
                logger.error(
                    f"Error during reference capture attempt {attempt + 1}: {e}"
                )
                time.sleep(0.1)

        logger.warning(
            f"Failed to capture dual reference poses after {max_attempts} attempts"
        )
        return False

    # ----------------------
    # Relative pose computation
    # ----------------------
    def compute_target_pose_left(
        self, current_tracker_pose: Transform3D
    ) -> Transform3D:
        """Compute target end-effector pose for left arm.

        Uses relative motion from reference (GLOBAL FRAME approach):
        - Δp = p_tracker - p_tracker_home_left
        - p_target = p_ee_home_left + Δp
        - ΔR_global = R_tracker * R_tracker_home_left⁻¹ (delta in global frame)
        - R_target = ΔR_global * R_ee_home_left (pre-multiply for global rotation)

        Args:
            current_tracker_pose: Current left tracker Transform3D

        Returns:
            Target left end-effector Transform3D
        """
        return self._compute_relative_target(
            current_tracker_pose,
            self.tracker_pose_home_left,
            self.ee_pose_home_left,
        )

    def compute_target_pose_right(
        self, current_tracker_pose: Transform3D
    ) -> Transform3D:
        """Compute target end-effector pose for right arm.

        Uses relative motion from reference.

        Args:
            current_tracker_pose: Current right tracker Transform3D

        Returns:
            Target right end-effector Transform3D
        """
        return self._compute_relative_target(
            current_tracker_pose,
            self.tracker_pose_home_right,
            self.ee_pose_home_right,
        )

    def _compute_relative_target(
        self,
        current_tracker: Transform3D,
        tracker_home: Transform3D | None,
        ee_home: Transform3D | None,
    ) -> Transform3D:
        """Compute target pose using relative motion from reference.

        Args:
            current_tracker: Current tracker Transform3D
            tracker_home: Reference tracker Transform3D
            ee_home: Reference end-effector Transform3D

        Returns:
            Target end-effector Transform3D
        """
        if tracker_home is None or ee_home is None:
            raise RuntimeError(
                "Home configuration not set. Call setup_home_configuration() first."
            )

        try:
            from dexim.core.spatial import Transform3D

            # Relative position: Δp = p_tracker - p_tracker_home
            delta_position = current_tracker.position - tracker_home.position

            # Target position: p_target = p_ee_home + Δp
            target_position = ee_home.position + delta_position

            # Relative rotation in GLOBAL frame:
            # ΔR_global = R_tracker_current * R_tracker_home^(-1)
            # R_target = ΔR_global * R_ee_home
            #          = R_tracker_current * R_tracker_home^(-1) * R_ee_home
            #
            # This computes the rotation change in the world/global frame,
            # then applies that global rotation to the EE home orientation.
            tracker_home_rot_inv = tracker_home.inverse().rotation_matrix
            delta_rotation_global = (
                current_tracker.rotation_matrix @ tracker_home_rot_inv
            )

            # Target rotation: R_target = ΔR_global * R_ee_home (pre-multiply for global frame)
            target_rotation = delta_rotation_global @ ee_home.rotation_matrix

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

        Captures tracker reference poses at current position (home) for both arms.
        The robot is already at home from setup_home_configuration().
        """
        super().on_start()

        # Capture reference poses at current position (should be home)
        if not self._capture_reference_pose():
            logger.warning("Failed to capture reference poses on START")

    def get_safe_position(self) -> np.ndarray | None:
        """Get safe position (home configuration).

        Returns:
            Home joint configuration, or None if not set
        """
        return self.q_home
