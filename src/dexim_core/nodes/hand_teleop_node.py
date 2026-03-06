"""HandTeleopNode - Base class for hand teleoperation using skeleton data.

This module provides the HandTeleopNode abstract base class for hand robots
that use skeleton data for teleoperation with optimization-based control:

- Skeleton data processing with feature extraction
- Vector optimization for joint angle computation
- No reference pose capture needed (uses finger vectors directly)
- Landscape-based sensor waiting with graceful degradation

Hand robots (DH5, Inspire, etc.) extend this class and implement robot-specific
feature extraction and optimization.

Example:
    from dexim_core.nodes import HandTeleopNode

    class DH5ControlNode(HandTeleopNode):
        def setup(self):
            self.subscriber = ManusSubscriber(...)
            self.model = DH5Model(...)
            self.optimizer = VectorOptimizer(self.model)
            self.interface = DH5Interface(...)

        def process_data(self, data):
            skeleton = self._get_skeleton_for_handedness(data)
            vectors = self._extract_features(skeleton)
            return self.optimizer.retarget(vectors)

    with DH5ControlNode("dh5_left", config) as node:
        node.run()
"""

from __future__ import annotations

from abc import abstractmethod
from typing import Any

import numpy as np
from loguru import logger

from dexim_core.nodes.protocols import (
    DEFAULT_SENSOR_WAIT_CONFIG,
    ControlNodeConfig,
    SensorWaitConfig,
    VectorOptimizerProtocol,
)
from dexim_core.nodes.teleop_node import TeleopNode


class HandTeleopNode(TeleopNode):
    """Base class for hand teleoperation using skeleton data.

    This class extends TeleopNode with hand-specific features:
    - Skeleton data parsing and handedness selection
    - Feature extraction from skeleton (finger vectors)
    - Vector optimization for joint angles
    - No reference pose capture needed
    - Landscape-based sensor waiting with configurable timeout

    Subclasses must implement:
    - setup(): Initialize subscriber, model, optimizer, interface
    - process_data(): Extract features and optimize
    - get_safe_position(): Return open hand position (zeros)
    - _get_feature_config(): Return feature extraction configuration

    Attributes:
        optimizer: Vector optimizer for retargeting (VectorOptimizerProtocol)
        feature_config: Feature extraction configuration dict
        handedness: Hand side ("left" or "right")
        model: Hand kinematic model
        sensor_wait_config: Configuration for sensor waiting behavior
    """

    def __init__(self, node_id: str, config: ControlNodeConfig):
        """Initialize hand teleop node.

        Args:
            node_id: Unique identifier (e.g., "dh5_left", "inspire_right")
            config: Control node configuration
        """
        super().__init__(node_id=node_id, config=config)

        # Hand-specific components (set by setup)
        self.optimizer: VectorOptimizerProtocol
        self.feature_config: dict[str, Any] = {}
        self.model: Any

        # Handedness (set by subclass based on config)
        self.handedness: str = "left"

        # Expected glove_id for skeleton matching (set by subclass)
        # Can be int (0, 1) or str (hex ID like "45FC3255")
        # If None, will use default convention: left=0, right=1
        self.expected_glove_id: str | int | None = None

        # Sensor wait configuration (can be overridden by subclass or config)
        self.sensor_wait_config: SensorWaitConfig = DEFAULT_SENSOR_WAIT_CONFIG

        # Track if skeleton warning has been logged (to avoid spamming)
        self._skeleton_warning_logged: bool = False

    @property
    def active_dofs(self) -> int:
        """Get number of active DOFs for this hand.

        Returns:
            Number of active degrees of freedom

        Note:
            Subclasses may override to provide config-based or model-based value.
        """
        # Try to derive from model
        try:
            if hasattr(self, "model"):
                if hasattr(self.model, "n_active_joints"):
                    return int(self.model.n_active_joints)
                if hasattr(self.model, "active_joint_indices"):
                    return len(self.model.active_joint_indices)
                if hasattr(self.model, "nq"):
                    return int(self.model.nq)
        except Exception:
            logger.debug("Failed to derive active DOFs from model")

        # Default fallback
        return 6

    # ----------------------
    # Abstract methods
    # ----------------------
    @abstractmethod
    def _get_feature_config(self) -> dict[str, Any]:
        """Get feature extraction configuration.

        Returns:
            Dict with keys:
            - src_indices: Source joint indices for feature extraction
            - dst_indices: Destination joint indices
            - apply_rotation: Whether to apply rotation during extraction

        Example:
            return {
                "src_indices": [0, 1, 2, 3, 4],
                "dst_indices": [5, 6, 7, 8, 9],
                "apply_rotation": True,
            }
        """
        pass

    # ----------------------
    # Skeleton initialization
    # ----------------------
    def setup_home_configuration(self) -> None:
        """Set up home configuration for hand teleoperation.

        This method is called automatically after setup() by TeleopNode._post_setup().
        It:
        1. Waits for skeleton to appear in landscape (graceful degradation, optional)
        2. Moves to safe (open hand) position

        Subclasses may override to add hand-specific logic.

        Raises:
            RuntimeError: If movement to home position fails
        """
        logger.info(f"Setting up home configuration for {self.node_id}...")

        # Step 1: Wait for skeleton to appear in landscape (optional)
        if self.sensor_wait_config.skip_on_setup:
            logger.info(
                "Skipping skeleton wait during setup (skip_on_setup=True). "
                "Skeleton will be used once available during control loop."
            )
        else:
            skeleton_ready = self.wait_for_skeleton_ready()
            if not skeleton_ready:
                if self.sensor_wait_config.graceful_degradation:
                    logger.warning(
                        f"Skeleton for '{self.handedness}' hand not found after "
                        f"{self.sensor_wait_config.timeout_sec}s. "
                        "Continuing with graceful degradation."
                    )
                else:
                    logger.error(
                        f"Skeleton for '{self.handedness}' hand not found after "
                        f"{self.sensor_wait_config.timeout_sec}s timeout."
                    )

        # Step 2: Move to safe (open hand) position
        logger.info("Moving to home (open hand) configuration...")
        safe_position = self.get_safe_position()

        success = self._move_to_position_safely(
            safe_position,
            timeout_sec=5.0,
            max_velocity_rad_s=1.0,  # Gentle for initial homing
        )

        if not success:
            raise RuntimeError(
                "Failed to move to home configuration: movement timed out"
            )

        logger.success(f"Home configuration setup complete for {self.node_id}")

    def wait_for_skeleton_ready(self) -> bool:
        """Wait for skeleton to appear in the subscriber's landscape.

        This method can be called during setup to ensure skeleton data is available
        before starting the control loop. It implements graceful degradation by
        logging warnings if skeleton is not found within the timeout.

        Uses the sensor_wait_config for timeout and polling settings.

        Returns:
            True if skeleton found, False if timeout
        """
        # Check if subscriber supports wait_for_sensor
        if hasattr(self.subscriber, "wait_for_sensor"):
            # For skeletons, we need to pass glove_id since ManusSkeletonData
            # doesn't have a handedness attribute - it only has glove_id
            result = self.subscriber.wait_for_sensor(
                sensor_type="skeleton",
                handedness=self.handedness,
                glove_id=self.expected_glove_id,  # Pass glove_id for skeleton matching
                timeout_sec=self.sensor_wait_config.timeout_sec,
                poll_interval_sec=self.sensor_wait_config.poll_interval_sec,
                require_stable_frames=self.sensor_wait_config.require_stable_frames,
            )
            if not result:
                if self.sensor_wait_config.graceful_degradation:
                    logger.warning(
                        f"Skeleton for '{self.handedness}' hand not found after "
                        f"{self.sensor_wait_config.timeout_sec}s. "
                        "Hand control will start when skeleton becomes available."
                    )
                else:
                    logger.error(
                        f"Skeleton for '{self.handedness}' hand not found after "
                        f"{self.sensor_wait_config.timeout_sec}s timeout."
                    )
            return result
        else:
            # Fallback: subscriber doesn't support landscape-based waiting
            logger.debug("Subscriber doesn't support wait_for_sensor, assuming ready")
            return True

    # ----------------------
    # Skeleton processing
    # ----------------------
    def _get_skeleton_by_glove_id(self, data: dict[str, Any]) -> Any | None:
        """Get skeleton data matching this hand's glove_id.

        Args:
            data: Data from subscriber.read() with "skeletons" dict

        Returns:
            ManusSkeletonData for matching glove_id, or None if no match found
        """
        skeletons = data.get("skeletons", {})

        if not skeletons:
            if not self._skeleton_warning_logged:
                logger.warning(
                    f"No skeletons found in data for glove_id {self.expected_glove_id} "
                    f"({self.handedness} hand). Waiting for skeleton data..."
                )
                self._skeleton_warning_logged = True
            return None

        self._skeleton_warning_logged = False

        if self.expected_glove_id is None:
            logger.warning(
                f"[{self.handedness}] No glove_id configured. "
                "Please set glove_id in config file."
            )
            return None

        # Convert glove_id to integer (supports hex string, decimal string, or int)
        try:
            if isinstance(self.expected_glove_id, str):
                try:
                    expected_glove_id_int = int(self.expected_glove_id, 16)
                except ValueError:
                    expected_glove_id_int = int(self.expected_glove_id, 10)
            else:
                expected_glove_id_int = int(self.expected_glove_id)
        except (ValueError, TypeError) as e:
            logger.error(
                f"[{self.handedness}] Invalid glove_id format ",
                f"'{self.expected_glove_id}': {e}",
            )
            return None

        # Find skeleton by glove_id
        for skeleton in skeletons.values():
            if (
                hasattr(skeleton, "glove_id")
                and skeleton.glove_id == expected_glove_id_int
            ):
                return skeleton

        logger.warning(
            f"[{self.handedness}] No skeleton matched glove_id "
            f"0x{expected_glove_id_int:X} ({expected_glove_id_int})"
        )
        return None

    def _extract_features(self, skeleton: Any) -> np.ndarray | None:
        """Extract feature vectors from skeleton data.

        Args:
            skeleton: ManusSkeletonData object

        Returns:
            Feature vectors array (num_fingers, 3), or None if extraction fails

        Note:
            Uses ManusFeatureExtractor.extract_position_vectors() with
            configuration from _get_feature_config().
        """
        if skeleton is None:
            return None

        try:
            from manus_subscriber import ManusFeatureExtractor

            config = self._get_feature_config()

            vectors = ManusFeatureExtractor.extract_position_vectors(
                skeleton,
                src_indices=config.get("src_indices"),
                dst_indices=config.get("dst_indices"),
                apply_rotation=config.get("apply_rotation", True),
            )

            # Convert to numpy array
            return ManusFeatureExtractor.vectors_to_array(vectors)

        except ImportError:
            logger.error("manus_subscriber not available for feature extraction")
            return None
        except Exception as e:
            logger.error(f"Feature extraction failed: {e}")
            return None

    def _scale_features(self, vectors: np.ndarray) -> np.ndarray:
        """Scale feature vectors using optimizer alpha.

        Args:
            vectors: Raw feature vectors (num_fingers, 3)

        Returns:
            Scaled feature vectors
        """
        if hasattr(self.optimizer, "alpha"):
            alpha = np.array(self.optimizer.alpha)
            return vectors * alpha[:, np.newaxis]
        return vectors

    # ----------------------
    # Default implementations
    # ----------------------
    def get_safe_position(self) -> np.ndarray:
        """Get safe position for hand (open hand).

        Returns:
            Joint angles for open hand (zeros)
        """
        return np.zeros(self.active_dofs)

    def on_start(self) -> None:
        """Called when START command is received.

        Hand nodes don't need reference pose capture.
        """
        super().on_start()
        # No reference pose capture needed for hands

    # ----------------------
    # Lifecycle hooks
    # ----------------------
    def on_pause(self) -> None:
        """Called when PAUSE command is received.

        Hand holds current position.
        """
        logger.info(f"{self.node_id} received PAUSE command - holding position")

    def on_stop(self) -> None:
        """Called when STOP command is received.

        Hand goes to safe (open) position.
        """
        logger.info(f"{self.node_id} received STOP command - going to safe position")
        try:
            self.go_to_safe_position()
        except Exception as e:
            logger.error(f"Error moving to safe position on stop: {e}")
