"""TeleopNode - Abstract base class for robot teleoperation control nodes.

This module provides the TeleopNode abstract base class for building
robot teleoperation control nodes with:
- Precise control rate with drift correction
- Data timeout monitoring
- Velocity limiting for safe motion
- Safe position fallback with smooth interpolation
- Graceful shutdown handling
- Performance statistics

TeleopNode is the base class for both ArmTeleopNode and HandTeleopNode,
providing the common control loop infrastructure.

Example:
    from dexim.core.nodes import TeleopNode

    class MyRobotNode(TeleopNode):
        def setup(self):
            self.subscriber = ManusSubscriber(...)
            self.interface = MyRobotInterface(...)

        def process_data(self, data):
            # Process sensor data, return joint positions
            return np.array([...])

        def get_safe_position(self):
            return np.zeros(6)  # Home position

    config = ControlNodeConfig.from_yaml("config.yaml")
    with MyRobotNode("my_robot", config) as node:
        node.run()
"""

from __future__ import annotations

import signal
import time
from abc import abstractmethod
from typing import Any

import numpy as np
from loguru import logger
from dexim.core.messages import TopicBuilder

from dexim.core.nodes.hardware_publisher import HardwarePublisherNode
from dexim.core.nodes.protocols import (
    ControlNodeConfig,
    DataSubscriber,
    RobotInterfaceProtocol,
)
from dexim.core.nodes.utils import RateLimiter, smootherstep


class TeleopNode(HardwarePublisherNode):
    """Abstract base class for robot teleoperation control nodes.

    This class provides the core control loop infrastructure with:
    - Precise control rate with drift correction (configurable, default 30Hz)
    - Data timeout monitoring
    - Velocity limiting for safe motion
    - Safe position fallback with smooth interpolation
    - Graceful shutdown handling
    - Performance statistics (rate, jitter, overtimes)
    - ZMQ-based orchestration (inherits from HardwarePublisherNode)
    - Data Plane publishing for actions and observations

    Subclasses must implement:
    - setup(): Initialize robot-specific components
    - process_data(): Convert sensor data to robot commands
    - get_safe_position(): Define safe fallback position

    Inheritance Hierarchy:
    - ArmTeleopNode: For arm robots using tracker data (IK-based)
    - DualArmTeleopNode: For dual-arm robots (e.g., G1 humanoid)
    - HandTeleopNode: For hand robots using skeleton data (optimizer-based)
    """

    def __init__(self, node_id: str, config: ControlNodeConfig):
        """Initialize teleop node with configuration.

        Args:
            node_id: Unique identifier for this node (e.g., "nova_left", "dh5_right")
            config: Complete control node configuration
        """
        # Get publishing configuration from config (with defaults)
        data_endpoint = getattr(config, "data_endpoint", "tcp://*:5556")
        bind_data = getattr(config, "bind_data", True)
        observation_rate = getattr(config, "observation_rate_hz", None)
        # Use control rate if observation_rate not specified
        if observation_rate is None:
            observation_rate = config.control.rate_hz

        # Initialize HardwarePublisherNode for Data Plane publishing
        super().__init__(
            node_id=node_id,
            data_endpoint=data_endpoint,
            bind=bind_data,
            heartbeat_interval=1.0,
            rate_hz=observation_rate,
            control_endpoint=getattr(config, "control_endpoint", None),
            status_endpoint=getattr(config, "status_endpoint", None),
        )

        # Robot-specific attributes (will be set by setup())
        self.subscriber: DataSubscriber
        self.interface: RobotInterfaceProtocol

        self.config = config
        self.last_data_time = 0.0

        # Calculate control period from rate
        self.dt = 1.0 / config.control.rate_hz

        # Create rate limiter for precise timing
        self.rate_limiter = RateLimiter(rate_hz=config.control.rate_hz)

        # Velocity limiting state
        self.current_joint_positions: np.ndarray | None = None
        self.velocity_limiting_enabled = config.control.enable_velocity_limiting

        # Control loop initialization flag
        self._control_loop_initialized = False

        # Generate topics for actions and observations
        builder = TopicBuilder()
        self._action_topic = builder.action.joint_cmd(self.node_id)
        self._state_topic = builder.observation.joint_state(self.node_id)

        logger.info(f"Initialized {self.__class__.__name__}")
        logger.info(f"  Node ID: {node_id}")
        logger.info(f"  Robot type: {config.robot_type}")
        logger.info(f"  Interface: {config.interface.mode}")
        logger.info(
            f"  Control rate: {config.control.rate_hz}Hz ({self.dt * 1000:.2f}ms period)"
        )
        logger.info(
            f"  Velocity limiting: {'enabled' if self.velocity_limiting_enabled else 'disabled'} "
            f"(max: {config.control.max_joint_velocity_rad_s} rad/s)"
        )
        logger.info(f"  Data endpoint: {data_endpoint}")
        logger.info(f"  Action topic: {self._action_topic}")
        logger.info(f"  State topic: {self._state_topic}")

    # ----------------------
    # Post-setup hook
    # ----------------------
    def _post_setup(self) -> None:
        """Called after setup() completes to perform additional initialization.

        This method automatically calls setup_home_configuration() if available,
        ensuring the robot moves to its home position at the end of initialization.

        Subclasses can override to add additional post-setup logic.
        """
        setup_home = getattr(self, "setup_home_configuration", None)
        if callable(setup_home):
            logger.info("Running post-setup: moving to home configuration...")
            setup_home()

    # ----------------------
    # Abstract methods
    # ----------------------
    @abstractmethod
    def setup(self) -> None:
        """Initialize robot-specific components.

        This method should:
        1. Create subscriber (e.g., ManusSubscriber)
        2. Create robot model (e.g., DH5Model or NovaModel)
        3. Create optimizer/IK solver (if needed)
        4. Create interface (sim or real)

        Example:
            self.subscriber = ManusSubscriber(...)
            self.model = DH5Model(...)
            self.interface = DH5SimInterface(...)
        """
        pass

    @abstractmethod
    def process_data(self, data: dict[str, Any]) -> np.ndarray | None:
        """Process sensor data and generate joint configurations.

        This method implements the data processing pipeline:
        1. Parse raw data from subscriber
        2. Extract features
        3. Compute joint angles (IK or optimization)

        Args:
            data: Data dict from subscriber.read()
                For hands: {"skeletons": {...}, ...}
                For arms: {"trackers": [...], ...}

        Returns:
            Joint positions array (np.ndarray), or None if processing failed
        """
        pass

    @abstractmethod
    def get_safe_position(self) -> np.ndarray | None:
        """Get safe fallback position for timeout/error situations.

        Returns:
            Safe position command (robot-specific), or None if not applicable

        Example (Hand):
            return np.zeros(6)  # Open hand

        Example (Arm):
            return self.q_home  # Home position
        """
        pass

    # ----------------------
    # Control loop
    # ----------------------
    def _initialize_control_loop(self) -> None:
        """One-time initialization before control loop starts.

        This method is called once at the beginning of the control loop to:
        - Set up signal handlers for graceful shutdown
        - Initialize velocity limiting with current joint positions
        """
        if self._control_loop_initialized:
            return

        logger.info("Initializing control loop...")

        # Setup signal handlers for graceful shutdown (backup to ZMQ control)
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        # Initialize current position for velocity limiting
        if self.velocity_limiting_enabled:
            try:
                initial_state = self.interface.read()
                self.current_joint_positions = initial_state.q
                logger.info(
                    "Initialized velocity limiting with current joint positions"
                )
            except Exception as e:
                logger.warning(
                    f"Could not read initial position: {e}. "
                    "Velocity limiting will initialize on first command."
                )

        self.last_data_time = time.time()
        self._control_loop_initialized = True
        logger.info("Control loop initialization complete")

    def _main_loop_iteration(self) -> None:
        """Execute one iteration of the robot control loop.

        This method is called repeatedly by ManagedNode.run() and implements:
        1. One-time initialization on first call
        2. Check if teleoperation is active (gated by _teleop_active flag)
        3. Get latest data from subscriber
        4. Process data to generate target command
        5. Apply velocity limiting for safe motion
        6. Send command to interface
        7. Monitor for timeout
        8. Maintain target control rate
        """
        # One-time initialization on first iteration
        if not self._control_loop_initialized:
            self._initialize_control_loop()

        # Gate control loop with _teleop_active flag
        # When not active, skip processing but maintain rate for responsive restart
        if not self._teleop_active:
            # Still drain subscriber queue to keep data fresh for quick resume
            # This prevents stale buffered data from being used when resuming
            try:
                self.subscriber.read()
            except Exception:
                pass  # Ignore errors during pause - just drain the queue
            # Maintain control rate for responsive command handling
            self.rate_limiter.sleep()
            return

        # Get latest data
        data = self._get_data()

        # Check for timeout
        if self._check_timeout():
            logger.warning(
                f"Data timeout ({self.config.control.timeout_sec}s), "
                "moving to safe position"
            )
            self.go_to_safe_position()
            # Reset timeout after going to safe position
            self.last_data_time = time.time()
        else:
            # Process data if available (non-empty dict)
            if data:
                try:
                    target_joints = self.process_data(data)

                    if target_joints is not None:
                        # Apply velocity limiting for safe motion
                        safe_joints = self._apply_velocity_limiting(target_joints)
                        self._send_command(safe_joints)
                        self.last_data_time = time.time()

                except Exception as e:
                    logger.error(f"Error processing data: {e}", exc_info=True)
                    # Skip this frame, continue running

        # Publish observation (joint state) if enabled
        if self.is_publishing:
            self._publish_observation()

        # Maintain control rate with rate limiter
        timing = self.rate_limiter.sleep()

        # Log overtime warnings
        if timing["overtime"]:
            logger.debug(
                f"Loop overtime: {timing['elapsed'] * 1000:.2f}ms "
                f"(target: {self.dt * 1000:.2f}ms)"
            )

        # Log statistics every 100 loops
        if self.rate_limiter.iterations % 100 == 0:
            stats = self.rate_limiter.get_statistics()
            logger.debug(
                f"Control rate: {stats['actual_rate']:.1f}Hz "
                f"(target: {self.config.control.rate_hz}Hz), "
                f"jitter: {stats['mean_jitter_ms']:.2f}ms, "
                f"overtimes: {stats['overtime_count']}"
            )

    def get_data(self) -> tuple[bytes, Any] | list[tuple[bytes, Any]] | None:
        """Required by HardwarePublisherNode, but not used.

        TeleopNode handles publishing directly in _main_loop_iteration().
        This method exists only to satisfy the abstract method requirement.

        Returns:
            None (publishing is handled directly in control loop)
        """
        return None

    def _get_data(self) -> dict[str, Any]:
        """Get latest data from subscriber.

        Uses subscriber.read() which provides:
        - Automatic message draining (receive_latest pattern)
        - Built-in parsing to structured format

        Returns:
            Dict with structure: {"skeletons": {}, "trackers": [], "raw": list}
            Or empty dict if no data available
        """
        try:
            data = self.subscriber.read()
            if data and (
                data.get("raw") or data.get("trackers") or data.get("skeletons")
            ):
                return data
            return {}
        except Exception as e:
            logger.error(f"Error reading data from subscriber: {e}")
            return {}

    def _check_timeout(self) -> bool:
        """Check if data timeout has occurred.

        Returns:
            True if timeout, False otherwise
        """
        if not self.config.control.safe_position_on_timeout:
            return False

        time_since_data = time.time() - self.last_data_time
        return time_since_data > self.config.control.timeout_sec

    # ----------------------
    # Velocity limiting
    # ----------------------
    def _apply_velocity_limiting(self, target_joints: np.ndarray) -> np.ndarray:
        """Apply velocity limits to target joint positions for safe motion.

        This method ensures that joint velocities do not exceed configured limits
        by clamping the change in position per control cycle.

        Args:
            target_joints: Desired joint positions from process_data()

        Returns:
            Velocity-limited joint positions safe to send to robot
        """
        # If velocity limiting is disabled, return target directly
        if not self.velocity_limiting_enabled:
            return target_joints

        # Read current position from interface
        try:
            current_state = self.interface.read()
            current_joints = current_state.q
        except Exception as e:
            logger.warning(f"Cannot read current position for velocity limiting: {e}")
            # Fallback: use cached position if available, otherwise use target
            if self.current_joint_positions is not None:
                current_joints = self.current_joint_positions
            else:
                logger.warning(
                    "No cached position available, skipping velocity limiting"
                )
                return target_joints

        # Compute desired change
        delta = target_joints - current_joints

        # Compute maximum allowed change per timestep
        max_velocity = self.config.control.max_joint_velocity_rad_s  # rad/s
        max_delta = max_velocity * self.dt  # rad per timestep

        # Clamp delta per joint
        clamped_delta = np.clip(delta, -max_delta, max_delta)

        # Compute safe command
        safe_joints = current_joints + clamped_delta

        # Cache for next iteration
        self.current_joint_positions = safe_joints.copy()

        # Log if significant clamping occurred (any joint > 110% of limit)
        if np.any(np.abs(delta) > max_delta * 1.1):
            max_violation = np.max(np.abs(delta) / max_delta)
            logger.debug(
                f"Velocity limiting active: max violation {max_violation:.2f}x limit"
            )

        return safe_joints

    # ----------------------
    # Safe position movement
    # ----------------------

    def _move_to_position_safely(
        self,
        target_joints: np.ndarray,
        timeout_sec: float = 5.0,
        max_velocity_rad_s: float | None = None,
    ) -> bool:
        """Move to target position with velocity limiting and smooth interpolation.

        This method ALWAYS applies velocity limiting and interpolation for safe
        movement, regardless of the config.control.enable_velocity_limiting setting.

        Args:
            target_joints: Target joint configuration
            timeout_sec: Maximum time to spend on movement (default: 5.0s)
            max_velocity_rad_s: Optional velocity limit (rad/s)

        Returns:
            True if reached target (within tolerance), False if timed out
        """
        try:
            # Read current position
            current_state = self.interface.read()
            current_joints = current_state.q
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

        # Interpolate and send commands
        start_time = time.time()
        for i in range(num_steps):
            # Check timeout
            if time.time() - start_time > timeout_sec:
                logger.warning(f"Safe position movement timed out after {timeout_sec}s")
                return False

            # Smootherstep interpolation
            t = (i + 1) / num_steps
            alpha = smootherstep(t)
            interpolated_joints = current_joints + alpha * delta

            # Send command
            self._send_command(interpolated_joints)

            # Sleep to maintain control rate (except last iteration)
            if i < num_steps - 1:
                self.rate_limiter.sleep()

        elapsed = time.time() - start_time
        logger.success(f"Reached safe position in {elapsed:.2f}s")
        return True

    def go_to_safe_position(self, max_velocity_rad_s: float | None = None) -> None:
        """Move robot to safe position with velocity limiting.

        Args:
            max_velocity_rad_s: Optional velocity limit override (rad/s)
        """
        try:
            safe_joints = self.get_safe_position()
            if safe_joints is None:
                logger.warning("get_safe_position() returned None")
                return

            success = self._move_to_position_safely(
                safe_joints, timeout_sec=5.0, max_velocity_rad_s=max_velocity_rad_s
            )

            if not success:
                logger.warning(
                    "Safe position movement timed out, "
                    "attempting direct movement as fallback"
                )
                self._send_command(safe_joints)

        except Exception as e:
            logger.error(f"Error moving to safe position: {e}", exc_info=True)

    def _send_command(self, joint_cfgs: np.ndarray) -> None:
        """Send command to robot interface and publish action.

        Args:
            joint_cfgs: Joint positions array from process_data()
        """
        # Publish action (command) to Data Plane before sending to robot
        if self.is_publishing:
            self._publish_action(joint_cfgs)

        # Send to robot interface
        try:
            # Import here to avoid circular dependency
            from dexim.core.robot_interface import JointCommand

            command = JointCommand(q=joint_cfgs, mode="position")
            self.interface.write(command)
        except ImportError:
            # Fallback for custom interfaces that don't use JointCommand
            logger.warning("JointCommand not available, sending raw array")
            self.interface.write(joint_cfgs)
        except Exception as e:
            logger.error(f"Error sending command: {e}")

    def _publish_action(self, joint_cfgs: np.ndarray) -> None:
        """Publish joint command action to Data Plane.

        Args:
            joint_cfgs: Joint positions array to publish
        """
        try:
            # Convert to float32 list for serialization
            if joint_cfgs.dtype != np.float32:
                joint_cfgs = joint_cfgs.astype(np.float32)

            # Use inherited _send() method from HardwarePublisherNode
            self._send(self._action_topic, joint_cfgs.tolist())
        except Exception as e:
            # Non-fatal: skip this action publication
            logger.debug(f"Failed to publish action: {e}")

    def _publish_observation(self) -> None:
        """Publish current joint state observation to Data Plane.

        Rate limiting is handled by HardwarePublisherNode's rate_hz.
        """
        try:
            # Read current state from interface
            state = self.interface.read()

            # Serialize JointState to dict
            state_dict = {
                "q": state.q.tolist() if hasattr(state.q, "tolist") else state.q,
                "qd": state.qd.tolist() if hasattr(state.qd, "tolist") else state.qd,
                "tau": (
                    state.tau.tolist() if hasattr(state.tau, "tolist") else state.tau
                ),
                "stamp": getattr(state, "stamp", time.time()),
            }

            # Use inherited _send() method from HardwarePublisherNode
            self._send(self._state_topic, state_dict)
        except Exception as e:
            # Non-fatal: skip this observation
            logger.debug(f"Failed to publish observation: {e}")

    def _signal_handler(self, signum, frame) -> None:
        """Handle shutdown signals (Ctrl+C, SIGTERM)."""
        logger.info(f"Received signal {signum}, shutting down...")
        self.running = False

    # ----------------------
    # ManagedNode lifecycle hooks
    # ----------------------
    def on_start(self) -> None:
        """Called when START command is received.

        Begins active teleoperation. The _teleop_active flag is set True
        by ManagedNode before calling this hook.

        Subclasses should override to capture reference pose for relative control.
        Call super().on_start() first.
        """
        # Call HardwarePublisherNode.on_start() to enable publishing
        super().on_start()
        logger.info(f"{self.node_id} received START command - teleoperation active")

    def on_pause(self) -> None:
        """Called when PAUSE command is received.

        Pauses teleoperation but holds current position for quick resume.
        The _teleop_active flag is set False by ManagedNode before calling.
        """
        # Call HardwarePublisherNode.on_pause() to disable publishing
        super().on_pause()
        logger.info(f"{self.node_id} received PAUSE command - holding position")

    def on_stop(self) -> None:
        """Called when STOP command is received.

        Stops teleoperation and moves robot to safe position.
        The _teleop_active flag is set False by ManagedNode before calling.
        """
        # Call HardwarePublisherNode.on_stop() to disable publishing
        super().on_stop()
        logger.info(f"{self.node_id} received STOP command - going to safe position")
        try:
            self.go_to_safe_position()
        except Exception as e:
            logger.error(f"Error moving to safe position on stop: {e}")

    def on_start_recording(self) -> None:
        """Called when START_REC command is received."""
        logger.info(f"{self.node_id} started recording")

    def on_stop_recording(self) -> None:
        """Called when STOP_REC command is received."""
        logger.info(f"{self.node_id} stopped recording")

    def on_shutdown(self) -> None:
        """Called during shutdown sequence."""
        logger.info(f"{self.node_id} received SHUTDOWN command - cleaning up robot")
        self._cleanup_robot()

    # ----------------------
    # Robot cleanup
    # ----------------------
    def _cleanup_robot(self) -> None:
        """Cleanup and shutdown robot resources."""
        logger.info("Cleaning up robot resources...")

        # Move to safe position if enabled
        if self.config.control.safe_position_on_timeout:
            try:
                self.go_to_safe_position()
            except Exception as e:
                logger.error(f"Error during safe position shutdown: {e}")

        # Disconnect interface
        try:
            self.interface.disconnect()
            logger.info("Interface disconnected")
        except Exception as e:
            logger.error(f"Error disconnecting interface: {e}")

        # Close subscriber
        try:
            self.subscriber.close()
            logger.info("Subscriber closed")
        except Exception as e:
            logger.error(f"Error closing subscriber: {e}")

        logger.success("Robot cleanup complete")

    def __enter__(self):
        """Context manager entry."""
        self.setup()
        self._post_setup()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self._cleanup_robot()
