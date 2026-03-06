"""Abstract base class for robot control nodes.

This module provides the RobotControlNode abstract base class for building
robot teleoperation control nodes with:
- Precise control rate with drift correction
- Data timeout monitoring
- Velocity limiting for safe motion
- Safe position fallback with smooth interpolation
- Graceful shutdown handling
- Performance statistics

Example:
    from dexim.core.nodes import RobotControlNode
    from dexim.core.config import ControlNodeConfig

    class MyRobotNode(RobotControlNode):
        def setup(self):
            self.subscriber = ManusSubscriber(...)
            self.interface = MyRobotInterface(...)

        def process_data(self, manus_data):
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
from abc import ABC, abstractmethod
from typing import Any, Optional, Protocol, runtime_checkable

import numpy as np
from loguru import logger

from dexim.core.nodes.managed import ManagedNode
from dexim.core.nodes.utils import RateLimiter, smootherstep


@runtime_checkable
class DataSubscriber(Protocol):
    """Protocol for data subscribers (e.g., ManusSubscriber)."""
from __future__ import annotations


    def read(self) -> dict[str, Any]:
        """Read latest data from the subscriber."""
from __future__ import annotations

        ...

    def close(self) -> None:
        """Close the subscriber connection."""
from __future__ import annotations

        ...


@runtime_checkable
class ControlNodeConfig(Protocol):
    """Protocol for control node configuration."""
from __future__ import annotations


    robot_type: str

    @property
    def control(self) -> Any:
        """Control configuration with rate_hz, timeout_sec, etc."""
from __future__ import annotations

        ...

    @property
    def interface(self) -> Any:
        """Interface configuration with mode."""
from __future__ import annotations

        ...


@runtime_checkable
class RobotInterfaceProtocol(Protocol):
    """Protocol for robot interfaces."""
from __future__ import annotations


    def read(self) -> Any:
        """Read current robot state."""
from __future__ import annotations

        ...

    def write(self, command: Any) -> None:
        """Write command to robot."""
from __future__ import annotations

        ...

    def disconnect(self) -> None:
        """Disconnect from robot."""
from __future__ import annotations

        ...


class RobotControlNode(ManagedNode, ABC):
    """Abstract base class for robot teleoperation control nodes.

    This class provides the core control loop infrastructure with:
    - Precise control rate with drift correction (configurable, default 30Hz)
    - Data timeout monitoring
    - Safe position fallback
    - Graceful shutdown handling
    - Performance statistics (rate, jitter, overtimes)
    - ZMQ-based orchestration (inherits from ManagedNode)

    Subclasses must implement:
    - setup(): Initialize robot-specific components
    - process_data(): Convert sensor data to robot commands
    - get_safe_position(): Define safe fallback position
    """
from __future__ import annotations


    def __init__(self, node_id: str, config: ControlNodeConfig):
        """Initialize control node with configuration.

        Args:
            node_id: Unique identifier for this node (e.g., "nova_left", "dh5_right")
            config: Complete control node configuration
        """
from __future__ import annotations

        # Initialize ManagedNode first for ZMQ orchestration
        super().__init__(
            node_id=node_id,
            heartbeat_interval=1.0,  # Send heartbeat every 1 second
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
        self.current_joint_positions = (
            None  # Track current position for velocity limiting
        )
        self.velocity_limiting_enabled = config.control.enable_velocity_limiting

        # Control loop initialization flag
        self._control_loop_initialized = False

        logger.info(f"Initialized {self.__class__.__name__}")
        logger.info(f"  Node ID: {node_id}")
        logger.info(f"  Robot type: {config.robot_type}")
        logger.info(f"  Interface: {config.interface.mode}")
        logger.info(
            f"  Control rate: {config.control.rate_hz}Hz ({self.dt*1000:.2f}ms period)"
        )
        logger.info(
            f"  Velocity limiting: {'enabled' if self.velocity_limiting_enabled else 'disabled'} "
            f"(max: {config.control.max_joint_velocity_rad_s} rad/s)"
        )

    @abstractmethod
    def setup(self):
        """Initialize robot-specific components.

        This method should:
        1. Create subscriber (e.g., ManusSubscriber)
        2. Create parser (e.g., ManusDataParser or ManusTrackerParser)
        3. Create robot model (e.g., DH5Model or NovaModel)
        4. Create optimizer/IK solver
        5. Create interface (sim or real)

        Example:
            self.subscriber = ManusSubscriber(...)
            self.parser = ManusDataParser()
            self.model = DH5Model(...)
            self.interface = DH5SimInterface(...)
        """
from __future__ import annotations

        pass

    @abstractmethod
    def process_data(self, manus_data: dict[str, Any]) -> np.typing.NDArray:
        """Process sensor data and generate joint configurations.

        This method implements the data processing pipeline:
        1. Parse raw data from subscriber
        2. Extract features
        3. Compute joint angles (IK or optimization)

        Args:
            manus_data: Raw JSON dict from subscriber.receive()
                       For DH5: skeleton data dict
                       For Nova: tracker data dict

        Returns:
            Joint positions array (np.ndarray), or None if processing failed

        Example (DH5):
            skeleton = self.parser.parse_raw_skeleton_data(manus_data)
            vectors = ManusFeatureExtractor.extract_position_vectors(skeleton)
            joint_angles = self.optimizer.retarget(vectors)
            return joint_angles  # np.ndarray
        """
from __future__ import annotations

        pass

    @abstractmethod
    def get_safe_position(self) -> Any:
        """Get safe fallback position for timeout/error situations.

        Returns:
            Safe position command (robot-specific)

        Example (DH5):
            return np.zeros(25)  # Open hand

        Example (Nova):
            return self.home_joints  # Home position
        """
from __future__ import annotations

        pass

    def _initialize_control_loop(self) -> None:
        """One-time initialization before control loop starts.

        This method is called once at the beginning of the control loop to:
        - Set up signal handlers for graceful shutdown
        - Initialize velocity limiting with current joint positions
        """
from __future__ import annotations

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
from __future__ import annotations

        # One-time initialization on first iteration
        if not self._control_loop_initialized:
            self._initialize_control_loop()

        # Gate control loop with _teleop_active flag
        # When not active, skip processing but maintain rate for responsive restart
        if not self._teleop_active:
            # Still maintain control rate for responsive command handling
            self.rate_limiter.sleep()
            return

        # Get latest data
        manus_data = self._get_data()

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
            if manus_data:
                try:
                    target_joints = self.process_data(manus_data)

                    # Apply velocity limiting for safe motion
                    safe_joints = self._apply_velocity_limiting(target_joints)

                except Exception as e:
                    logger.error(f"Error processing data: {e}", exc_info=True)
                    # Skip this frame, continue running
                    return

                try:
                    self._send_command(safe_joints)
                    self.last_data_time = time.time()
                except Exception as e:
                    logger.error(f"Error sending command: {e}", exc_info=True)

        # Maintain control rate with rate limiter
        timing = self.rate_limiter.sleep()

        # Log overtime warnings
        if timing["overtime"]:
            logger.debug(
                f"Loop overtime: {timing['elapsed']*1000:.2f}ms "
                f"(target: {self.dt*1000:.2f}ms)"
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

    def _get_data(self) -> dict[str, Any]:
        """Get latest data from subscriber.

        Uses subscriber.read() which provides:
        - Automatic message draining (receive_latest pattern)
        - Built-in parsing to structured format
        - Landscape updates and metrics tracking

        Returns:
            Dict with structure: {"skeletons": {}, "trackers": [], "raw": list}
            Or empty dict if no data available

        Note:
            The returned dict contains:
            - "trackers": List of TrackerData objects (parsed)
            - "skeletons": Dict of ManusSkeletonData (parsed)
            - "raw": Original raw list data

            Subclasses can access either parsed or raw data as needed.
        """
from __future__ import annotations

        try:
            data = self.subscriber.read()
            # read() returns {"skeletons": {}, "trackers": [], "raw": list}
            # For backward compatibility, if raw data exists, also expose it at top level
            if data and data.get("raw"):
                # Keep structured format but allow legacy access to raw
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
from __future__ import annotations

        if not self.config.control.safe_position_on_timeout:
            return False

        time_since_data = time.time() - self.last_data_time
        return time_since_data > self.config.control.timeout_sec

    def _apply_velocity_limiting(self, target_joints: np.ndarray) -> np.ndarray:
        """Apply velocity limits to target joint positions for safe motion.

        This method ensures that joint velocities do not exceed configured limits
        by clamping the change in position per control cycle. This prevents
        dangerous jumps in joint positions while maintaining responsiveness to
        the latest sensor data.

        Args:
            target_joints: Desired joint positions from process_data()

        Returns:
            Velocity-limited joint positions safe to send to robot

        Note:
            - Always moves toward latest target (no lag accumulation)
            - Falls back gracefully if current position cannot be read
            - Can be disabled via config.control.enable_velocity_limiting
        """
from __future__ import annotations

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

    def _move_to_position_safely(
        self,
        target_joints: np.ndarray,
        timeout_sec: float = 5.0,
        max_velocity_rad_s: Optional[float] = None,
    ) -> bool:
        """Move to target position with velocity limiting and smooth interpolation.

        This method ALWAYS applies velocity limiting and interpolation for safe
        movement, regardless of the config.control.enable_velocity_limiting setting.
        It's used for critical movements like going to safe position during timeout
        or shutdown.

        Args:
            target_joints: Target joint configuration
            timeout_sec: Maximum time to spend on movement (default: 5.0s)
            max_velocity_rad_s: Optional velocity limit (rad/s). If None, uses
                config.safe_position_max_velocity_rad_s, falling back to
                config.max_joint_velocity_rad_s. Priority:
                method parameter > config.safe_position > config.default

        Returns:
            True if reached target (within tolerance), False if timed out

        Note:
            - Uses smootherstep (5th order polynomial) interpolation for smooth motion
            - Smooth acceleration at start, smooth deceleration at end
            - Zero velocity and acceleration at boundaries (no jerk)
            - Respects velocity limits throughout the motion
            - Includes timeout protection to prevent infinite loops
            - Logs progress for movements taking >1 second
        """
from __future__ import annotations

        try:
            # Read current position
            current_state = self.interface.read()
            current_joints = current_state.q
        except Exception as e:
            logger.warning(
                f"Cannot read current position for safe movement: {e}. "
                "Attempting direct movement."
            )
            # Fallback to direct movement if we can't read position
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
            # Use explicitly provided velocity (highest priority)
            velocity_limit = max_velocity_rad_s
            logger.debug(f"Using method-provided velocity: {velocity_limit:.2f} rad/s")
        elif self.config.control.safe_position_max_velocity_rad_s is not None:
            # Use safe-position-specific config
            velocity_limit = self.config.control.safe_position_max_velocity_rad_s
            logger.debug(
                f"Using config safe_position velocity: {velocity_limit:.2f} rad/s"
            )
        else:
            # Fall back to general velocity limit
            velocity_limit = self.config.control.max_joint_velocity_rad_s
            logger.debug(f"Using default velocity: {velocity_limit:.2f} rad/s")

        # Calculate number of steps needed based on max velocity
        max_delta_per_step = velocity_limit * self.dt
        num_steps = int(np.ceil(max_joint_delta / max_delta_per_step))

        # Add safety margin and cap at timeout
        num_steps = min(num_steps + 5, int(timeout_sec / self.dt))

        logger.info(
            f"Moving to safe position: {num_steps} steps "
            f"(~{num_steps * self.dt:.2f}s, max delta: {max_joint_delta:.3f} rad, "
            f"velocity: {velocity_limit:.2f} rad/s)"
        )

        # Interpolate and send commands
        start_time = time.time()
        for i in range(num_steps):
            # Check timeout
            if time.time() - start_time > timeout_sec:
                logger.warning(f"Safe position movement timed out after {timeout_sec}s")
                return False

            # Smootherstep interpolation for smooth acceleration/deceleration
            t = (i + 1) / num_steps  # Linear parameter [0, 1]
            alpha = smootherstep(t)  # Smooth S-curve [0, 1]
            interpolated_joints = current_joints + alpha * delta

            # Send command
            self._send_command(interpolated_joints)

            # Sleep to maintain control rate (except last iteration)
            if i < num_steps - 1:
                self.rate_limiter.sleep()

        elapsed = time.time() - start_time
        logger.success(f"Reached safe position in {elapsed:.2f}s")
        return True

    def go_to_safe_position(self, max_velocity_rad_s: Optional[float] = None):
        """Move robot to safe position with velocity limiting.

        This method can be overridden by subclasses to provide robot-specific
        safe position behavior. The default implementation uses interpolated
        movement with velocity limiting for smooth, safe motion.

        Args:
            max_velocity_rad_s: Optional velocity limit override (rad/s).
                If None, uses config values. Priority:
                method parameter > config.safe_position_max_velocity_rad_s >
                config.max_joint_velocity_rad_s

        Note:
            Subclasses can override this method to customize safe position behavior.
            Use self._move_to_position_safely() for velocity-limited movement.
        """
from __future__ import annotations

        try:
            safe_joints = self.get_safe_position()
            if safe_joints is None:
                logger.warning("get_safe_position() returned None")
                return

            # Move to safe position with velocity limiting
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

    def _send_command(self, joint_cfgs: np.ndarray):
        """Send command to robot interface.

        This method creates a JointCommand and sends it to the interface.
        Subclasses may override this to customize command creation.

        Args:
            joint_cfgs: Joint positions array from process_data()
        """
from __future__ import annotations

        try:
            # Import here to avoid circular dependency and keep base class generic
            from dexim.core.robot_interface import JointCommand

            # Create JointCommand from positions
            command = JointCommand(q=joint_cfgs, mode="position")
            self.interface.write(command)
        except ImportError:
            # Fallback for custom interfaces that don't use JointCommand
            logger.warning("JointCommand not available, sending raw array")
            self.interface.write(joint_cfgs)
        except Exception as e:
            logger.error(f"Error sending command: {e}")

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals (Ctrl+C, SIGTERM).

        Args:
            signum: Signal number
            frame: Current stack frame
        """
from __future__ import annotations

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
from __future__ import annotations

        logger.info(f"{self.node_id} received START command - teleoperation active")
        # Subclasses override to capture tracker reference pose

    def on_pause(self) -> None:
        """Called when PAUSE command is received.

        Pauses teleoperation but holds current position for quick resume.
        The _teleop_active flag is set False by ManagedNode before calling.

        Subclasses can override to add robot-specific pause logic.
        """
from __future__ import annotations

        logger.info(f"{self.node_id} received PAUSE command - holding position")
        # Hold current position - no movement, ready for quick resume

    def on_stop(self) -> None:
        """Called when STOP command is received.

        Stops teleoperation and moves robot to safe position.
        The _teleop_active flag is set False by ManagedNode before calling.

        On next START, reference pose will be recaptured.
        """
from __future__ import annotations

        logger.info(f"{self.node_id} received STOP command - going to safe position")
        try:
            self.go_to_safe_position()
        except Exception as e:
            logger.error(f"Error moving to safe position on stop: {e}")

    def on_start_recording(self) -> None:
        """Called when START_REC command is received (recording control).

        Begins recording/data collection. The robot control loop continues running.

        Subclasses can override to add robot-specific recording logic.
        """
from __future__ import annotations

        logger.info(f"{self.node_id} started recording")
        # is_recording flag already set by ManagedNode
        # Subclasses can override to integrate with data recorder

    def on_stop_recording(self) -> None:
        """Called when STOP_REC command is received (recording control).

        Stops recording/data collection. The robot control loop continues running.

        Subclasses can override to add robot-specific save logic (flush buffers, etc).
        """
from __future__ import annotations

        logger.info(f"{self.node_id} stopped recording")
        # is_recording flag already cleared by ManagedNode
        # Subclasses can override to save/flush data

    def on_shutdown(self) -> None:
        """Called during shutdown sequence (before ZMQ sockets are closed).

        This is triggered by the SHUTDOWN command or signal handler.
        Performs robot-specific cleanup: safe position, disconnect interface, close subscriber.
        """
from __future__ import annotations

        logger.info(f"{self.node_id} received SHUTDOWN command - cleaning up robot")
        self._cleanup_robot()

    # ----------------------
    # Robot cleanup
    # ----------------------
    def _cleanup_robot(self):
        """Cleanup and shutdown robot resources.

        This method:
        1. Moves to safe position
        2. Disconnects interface
        3. Closes subscriber
        """
from __future__ import annotations

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
from __future__ import annotations

        self.setup()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
from __future__ import annotations

        self._cleanup_robot()
