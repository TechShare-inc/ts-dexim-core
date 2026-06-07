"""TeleopOrchestrator - Central coordinator for multi-node teleoperation system.

This module provides the TeleopOrchestrator class for managing multiple robot
control nodes in a distributed teleoperation system. It handles:

- Process management: Launch and monitor node processes
- Control plane: Broadcast commands (START, STOP_REC, SHUTDOWN) to all nodes
- Status plane: Collect and track health/status from all nodes
- Lifecycle management: Coordinate startup, recording, and graceful shutdown

Architecture:
    Orchestrator (Control Plane PUB + Status Plane PULL)
         │
         ├─> Node 1 (SUB control, PUSH status)
         ├─> Node 2 (SUB control, PUSH status)
         └─> Node N (SUB control, PUSH status)

Example:
    from dexim.core.nodes import TeleopOrchestrator

    # Create orchestrator
    orch = TeleopOrchestrator()

    # Launch nodes
    orch.launch_node("nova_left", "scripts/run_nova.py", "examples/configs/nova_sim_l.yaml")
    orch.launch_node("nova_right", "scripts/run_nova.py", "examples/configs/nova_sim_r.yaml")

    # Wait for all nodes to be ready
    orch.wait_for_nodes(timeout=10.0)

    # Start synchronized recording
    orch.send_command("START")

    # ... collect data ...
    time.sleep(10)

    # Stop recording
    orch.send_command("STOP_REC")

    # Shutdown
    orch.shutdown()

Author: Haoyan Li
Date: November 12, 2025
"""

from __future__ import annotations

import json
import subprocess
import sys
import time
from pathlib import Path

import zmq
from dexim.core.messages import (
    CTRL_PAUSE,
    CTRL_PUB_ENDPOINT,
    CTRL_SHUTDOWN,
    CTRL_START,
    CTRL_START_REC,
    CTRL_STOP_REC,
    STATUS_HEALTHY,
    STATUS_INITIALIZED,
    STATUS_PULL_ENDPOINT,
    TOPIC_CTRL,
)
from loguru import logger


class NodeStatus:
    """Status information for a single node."""

    def __init__(self, node_id: str):
        self.node_id = node_id
        self.status: str = "UNKNOWN"
        self.is_recording: bool = False
        self.last_heartbeat: float = 0.0
        self.info: dict = {}

    def update(self, status: str, is_recording: bool, timestamp: float, info: dict):
        """Update status from received message."""
        self.status = status
        self.is_recording = is_recording
        self.last_heartbeat = timestamp
        self.info = info

    def is_healthy(self, timeout: float = 5.0) -> bool:
        """Check if node is healthy (recent heartbeat)."""
        if self.status == "UNKNOWN":
            return False
        time_since_heartbeat = time.time() - self.last_heartbeat
        return time_since_heartbeat < timeout

    def __repr__(self) -> str:
        return (
            f"NodeStatus({self.node_id}, {self.status}, "
            f"recording={self.is_recording}, "
            f"healthy={self.is_healthy()})"
        )


class TeleopOrchestrator:
    """Central orchestrator for multi-node teleoperation system.

    This class manages the lifecycle of multiple robot control nodes,
    providing centralized command broadcasting and status monitoring.
    """

    def __init__(
        self,
        control_endpoint: str = CTRL_PUB_ENDPOINT,
        status_endpoint: str = STATUS_PULL_ENDPOINT,
    ):
        """Initialize orchestrator.

        Args:
            control_endpoint: Control plane endpoint to bind (default: tcp://*:5550)
            status_endpoint: Status plane endpoint to bind (default: tcp://*:5551)
        """
        self.control_endpoint = control_endpoint
        self.status_endpoint = status_endpoint

        # ZMQ context and sockets
        self._ctx: zmq.Context | None = None
        self._pub_control: zmq.Socket | None = None
        self._pull_status: zmq.Socket | None = None
        self._poller: zmq.Poller | None = None

        # Node tracking
        self._processes: dict[str, subprocess.Popen] = {}
        self._node_statuses: dict[str, NodeStatus] = {}
        self._expected_nodes: set[str] = set()

        # Initialize ZMQ
        self._initialize_zmq()

        logger.info("TeleopOrchestrator initialized")
        logger.info(f"  Control endpoint: {control_endpoint}")
        logger.info(f"  Status endpoint: {status_endpoint}")

    def _initialize_zmq(self) -> None:
        """Initialize ZMQ context and sockets."""
        self._ctx = zmq.Context.instance()

        # Control Plane: PUB socket (bind)
        self._pub_control = self._ctx.socket(zmq.PUB)  # type: ignore
        self._pub_control.setsockopt(zmq.SNDHWM, 100)
        # Replace localhost with 0.0.0.0 for binding
        bind_addr = self.control_endpoint.replace("localhost", "0.0.0.0")
        if bind_addr.startswith("tcp://*"):
            bind_addr = bind_addr.replace("*", "0.0.0.0")
        self._pub_control.bind(bind_addr)  # type: ignore
        logger.debug(f"Control plane PUB bound to: {bind_addr}")

        # Status Plane: PULL socket (bind)
        self._pull_status = self._ctx.socket(zmq.PULL)  # type: ignore
        bind_addr = self.status_endpoint.replace("localhost", "0.0.0.0")
        if bind_addr.startswith("tcp://*"):
            bind_addr = bind_addr.replace("*", "0.0.0.0")
        self._pull_status.bind(bind_addr)  # type: ignore
        logger.debug(f"Status plane PULL bound to: {bind_addr}")

        # Poller for status messages
        self._poller = zmq.Poller()
        self._poller.register(self._pull_status, zmq.POLLIN)

        # Small delay to ensure bindings are ready
        time.sleep(0.2)

    def launch_node(
        self,
        node_id: str,
        script_path: str,
        config_path: str,
        *,
        args: list[str] | None = None,
        cwd: str | None = None,
    ) -> subprocess.Popen:
        """Launch a node as a subprocess.

        Args:
            node_id: Unique identifier for the node (e.g., "nova_left")
            script_path: Path to the run script (e.g., "scripts/run_nova.py")
            config_path: Path to the config file (e.g., "examples/configs/nova_sim_l.yaml")
            args: Additional command-line arguments for the script
            cwd: Working directory for the subprocess (default: project root)

        Returns:
            subprocess.Popen: The launched process

        Raises:
            ValueError: If node_id already exists
            FileNotFoundError: If script or config not found
        """
        if node_id in self._processes:
            raise ValueError(f"Node '{node_id}' already launched")

        # Validate paths
        script = Path(script_path)
        config = Path(config_path)

        if not script.exists():
            raise FileNotFoundError(f"Script not found: {script_path}")
        if not config.exists():
            raise FileNotFoundError(f"Config not found: {config_path}")

        # Build command
        cmd = [
            sys.executable,  # Python interpreter
            str(script.absolute()),
            "--config",
            str(config.absolute()),
            "--auto-run",  # Skip interactive prompts
        ]

        if args:
            cmd.extend(args)

        # Launch process
        logger.info(f"Launching node '{node_id}'...")
        logger.debug(f"  Command: {' '.join(cmd)}")

        process = subprocess.Popen(
            cmd,
            cwd=cwd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,  # Line buffered
        )

        # Track process and node
        self._processes[node_id] = process
        self._expected_nodes.add(node_id)
        self._node_statuses[node_id] = NodeStatus(node_id)

        logger.success(f"Node '{node_id}' launched (PID: {process.pid})")
        return process

    def send_command(self, command: str) -> None:
        """Broadcast a control command to all nodes.

        Args:
            command: Command string (START, PAUSE, SHUTDOWN, START_REC, STOP_REC)
        """
        if self._pub_control is None:
            logger.warning("Control socket not initialized")
            return

        # Validate command
        valid_commands = {
            CTRL_START,
            CTRL_PAUSE,
            CTRL_SHUTDOWN,
            CTRL_START_REC,
            CTRL_STOP_REC,
        }
        if command not in valid_commands:
            logger.warning(f"Unknown command: {command}")

        # Broadcast command
        try:
            self._pub_control.send_multipart(
                [TOPIC_CTRL, command.encode("utf-8")], flags=zmq.DONTWAIT
            )
            logger.info(f"📢 Broadcast command: {command}")
        except Exception as e:
            logger.error(f"Error sending command: {e}")

    def poll_status(self, timeout_ms: int = 10) -> int:
        """Poll for status messages and update node tracking.

        Args:
            timeout_ms: Polling timeout in milliseconds

        Returns:
            Number of status messages received
        """
        if self._poller is None or self._pull_status is None:
            return 0

        count = 0
        events = dict(self._poller.poll(timeout=timeout_ms))

        while self._pull_status in events:
            try:
                # Receive status message
                payload = self._pull_status.recv(flags=zmq.NOBLOCK)
                msg = json.loads(payload.decode("utf-8"))

                # Extract fields
                node_id = msg.get("node_id", "unknown")
                status = msg.get("status", "UNKNOWN")
                is_recording = msg.get("is_recording", False)
                timestamp = msg.get("timestamp", time.time())
                info = msg.get("info", {})

                # Update node status
                if node_id not in self._node_statuses:
                    self._node_statuses[node_id] = NodeStatus(node_id)

                self._node_statuses[node_id].update(
                    status, is_recording, timestamp, info
                )

                count += 1
                logger.debug(
                    f"Status from '{node_id}': {status} (recording={is_recording})"
                )

            except zmq.Again:
                break
            except Exception as e:
                logger.error(f"Error processing status message: {e}")
                break

            # Check for more messages
            events = dict(self._poller.poll(timeout=0))

        return count

    def get_node_status(self, node_id: str) -> NodeStatus | None:
        """Get status of a specific node.

        Args:
            node_id: Node identifier

        Returns:
            NodeStatus if node exists, None otherwise
        """
        return self._node_statuses.get(node_id)

    def get_all_statuses(self) -> dict[str, NodeStatus]:
        """Get status of all nodes.

        Returns:
            Dictionary mapping node_id to NodeStatus
        """
        return self._node_statuses.copy()

    def wait_for_nodes(
        self,
        timeout: float = 10.0,
        required_status: str = STATUS_INITIALIZED,
    ) -> bool:
        """Wait for all expected nodes to reach a status.

        Args:
            timeout: Maximum time to wait (seconds)
            required_status: Status to wait for (default: INITIALIZED)

        Returns:
            True if all nodes reached status, False if timeout
        """
        logger.info(f"Waiting for {len(self._expected_nodes)} nodes to be ready...")

        start_time = time.time()
        ready_nodes: set[str] = set()

        while time.time() - start_time < timeout:
            # Poll for status updates
            self.poll_status(timeout_ms=100)

            # Check which nodes are ready
            for node_id in self._expected_nodes:
                if node_id in ready_nodes:
                    continue

                status = self._node_statuses.get(node_id)
                if status and status.status in {required_status, STATUS_HEALTHY}:
                    ready_nodes.add(node_id)
                    logger.success(f"  ✓ Node '{node_id}' ready")

            # All nodes ready?
            if ready_nodes == self._expected_nodes:
                logger.success("All nodes ready!")
                return True

            time.sleep(0.1)

        # Timeout
        missing = self._expected_nodes - ready_nodes
        logger.error(f"Timeout waiting for nodes: {missing}")
        return False

    def shutdown(self, wait_timeout: float = 5.0) -> None:
        """Shutdown all nodes and cleanup resources.

        Args:
            wait_timeout: Time to wait for graceful shutdown (seconds)
        """
        logger.info("Shutting down orchestrator...")

        # Send SHUTDOWN command to all nodes
        self.send_command(CTRL_SHUTDOWN)

        # Wait for processes to exit gracefully
        logger.info(f"Waiting {wait_timeout}s for nodes to shutdown...")
        time.sleep(wait_timeout)

        # Force kill any remaining processes
        for node_id, process in self._processes.items():
            if process.poll() is None:  # Still running
                logger.warning(f"Force killing node '{node_id}' (PID: {process.pid})")
                process.kill()
                process.wait()
            else:
                logger.debug(f"Node '{node_id}' exited cleanly")

        # Cleanup ZMQ
        self._cleanup_zmq()

        logger.success("Orchestrator shutdown complete")

    def _cleanup_zmq(self) -> None:
        """Close ZMQ sockets and context."""
        try:
            if self._pub_control is not None:
                self._pub_control.close(linger=0)
        finally:
            self._pub_control = None

        try:
            if self._pull_status is not None:
                self._pull_status.close(linger=0)
        finally:
            self._pull_status = None

        self._poller = None
        # Don't terminate global context to avoid impacting others
        self._ctx = None

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.shutdown()
