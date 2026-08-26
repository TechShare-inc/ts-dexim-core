"""
VectorOptimizer: Optimization tools for vector-based inverse kinematics.

This module provides the VectorOptimizer class for retargeting hand keypoints
to robot joint configurations using nlopt optimization.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING

import nlopt
import numpy as np
import numpy.typing as npt
import torch

try:
    import pinocchio as pin
except ImportError as _err:
    raise ImportError(
        "pinocchio is required by dexim.core.model but is not installed. "
        "Install it via conda: conda install pinocchio -c conda-forge"
    ) from _err
from loguru import logger
from torch.nn.functional import huber_loss as huber_loss_fn

if TYPE_CHECKING:
    from dexim.core.model.base_hand_model import BaseHandModel


class NloptReturn(Enum):
    SUCCESS = 1
    STOPVAL_REACHED = 2
    FTOL_REACHED = 3
    XTOL_REACHED = 4
    MAXEVAL_REACHED = 5
    MAXTIME_REACHED = 6
    FAILURE = -1
    INVALID_ARGS = -2
    OUT_OF_MEMORY = -3
    ROUNDOFF_LIMITED = -4
    FORCED_STOP = -5


@dataclass
class OptimizerConfig:
    """Configuration parameters for the VectorOptimizer."""

    num_of_qs: int = 7
    num_of_features: int = 5
    feature_space_dim: int = 3
    alpha: list[float] = field(
        default_factory=lambda: [1.0]
    )  # Scaling factor for each dimension
    beta: float = 1e-12  # Weight for the temporal consistency penalty
    lower_bounds: list[float] | None = None
    upper_bounds: list[float] | None = None
    xtol_rel: float = 1e-4  # Relative tolerance for stopping
    ftol_rel: float = 1e-4  # Relative tolerance for stopping
    max_time: float = 0.01  # Global maxtime in seconds (default 10 ms)

    def __post_init__(self):
        """Set default joint limits if not provided."""
        if len(self.alpha) != self.num_of_features:
            self.alpha = [1.0] * self.num_of_features
        if self.lower_bounds is None:
            self.lower_bounds = [-np.pi] * self.num_of_qs
        if self.upper_bounds is None:
            self.upper_bounds = [np.pi] * self.num_of_qs


class VectorOptimizer:
    # Maximum number of outer optimization iterations (manifold-retraction loop).
    _MAX_ITER: int = 10
    _MANIFOLD_CONVERGENCE_THRESHOLD: float = 1e-6

    def __init__(
        self,
        hand_model: BaseHandModel,
        config: OptimizerConfig | None = None,
    ):
        """
        Initialize the VectorOptimizer with configuration parameters.

        Args:
            hand_model: BaseHandModel instance for hand kinematics
            config (OptimizerConfig): Configuration object containing all optimizer parameters.
                                    If None, default configuration will be used.
        """
        # Use default config if none provided
        np.set_printoptions(precision=4, suppress=True)

        self.config = config if config is not None else OptimizerConfig()
        self.robot = hand_model

        # Get model_name for logging
        model_name = hand_model.__class__.__name__

        self.logger = logger.bind(model_name=model_name)
        self.nq = self.robot.model.nq
        self.nv = self.robot.model.nv
        self.MAX_ITER = self._MAX_ITER
        self.MANIFOLD_CONVERGENCE_THRESHOLD = self._MANIFOLD_CONVERGENCE_THRESHOLD

        # Store config values as instance attributes for easy access
        self.num_joints = self.config.num_of_qs
        self.feature_count = self.config.num_of_features
        self.feature_dim = self.config.feature_space_dim
        self.alpha = self.config.alpha
        self.beta = self.config.beta
        self.lower_bounds = [-np.pi / 4] * self.nv
        self.upper_bounds = [np.pi / 4] * self.nv
        self.xtol_rel = self.config.xtol_rel
        self.ftol_rel = self.config.ftol_rel
        # Use configurable global max time (seconds)
        self.max_time = self.config.max_time
        self.logger.debug(f"Initialized config: {self.config}")

        # Get actual robot joint limits from the model
        self.joint_lower_limits, self.joint_upper_limits = self.robot.get_joint_limits()
        self.logger.debug(
            f"Robot joint limits: lower={self.joint_lower_limits}, upper={self.joint_upper_limits}"
        )

        # Initialize robot keypoint indices
        self._init_robot_keypoint_variables()

        # Initialize state
        self.q_last = np.zeros(self.nq)
        self.v_last = np.zeros(self.nv)
        self.logger.debug(f"Initialized state: {self.q_last}")

        self.logger.info("VectorOptimizer initialized.")

    def _init_robot_keypoint_variables(self) -> None:
        """
        Initialize the robot keypoint variables using the model's keypoint targets.
        """
        # Get keypoint targets from the model
        targets = self.robot.get_keypoint_targets()

        self.robot_keypoint_src_names = [src for src, _ in targets]
        self.robot_keypoint_dst_names = [dst for _, dst in targets]
        self.robot_keypoint_src_ids = [
            self.robot.model.getFrameId(src) for src in self.robot_keypoint_src_names
        ]
        self.robot_keypoint_dst_ids = [
            self.robot.model.getFrameId(dst) for dst in self.robot_keypoint_dst_names
        ]

        self.logger.debug("Initialized robot keypoint variables.")

    def create_optimizer(self) -> nlopt.opt:
        """
        Create and configure the NLopt optimizer.

        Returns:
            Configured NLopt optimizer instance.
        """
        opt = nlopt.opt(nlopt.LD_SLSQP, self.nv)

        opt.set_ftol_rel(self.ftol_rel)
        opt.set_upper_bounds(self.upper_bounds)
        opt.set_lower_bounds(self.lower_bounds)
        # Set NLopt's internal max time for local optimization
        opt.set_maxtime(self.max_time)
        return opt

    def retarget(self, target_features: npt.NDArray) -> tuple[npt.NDArray, NloptReturn]:
        if target_features.shape != (self.feature_count, self.feature_dim):
            raise ValueError(f"Invalid target shape: {target_features.shape}")

        current_time = time.time()

        q_init = self.q_last.copy()
        q_optimal = q_init.copy()
        reason = NloptReturn.FAILURE

        iter = 0
        for iter in range(self.MAX_ITER):
            self.logger.debug(f"Optimization iteration {iter + 1}/{self.MAX_ITER}")

            # Check global elapsed time before starting this iteration
            elapsed = time.time() - current_time
            if elapsed > self.max_time:
                self.logger.debug(
                    f"Global optimization timeout reached after {elapsed:.6f}s (max {self.max_time}s)"
                )
                reason = NloptReturn.MAXTIME_REACHED
                break

            q_optimal, _, local_result = self._local_optimize(target_features, q_init)

            # Map local optimizer result to our enum (safe conversion)
            try:
                local_reason = NloptReturn(local_result)
            except Exception:
                local_reason = NloptReturn.FAILURE

            # If local optimizer timed out, propagate as global timeout
            if local_reason == NloptReturn.MAXTIME_REACHED:
                reason = NloptReturn.MAXTIME_REACHED
                break

            eval_value = np.abs(np.mean(q_init - q_optimal))
            self.logger.debug(f"Manifold eval value: {eval_value}")
            if eval_value <= self.xtol_rel:
                self.logger.debug(
                    f"Optimizer converged at iteration {iter + 1}/{self.MAX_ITER} "
                    f"for the manifold."
                )
                reason = NloptReturn.FTOL_REACHED
                break

            q_init = q_optimal.copy()

        if reason == NloptReturn.FAILURE and iter == self.MAX_ITER - 1:
            reason = NloptReturn.MAXEVAL_REACHED

        end_time = time.time()
        self.logger.info(f"Optimization took {end_time - current_time:.6f} seconds")

        return q_optimal, reason

    def _local_optimize(
        self, target_features: npt.NDArray, q_init: npt.NDArray
    ) -> tuple[npt.NDArray, float, int]:
        """
        Retarget the hand keypoints to the robot keypoints.

        Args:
            target_features: Target features to optimize towards

        Returns:
            Optimal joint configuration
        """
        start_time = time.time()
        opt = self.create_optimizer()
        opt.set_min_objective(
            lambda v, grad: self.objective_fn(v, grad, target_features, q_init)
        )

        try:
            v_init = np.zeros(self.nv)
            v_optimal = opt.optimize(v_init)

            # Check optimization result
            last_result = opt.last_optimize_result()
            if last_result < 0:
                # Log negative (failure) codes
                try:
                    name = NloptReturn(last_result).name
                except Exception:
                    name = str(last_result)
                self.logger.warning(f"NLopt optimization failed with code: {name}")
                return self.q_last, float("inf"), last_result
            else:
                self.logger.debug(
                    f"Optimal velocities({len(v_optimal)}): {v_optimal}, "
                    f"loss: {opt.last_optimum_value()}, "
                    f"Result: {NloptReturn(last_result).name}"
                )

            # Update stored configuration for next iteration
            self.v_last = v_optimal
            self.q_last = pin.integrate(self.robot.model, q_init, v_optimal)
            self.q_last = np.clip(
                self.q_last, self.joint_lower_limits, self.joint_upper_limits
            )

            self.logger.debug(
                f"Optimization succeeded with loss: {opt.last_optimum_value():.6f}"
            )

        except Exception as e:
            self.logger.exception(f"An error occurred during optimization: {e}")
            return self.q_last, float("inf"), NloptReturn.FAILURE.value
        end_time = time.time()
        self.logger.debug(
            f"Local optimization took {end_time - start_time:.6f} seconds"
        )
        return self.q_last, opt.last_optimum_value(), opt.last_optimize_result()

    def objective_fn(
        self,
        v: npt.NDArray,
        grad: npt.NDArray,
        input_features: npt.NDArray,
        q_init: npt.NDArray,
    ) -> float:
        """
        Creates and returns the objective function to be minimized by NLopt.
        This function implements the formula:

        $min(q_t) \\sum_{i=1}^{N} ||\\alpha * v_{ti} - f_i(q_t)||^2 + \\beta * ||q_t - q_{t-1}||^2$

        Args:
            input_features: Target features to optimize towards
            q_init: Previous configuration for temporal consistency
            v: Current velocities
            grad: Gradient output array

        Returns:
            Objective function value (loss)
        """
        input_features_tensor = torch.as_tensor(input_features).requires_grad_(False)

        # Forward pass: compute robot configuration and positions
        v_curr = v.copy()
        q_curr = pin.integrate(self.robot.model, q_init, v_curr)
        pin.forwardKinematics(self.robot.model, self.robot.data, q_curr)
        pin.updateFramePlacements(self.robot.model, self.robot.data)

        # Get unique keypoint link names and their frame IDs
        src_link_indices = self.robot_keypoint_src_ids
        dst_link_indices = self.robot_keypoint_dst_ids

        # Extract link positions from robot data
        src_link_poses = [self.robot.data.oMf[idx] for idx in src_link_indices]
        dst_link_poses = [self.robot.data.oMf[idx] for idx in dst_link_indices]
        src_link_positions = np.array([pose.translation for pose in src_link_poses])
        dst_link_positions = np.array([pose.translation for pose in dst_link_poses])

        # Convert to PyTorch tensor for automatic differentiation
        src_positions_tensor = torch.as_tensor(src_link_positions).requires_grad_()
        dst_positions_tensor = torch.as_tensor(dst_link_positions).requires_grad_()

        # Compute robot features (destination - source for each pair)
        robot_features_tensor = dst_positions_tensor - src_positions_tensor

        # Apply Huber loss for robustness
        huber_loss = huber_loss_fn(
            input_features_tensor, robot_features_tensor, delta=1.0, reduction="sum"
        )

        # Add temporal consistency penalty: beta * ||q_t - q_{t-1}||^2
        q_prev_tensor = torch.as_tensor(q_init).requires_grad_()
        q_curr_tensor = torch.as_tensor(q_curr).requires_grad_()
        temporal_penalty = self.beta * torch.sum((q_curr_tensor - q_prev_tensor) ** 2)
        total_loss = huber_loss + temporal_penalty

        result = total_loss.cpu().detach().item()

        # Backward pass: compute gradient dLoss/dq
        if grad.size > 0:
            total_loss.backward()

            # Compute Jacobian of link positions with respect to q,
            # represented in the tangent space at q_curr
            link_names = self.robot_keypoint_src_names + self.robot_keypoint_dst_names
            J_positions_q = np.zeros((3 * len(link_names), self.robot.model.nv))

            for i, link_name in enumerate(link_names):
                frame_id = self.robot.model.getFrameId(link_name)
                # Get frame Jacobian (position part only)
                pin.computeJointJacobians(self.robot.model, self.robot.data, q_curr)
                frame_jacobian = pin.getFrameJacobian(
                    self.robot.model,
                    self.robot.data,
                    frame_id,
                    pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
                )
                # Extract linear velocity part (3 x nv)
                J_positions_q[i * 3 : (i + 1) * 3, :] = frame_jacobian[:3, :]

            # Chain rule: dLoss/dq = dLoss/dPositions @ dPositions/dq
            if src_positions_tensor.grad is None or dst_positions_tensor.grad is None:
                self.logger.error("Gradient is None - this should not happen")
                return float("inf")

            grad_loss_positions = (
                torch.cat(
                    [
                        src_positions_tensor.grad.view(-1),
                        dst_positions_tensor.grad.view(-1),
                    ]
                )
                .cpu()
                .detach()
                .numpy()
            )  # Flatten to (3F,)

            grad_loss_q = grad_loss_positions @ J_positions_q

            J_integrate_v = pin.dIntegrate(
                self.robot.model, q_init, v_curr, pin.ArgumentPosition.ARG1
            )
            grad_loss_v = grad_loss_q @ J_integrate_v

            grad[:] = grad_loss_v

        return result
