"""
Optimizer module for core_model package.

This module provides optimization tools for inverse kinematics:
- VectorOptimizer: Main optimizer class for vector-based IK using nlopt
- OptimizerConfig: Configuration dataclass for VectorOptimizer parameters
- NloptReturn: Enum for nlopt optimization return codes
"""

from __future__ import annotations

from dexim.core.model.optimizer.optimizer import (
    NloptReturn,
    OptimizerConfig,
    VectorOptimizer,
)

__all__ = [
    # Main optimizer classes
    "VectorOptimizer",
    # Configuration
    "OptimizerConfig",
    # Enums
    "NloptReturn",
]
