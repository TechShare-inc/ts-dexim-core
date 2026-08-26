"""
Core Model - Reusable robot model utilities.

This package provides core utilities for robotics packages:
- URDF loading with package:// URL resolution
- Weighted moving average filtering
- Base visualizer class for Viser-based visualization
- BaseHandModel: Abstract base class for hand robot models
- VectorOptimizer: Optimization for inverse kinematics

Example:
    from dexim.core.model import load_urdf_model, load_urdf_models
    from dexim.core.model import BaseHandModel, VectorOptimizer, OptimizerConfig
    from dexim.core.model.filtering import WeightedMovingFilter
    from dexim.core.model.visualizer import BaseRobotVisualizer
"""

from __future__ import annotations

from dexim.core.model.base_hand_model import BaseHandModel
from dexim.core.model.helpers import replace_package_url_in_content
from dexim.core.model.utils import load_urdf_model, load_urdf_models

__version__ = "0.1.0"

__all__ = [
    "load_urdf_model",
    "load_urdf_models",
    "replace_package_url_in_content",
    "BaseHandModel",
    "VectorOptimizer",
    "OptimizerConfig",
    "NloptReturn",
]

_LAZY_OPTIMIZER_EXPORTS = {
    "NloptReturn",
    "OptimizerConfig",
    "VectorOptimizer",
}


def __getattr__(name: str):
    """Import optimizer dependencies only when explicitly requested."""
    if name not in _LAZY_OPTIMIZER_EXPORTS:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

    from dexim.core.model.optimizer import (
        NloptReturn,
        OptimizerConfig,
        VectorOptimizer,
    )

    exports = {
        "NloptReturn": NloptReturn,
        "OptimizerConfig": OptimizerConfig,
        "VectorOptimizer": VectorOptimizer,
    }

    # Cache imported objects so subsequent access does not import them again.
    globals().update(exports)
    return exports[name]