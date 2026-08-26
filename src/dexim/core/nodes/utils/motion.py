"""Motion utility functions shared across node types."""

from __future__ import annotations

import numpy as np


def smootherstep(t: float) -> float:
    """Apply smootherstep interpolation (5th-order polynomial).

    Provides smooth acceleration and deceleration with zero velocity
    and acceleration at the boundaries (t=0 and t=1), making it ideal
    for robot motion profiles.

    Formula: 6t^5 - 15t^4 + 10t^3  (Ken Perlin's improved smoothstep)

    Args:
        t: Linear interpolation parameter in [0, 1].

    Returns:
        Smoothed interpolation parameter in [0, 1].
    """
    t = np.clip(t, 0.0, 1.0)
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)
