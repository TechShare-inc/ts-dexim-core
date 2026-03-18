"""Exponential Moving Average (EMA) filter for joint-angle smoothing.

A first-order IIR low-pass filter:

    q_out[k] = alpha * q_in[k] + (1 - alpha) * q_out[k-1]

Compared to ``WeightedMovingFilter`` (FIR):

* **Lower lag** — responds in one sample, not after the window fills.
* **No warm-up** — produces valid output immediately.
* **Single parameter** — ``alpha`` in ``(0, 1]``:
  - ``alpha = 1.0`` → pass-through (no smoothing).
  - ``alpha = 0.3`` → moderate smoothing (~2 frame time constant at 30 Hz).
  - ``alpha = 0.1`` → heavy smoothing (~9 frame time constant at 30 Hz).
"""

from __future__ import annotations

import numpy as np
import numpy.typing as npt


class ExponentialMovingFilter:
    """First-order IIR low-pass filter applied jointly across all joints.

    Args:
        alpha: Smoothing factor in ``(0, 1]``.  Higher values track the
            signal more closely (less smoothing, less lag).
        data_size: Number of joints (determines vector width).

    Example::

        f = ExponentialMovingFilter(alpha=0.3, data_size=6)
        while running:
            q_smooth = f.filter(q_raw)
    """

    def __init__(self, alpha: float, data_size: int) -> None:
        if not (0.0 < alpha <= 1.0):
            raise ValueError(f"alpha must be in (0, 1], got {alpha}")
        if data_size <= 0:
            raise ValueError(f"data_size must be positive, got {data_size}")

        self._alpha = float(alpha)
        self._data_size = int(data_size)
        self._prev: np.ndarray | None = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def filter(self, data: npt.ArrayLike) -> np.ndarray:
        """Apply EMA to a new sample.

        On the first call the sample is returned as-is (cold-start).

        Args:
            data: New joint-angle vector of shape ``(data_size,)``.

        Returns:
            Filtered joint-angle vector of shape ``(data_size,)``.
        """
        x = np.asarray(data, dtype=float)
        if x.shape != (self._data_size,):
            raise ValueError(f"Expected shape ({self._data_size},), got {x.shape}")

        if self._prev is None:
            # Cold start: pass through, initialise state.
            self._prev = x.copy()
            return x.copy()

        out = self._alpha * x + (1.0 - self._alpha) * self._prev
        self._prev = out
        return out.copy()

    def reset(self) -> None:
        """Clear filter state (use after a pause or discontinuity)."""
        self._prev = None


__all__ = ["ExponentialMovingFilter"]
