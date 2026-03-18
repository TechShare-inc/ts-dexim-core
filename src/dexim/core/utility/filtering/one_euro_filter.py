"""One Euro Filter for adaptive low-pass smoothing.

Casiez, G., Roussel, N., & Vogel, D. (2012). 1€ filter: A simple speed-based
low-pass filter for noisy input in interactive systems. CHI 2012.

The filter adapts its cutoff frequency to signal speed:
- Slow motion  → low cutoff  → heavy smoothing, minimal jitter.
- Fast motion  → high cutoff → less lag, responsive tracking.

This makes it ideal for teleoperation: glove data is naturally noisy at rest
but must track fast intentional movements with minimal delay.
"""

from __future__ import annotations

import numpy as np
import numpy.typing as npt


class OneEuroFilter:
    """Adaptive first-order low-pass filter for joint-angle trajectories.

    Each joint is filtered independently.  The cutoff frequency rises with
    the estimated per-joint speed so that fast intentional motion is tracked
    with low lag while slow/stationary joints are heavily smoothed.

    Args:
        freq: Sampling frequency in Hz (must match your control-loop rate).
        min_cutoff: Minimum cutoff frequency in Hz.  Lower values give more
            smoothing at rest (default 1.0 Hz).
        beta: Speed coefficient.  Higher values make the filter more
            responsive at high velocity but reduce smoothing.  Set to 0 to
            disable speed adaptation (pure fixed-cutoff LP) (default 0.007).
        d_cutoff: Cutoff frequency for the derivative (speed) estimate in Hz
            (default 1.0 Hz).  Rarely needs tuning.
        data_size: Number of joints (determines vector width).

    Example::

        f = OneEuroFilter(freq=30.0, min_cutoff=1.5, beta=0.01, data_size=6)
        while running:
            q_smooth = f.filter(q_raw)
    """

    def __init__(
        self,
        freq: float,
        min_cutoff: float = 1.0,
        beta: float = 0.007,
        d_cutoff: float = 1.0,
        data_size: int = 1,
    ) -> None:
        if freq <= 0:
            raise ValueError(f"freq must be positive, got {freq}")
        if min_cutoff <= 0:
            raise ValueError(f"min_cutoff must be positive, got {min_cutoff}")
        if beta < 0:
            raise ValueError(f"beta must be non-negative, got {beta}")
        if d_cutoff <= 0:
            raise ValueError(f"d_cutoff must be positive, got {d_cutoff}")
        if data_size <= 0:
            raise ValueError(f"data_size must be positive, got {data_size}")

        self._freq = float(freq)
        self._min_cutoff = float(min_cutoff)
        self._beta = float(beta)
        self._d_cutoff = float(d_cutoff)
        self._data_size = int(data_size)

        self._x_prev: np.ndarray | None = None
        self._dx_est = np.zeros(self._data_size, dtype=float)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def filter(self, data: npt.ArrayLike) -> np.ndarray:
        """Apply the One Euro Filter to a new sample.

        On the first call the sample is returned as-is (cold-start).

        Args:
            data: New joint-angle vector of shape ``(data_size,)``.

        Returns:
            Filtered joint-angle vector of shape ``(data_size,)``.
        """
        x = np.asarray(data, dtype=float)
        if x.shape != (self._data_size,):
            raise ValueError(f"Expected shape ({self._data_size},), got {x.shape}")

        if self._x_prev is None:
            # Cold start: pass through, initialise state.
            self._x_prev = x.copy()
            return x.copy()

        # --- derivative estimate (low-pass filtered velocity) -----------
        dx_raw = (x - self._x_prev) * self._freq
        alpha_d = self._alpha_from_cutoff(self._d_cutoff)
        self._dx_est = alpha_d * dx_raw + (1.0 - alpha_d) * self._dx_est

        # --- adaptive cutoff based on per-joint speed -------------------
        cutoff = self._min_cutoff + self._beta * np.abs(self._dx_est)
        alpha = self._alpha_from_cutoff(cutoff)  # may be scalar or array

        # --- low-pass filter on the signal ------------------------------
        x_hat = alpha * x + (1.0 - alpha) * self._x_prev
        self._x_prev = x_hat
        return x_hat.copy()

    def reset(self) -> None:
        """Clear filter state (use after a pause or discontinuity)."""
        self._x_prev = None
        self._dx_est = np.zeros(self._data_size, dtype=float)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _alpha_from_cutoff(self, cutoff: float | np.ndarray) -> float | np.ndarray:
        """Compute the EMA alpha from a cutoff frequency (Hz).

        Args:
            cutoff: Cutoff frequency (scalar or per-joint array).

        Returns:
            Alpha in ``(0, 1]`` (same shape as *cutoff*).
        """
        # τ = 1 / (2π f_c),  α = dt / (τ + dt)  =  1 / (1 + τ/dt)
        tau = 1.0 / (2.0 * np.pi * cutoff)
        dt = 1.0 / self._freq
        return 1.0 / (1.0 + tau / dt)


__all__ = ["OneEuroFilter"]
