"""
Weighted moving average filter for smoothing joint trajectories.

The filter keeps a fixed-size history window and applies exponentially
decaying weights (default: [0.4, 0.3, 0.2, 0.1]) to produce smoothed
joint configurations. Duplicate inputs are ignored and the most recent
sample is returned while the history is still warming up.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Iterable

import numpy as np
import numpy.typing as npt


class WeightedMovingFilter:
    """Apply an exponentially weighted moving average to vector data."""

    def __init__(
        self,
        weights: Iterable[float],
        data_size: int,
    ) -> None:
        weight_array = np.asarray(list(weights), dtype=float)
        if weight_array.ndim != 1:
            raise ValueError("weights must be a one-dimensional iterable")
        if weight_array.size == 0:
            raise ValueError("weights must contain at least one element")
        if not np.isclose(weight_array.sum(), 1.0):
            raise AssertionError(
                "[WeightedMovingFilter] the sum of weights list must be 1.0!"
            )

        self._weights = weight_array
        self._window_size = weight_array.size
        self._data_size = int(data_size)
        if self._data_size <= 0:
            raise ValueError("data_size must be positive")

        self._data_queue: deque[np.ndarray] = deque(maxlen=self._window_size)
        self._filtered_data = np.zeros(self._data_size, dtype=float)

    def reset(self) -> None:
        """Clear history and reset the filtered output to zeros."""
        self._data_queue.clear()
        self._filtered_data.fill(0.0)

    def add_data(self, new_data: npt.ArrayLike) -> None:
        """
        Insert a new sample into the history and update the filtered result.

        Duplicate samples (identical to the most recent entry) are skipped to
        avoid adding redundant data.
        """
        new_array = np.asarray(new_data, dtype=float)
        if new_array.shape != (self._data_size,):
            raise ValueError(
                f"Expected data shape ({self._data_size},), got {new_array.shape}"
            )

        if self._data_queue and np.array_equal(new_array, self._data_queue[-1]):
            return

        self._data_queue.append(new_array)
        self._filtered_data = self._apply_filter()

    def filter(self, data: npt.ArrayLike) -> np.ndarray:
        """Add a sample and return the filtered result.

        Convenience alias for ``add_data`` + ``filtered_data`` so all filter
        types share the same single-call interface.

        Args:
            data: New sample vector of shape ``(data_size,)``.

        Returns:
            Filtered vector of shape ``(data_size,)``.
        """
        self.add_data(data)
        return self.filtered_data

    @property
    def filtered_data(self) -> np.ndarray:
        """Return the most recent filtered sample."""
        return self._filtered_data.copy()

    def _apply_filter(self) -> np.ndarray:
        """Apply the weighted moving average to the current history."""
        if len(self._data_queue) == 0:
            return np.zeros(self._data_size, dtype=float)

        if len(self._data_queue) < self._window_size:
            return self._data_queue[-1].copy()

        data_array = np.vstack(self._data_queue)
        # Apply convolution along the time axis for each joint.
        convolved = np.apply_along_axis(
            lambda column: np.convolve(column, self._weights, mode="valid")[-1],
            axis=0,
            arr=data_array,
        )
        return convolved.astype(float, copy=False)


__all__ = ["WeightedMovingFilter"]
