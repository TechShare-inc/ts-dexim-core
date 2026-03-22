"""JointFilter — pluggable per-joint smoothing."""

from __future__ import annotations

import numpy as np
from dexim.core.config import FilterConfig
from dexim.core.utility import (
    ExponentialMovingFilter,
    OneEuroFilter,
    WeightedMovingFilter,
)
from loguru import logger


class JointFilter:
    """Applies optional per-joint smoothing to joint-angle arrays.

    Supports ``"wma"``, ``"ema"``, and ``"one_euro"`` filter types.
    Pass ``filter_config=None`` to disable filtering (identity transform).

    Args:
        filter_config: Filter configuration, or None to disable.
        data_size: Number of joints (determines filter width).
    """

    def __init__(
        self,
        filter_config: FilterConfig | None,
        data_size: int,
    ) -> None:
        self._filter: (
            WeightedMovingFilter | ExponentialMovingFilter | OneEuroFilter | None
        ) = None

        if filter_config is None:
            logger.info("Smoothing filter disabled")
            return

        ftype = filter_config.type

        if ftype == "wma":
            weights = np.array(filter_config.weights)
            self._filter = WeightedMovingFilter(weights=weights, data_size=data_size)
            logger.info(f"Smoothing filter: wma, weights={weights.tolist()}")

        elif ftype == "ema":
            self._filter = ExponentialMovingFilter(
                alpha=filter_config.alpha, data_size=data_size
            )
            logger.info(f"Smoothing filter: ema, alpha={filter_config.alpha}")

        elif ftype == "one_euro":
            self._filter = OneEuroFilter(
                freq=filter_config.freq,
                min_cutoff=filter_config.min_cutoff,
                beta=filter_config.beta,
                d_cutoff=filter_config.d_cutoff,
                data_size=data_size,
            )
            logger.info(
                f"Smoothing filter: one_euro, freq={filter_config.freq}Hz, "
                f"min_cutoff={filter_config.min_cutoff}Hz, beta={filter_config.beta}"
            )

        else:
            raise ValueError(f"Unknown filter type: {ftype!r}")

    def __call__(self, q: np.ndarray) -> np.ndarray:
        """Apply smoothing filter to joint angles.

        Args:
            q: Raw joint angles.

        Returns:
            Filtered joint angles (identity if filter is disabled).
        """
        if self._filter is None:
            return q
        return self._filter.filter(q)
