"""PipelineProfiler — per-stage timing accumulation and periodic reporting."""

from __future__ import annotations

import contextlib
import time
from collections.abc import Generator

from dexim.core.nodes.utils import RateLimiter
from loguru import logger


class PipelineProfiler:
    """Accumulates worst-case stage durations and logs periodic stats.

    Usage::

        profiler = PipelineProfiler()
        profiler.record_stages({"recv": (t0, t1), "extract": (t1, t2)})
        profiler.report_if_due(node_id, rate_limiter, data_age_sec, every_n=200)
    """

    def __init__(self) -> None:
        self._stage_durations_max: dict[str, float] = {}

    @contextlib.contextmanager
    def stage(self, name: str) -> Generator[None, None, None]:
        """Context manager that records the duration of a pipeline stage.

        Updates the running worst-case duration for *name* and stores it
        for the next :meth:`report_if_due` call.

        Args:
            name: Stage identifier (e.g. ``"recv"``, ``"extract"``).
        """
        t0 = time.perf_counter()
        try:
            yield
        finally:
            dt = time.perf_counter() - t0
            self._stage_durations_max[name] = max(
                self._stage_durations_max.get(name, 0.0), dt
            )

    def record_stages(self, stages: dict[str, tuple[float, float]]) -> None:
        """Record max durations for multiple pipeline stages and log them.

        Updates the running worst-case per stage and emits a single DEBUG
        line with all stage timings for this iteration.

        Args:
            stages: Mapping of stage name → (t_start, t_end) from
                ``time.perf_counter()``.
        """
        parts: list[str] = []
        for name, (t0, t1) in stages.items():
            dt = t1 - t0
            self._stage_durations_max[name] = max(
                self._stage_durations_max.get(name, 0.0), dt
            )
            parts.append(f"{name}={dt * 1000:.2f}ms")
        logger.debug("[pipeline] " + "  ".join(parts))

    def report_if_due(
        self,
        node_id: str,
        rate_limiter: RateLimiter,
        data_age_sec: float | None = None,
        every_n: int = 200,
    ) -> None:
        """Log a one-line DEBUG summary of control-loop health if due.

        Resets the worst-case stage durations after logging so each window
        reflects only the interval since the last report.

        Args:
            node_id: Node identifier for the log prefix.
            rate_limiter: RateLimiter instance.
            data_age_sec: Age of the last received data frame, or None.
            every_n: Log every N iterations.
        """
        iterations = rate_limiter.iterations
        if iterations % every_n != 0:
            return

        stats = rate_limiter.get_statistics()
        data_age_str = (
            f"{data_age_sec * 1000:.1f}ms" if data_age_sec is not None else "n/a"
        )
        stage_parts = ", ".join(
            f"{k}={v * 1000:.1f}ms"
            for k, v in sorted(self._stage_durations_max.items())
        )
        stage_str = stage_parts if stage_parts else "n/a"
        logger.debug(
            f"[{node_id}/{iterations}] "
            f"rate={stats['actual_rate']:.1f}/{stats['target_rate']:.0f}Hz  "
            f"jitter={stats['mean_jitter_ms']:.2f}ms  "
            f"overtime={stats['overtime_percentage']:.1f}%  "
            f"data_age={data_age_str}  "
            f"stage_max=[{stage_str}]"
        )
        self._stage_durations_max.clear()
