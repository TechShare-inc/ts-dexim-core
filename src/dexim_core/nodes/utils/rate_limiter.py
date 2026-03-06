"""
Rate limiter for precise control loop timing.

This module provides a RateLimiter class for maintaining precise control loop
rates with drift correction and statistics tracking.

Author: Haoyan Li
Date: October 24, 2025
"""

import time
from collections import deque
from typing import Dict

from loguru import logger


class RateLimiter:
    """Precise rate limiter for control loops.

    Features:
    - Target rate enforcement with drift correction
    - Jitter tracking and statistics
    - Overtime detection
    - CPU-efficient sleeping with high-precision busy-wait

    The rate limiter maintains precise timing by:
    1. Using high-resolution performance counter
    2. Targeting absolute future time (not relative delays) to prevent drift
    3. Sleeping for most of the period, then busy-waiting for last 1ms

    Example:
        limiter = RateLimiter(rate_hz=30.0)

        while running:
            # Do work
            process_data()
            send_command()

            # Sleep to maintain rate
            timing = limiter.sleep()

            # Check for overtime
            if timing['overtime']:
                logger.warning("Loop took too long!")

            # Check statistics every 100 iterations
            if limiter.iterations % 100 == 0:
                stats = limiter.get_statistics()
                print(f"Actual rate: {stats['actual_rate']:.2f}Hz")
                print(f"Mean jitter: {stats['mean_jitter_ms']:.3f}ms")
    """

    def __init__(self, rate_hz: float):
        """Initialize rate limiter.

        Args:
            rate_hz: Target control rate in Hz (e.g., 30.0 for 30Hz)
        """
        self.rate_hz = rate_hz
        self.target_period = 1.0 / rate_hz

        # Timing state
        self.start_time = time.perf_counter()
        self.last_time = self.start_time
        self.target_time = self.start_time + self.target_period

        # Statistics
        self.iterations = 0
        self.overtime_count = 0
        self.total_sleep_time = 0.0
        self.total_elapsed = 0.0

        # Rolling window for jitter (last 100 samples)
        self.jitter_samples = deque(maxlen=100)

        logger.debug(
            f"RateLimiter initialized: {rate_hz}Hz ({self.target_period*1000:.2f}ms period)"
        )

    def sleep(self) -> Dict[str, float]:
        """Sleep to maintain target rate.

        This method:
        1. Calculates time until next target time
        2. Sleeps for most of the period (leaving 1ms buffer)
        3. Busy-waits for remaining time for precision
        4. Updates timing statistics

        Returns:
            Dict with timing information:
            - 'elapsed': Time since last sleep (seconds)
            - 'sleep_time': Time slept (seconds)
            - 'overtime': True if loop took longer than period
            - 'jitter': Deviation from target period (seconds)
        """
        current_time = time.perf_counter()
        elapsed = current_time - self.last_time

        # Calculate sleep time to hit target_time (drift correction)
        sleep_time = self.target_time - current_time

        # Check for overtime
        overtime = sleep_time < 0
        jitter = elapsed - self.target_period

        if sleep_time > 0:
            # Sleep for most of the time, leaving 1ms for busy-wait
            # This prevents OS scheduler jitter while maintaining precision
            if sleep_time > 0.001:
                time.sleep(sleep_time - 0.001)
                self.total_sleep_time += sleep_time - 0.001

            # Busy-wait for remaining time (high precision)
            while time.perf_counter() < self.target_time:
                pass

        # Update timing state
        actual_time = time.perf_counter()
        self.last_time = actual_time
        self.target_time += self.target_period  # Drift correction: target absolute time
        self.total_elapsed = actual_time - self.start_time

        # Track statistics
        self.iterations += 1
        if overtime:
            self.overtime_count += 1
        self.jitter_samples.append(abs(jitter))

        return {
            "elapsed": elapsed,
            "sleep_time": max(0, sleep_time),
            "overtime": overtime,
            "jitter": jitter,
        }

    def reset(self):
        """Reset timing statistics.

        Useful for restarting the rate limiter without creating a new instance.
        """
        current_time = time.perf_counter()
        self.start_time = current_time
        self.last_time = current_time
        self.target_time = current_time + self.target_period

        self.iterations = 0
        self.overtime_count = 0
        self.total_sleep_time = 0.0
        self.total_elapsed = 0.0
        self.jitter_samples.clear()

        logger.debug("RateLimiter reset")

    def get_statistics(self) -> Dict[str, float]:
        """Get timing statistics.

        Returns:
            Dict with statistics:
            - 'actual_rate': Measured loop rate (Hz)
            - 'target_rate': Target rate (Hz)
            - 'mean_jitter_ms': Mean absolute jitter (milliseconds)
            - 'max_jitter_ms': Maximum absolute jitter (milliseconds)
            - 'min_jitter_ms': Minimum absolute jitter (milliseconds)
            - 'overtime_count': Number of missed deadlines
            - 'overtime_percentage': Percentage of loops that were overtime
            - 'iterations': Total iterations
            - 'efficiency': Percentage of time spent doing work (not sleeping)
            - 'total_elapsed': Total elapsed time (seconds)
        """
        if self.iterations == 0:
            return {
                "actual_rate": 0.0,
                "target_rate": self.rate_hz,
                "mean_jitter_ms": 0.0,
                "max_jitter_ms": 0.0,
                "min_jitter_ms": 0.0,
                "overtime_count": 0,
                "overtime_percentage": 0.0,
                "iterations": 0,
                "efficiency": 0.0,
                "total_elapsed": 0.0,
            }

        # Calculate actual rate from total elapsed time
        actual_rate = (
            self.iterations / self.total_elapsed if self.total_elapsed > 0 else 0
        )

        # Jitter statistics
        jitter_list = list(self.jitter_samples)
        mean_jitter = sum(jitter_list) / len(jitter_list) if jitter_list else 0
        max_jitter = max(jitter_list) if jitter_list else 0
        min_jitter = min(jitter_list) if jitter_list else 0

        # Efficiency (time not sleeping)
        efficiency = (
            (1.0 - (self.total_sleep_time / self.total_elapsed)) * 100
            if self.total_elapsed > 0
            else 0
        )

        # Overtime percentage
        overtime_percentage = (self.overtime_count / self.iterations) * 100

        return {
            "actual_rate": actual_rate,
            "target_rate": self.rate_hz,
            "mean_jitter_ms": mean_jitter * 1000,
            "max_jitter_ms": max_jitter * 1000,
            "min_jitter_ms": min_jitter * 1000,
            "overtime_count": self.overtime_count,
            "overtime_percentage": overtime_percentage,
            "iterations": self.iterations,
            "efficiency": efficiency,
            "total_elapsed": self.total_elapsed,
        }

    def __repr__(self) -> str:
        """String representation of rate limiter."""
        stats = self.get_statistics()
        return (
            f"RateLimiter(rate={self.rate_hz}Hz, "
            f"actual={stats['actual_rate']:.2f}Hz, "
            f"iterations={self.iterations})"
        )
