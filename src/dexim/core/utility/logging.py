"""Logging configuration using loguru."""
from __future__ import annotations


from loguru import logger
import sys
from pathlib import Path


def setup_logger(log_dir: str = "logs", log_level: str = "INFO"):
    """Configure loguru logger with file rotation and formatting.

    Args:
        log_dir: Directory for log files
        log_level: Logging level (DEBUG, INFO, WARNING, ERROR)

    Returns:
        logger: Configured loguru logger
    """
from __future__ import annotations

    # Remove default handler
    logger.remove()

    # Add console handler with colors
    logger.add(
        sys.stderr,
        format="<green>{time:YYYY-MM-DD HH:mm:ss}</green> | <level>{level: <8}</level> | <cyan>{name}</cyan>:<cyan>{function}</cyan>:<cyan>{line}</cyan> - <level>{message}</level>",
        level=log_level,
        colorize=True,
    )

    # Add file handler with rotation
    log_path = Path(log_dir)
    log_path.mkdir(exist_ok=True, parents=True)

    logger.add(
        log_path / "teleop_{time:YYYY-MM-DD}.log",
        rotation="00:00",  # Rotate at midnight
        retention="7 days",  # Keep logs for 7 days
        format="{time:YYYY-MM-DD HH:mm:ss} | {level: <8} | {name}:{function}:{line} - {message}",
        level=log_level,
    )

    logger.info(f"Logger initialized with level={log_level}, log_dir={log_dir}")

    return logger
