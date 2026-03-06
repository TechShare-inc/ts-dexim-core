"""
Helper functions for URDF content processing.

This module contains utility functions for URL replacement and content normalization
in URDF files.
"""
from __future__ import annotations


import os
import re


def replace_package_url_in_content(
    content: str, package_dir: str, description_name: str | None = None
) -> str:
    """
    Normalize resource URLs in URDF content.

    General patterns: "package://", "file://", or nothing.
    - If it's already "file://": do not modify.
    - Otherwise ("package://" or bare path): convert to "file://" with absolute path.

    Args:
        content: URDF content as string.
        package_dir: Base directory for resolving package/bare paths.
        description_name: Name of the robot description package/folder

    Returns:
        Modified content with normalized file URLs.
    """
from __future__ import annotations

    abs_package_dir = os.path.abspath(package_dir)

    # 1) Convert package://pkg/... to file://<abs>
    def _replace_package_url(match):
        package_name = match.group(1)
        relative_path = match.group(2)
        clean_relative_path = relative_path.lstrip("/")
        full_path = os.path.join(abs_package_dir, package_name, clean_relative_path)
        full_path = full_path.replace("\\", "/")
        return f"file://{full_path}"

    content = re.sub(r'package://([^/]+)(/[^"\s>]*)', _replace_package_url, content)

    # 2) Convert bare paths in common URL-bearing attributes to file://<abs>
    url_attr_pattern = re.compile(
        r"(filename|texture|uri)=([\"\'])"  # attribute and opening quote
        r"(?!file://)(?!package://)"  # not already a respected scheme
        r"([^\"\']+?)"  # path up to closing quote
        r"\2"  # same closing quote
    )

    def _replace_bare_attr(match: re.Match) -> str:
        attr = match.group(1)
        quote = match.group(2)
        rel_path = match.group(3).lstrip("/")
        base_dir = (
            os.path.join(abs_package_dir, description_name)
            if description_name
            else abs_package_dir
        )
        abs_path = os.path.join(base_dir, rel_path).replace("\\", "/")
        return f"{attr}={quote}file://{abs_path}{quote}"

    content = url_attr_pattern.sub(_replace_bare_attr, content)

    return content


__all__ = ["replace_package_url_in_content"]
