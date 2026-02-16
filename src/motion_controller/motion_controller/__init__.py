"""
motion_controller package

Provides:
- MotionController node for velocity limiting, smoothing and watchdog safety.
"""

from .node import MotionController

__all__ = ["MotionController"]
