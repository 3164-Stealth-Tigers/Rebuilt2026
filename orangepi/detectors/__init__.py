"""Detection modules for Orange Pi vision system."""

from .base_detector import BaseDetector, Detection
from .robot_detector import RobotDetector
from .fuel_detector import FuelDetector

__all__ = ["BaseDetector", "Detection", "RobotDetector", "FuelDetector"]
