"""
FUEL (yellow ball) detection using YOLOv8 on RKNN.
"""

from typing import List, Optional, Tuple
import numpy as np

from .base_detector import BaseDetector, Detection, nms
from config import (
    FUEL_CONFIDENCE_THRESHOLD,
    FUEL_MODEL_PATH,
    FUEL_DIAMETER,
    INTAKE_RANGE_DISTANCE,
)
from utils.geometry import (
    pixel_to_robot_coords,
    estimate_distance,
    get_bbox_center,
    get_bbox_dimensions,
)


class FuelDetector(BaseDetector):
    """Detector for FUEL game pieces (yellow balls)."""

    # Class names for FUEL detection model
    CLASS_NAMES = ["fuel"]

    def __init__(
        self,
        model_path: str = FUEL_MODEL_PATH,
        confidence_threshold: float = FUEL_CONFIDENCE_THRESHOLD,
    ):
        super().__init__(
            model_path=model_path,
            confidence_threshold=confidence_threshold,
        )

    def postprocess(
        self,
        outputs: List[np.ndarray],
        original_size: Tuple[int, int],
    ) -> List[Detection]:
        """
        Post-process YOLOv8 outputs for FUEL detection.

        Args:
            outputs: Raw model outputs
            original_size: Original frame size (width, height)

        Returns:
            List of Detection objects with robot-relative coordinates
        """
        if outputs is None or len(outputs) == 0:
            return []

        # YOLOv8 output format
        output = outputs[0]
        if len(output.shape) == 3:
            output = output[0]
        if output.shape[0] < output.shape[1]:
            output = output.T

        boxes = []
        scores = []
        class_ids = []

        scale_x = original_size[0] / self.input_size[0]
        scale_y = original_size[1] / self.input_size[1]

        for row in output:
            x_center, y_center, w, h = row[:4]
            class_scores = row[4:]

            max_score = np.max(class_scores)
            class_id = np.argmax(class_scores)

            if max_score >= self.confidence_threshold:
                x1 = (x_center - w / 2) * scale_x
                y1 = (y_center - h / 2) * scale_y
                x2 = (x_center + w / 2) * scale_x
                y2 = (y_center + h / 2) * scale_y

                boxes.append([x1, y1, x2, y2])
                scores.append(max_score)
                class_ids.append(class_id)

        if len(boxes) == 0:
            return []

        # Apply NMS
        boxes = np.array(boxes)
        scores = np.array(scores)
        keep = nms(boxes, scores, self.nms_threshold)

        detections = []

        for idx in keep:
            bbox = tuple(boxes[idx])
            center_x, center_y = get_bbox_center(*bbox)
            _, bbox_height = get_bbox_dimensions(*bbox)

            # Estimate distance from bounding box size (using known FUEL diameter)
            distance = estimate_distance(
                bbox_height,
                FUEL_DIAMETER,
                original_size[1],
            )

            # Convert to robot-relative coordinates
            x, y, angle = pixel_to_robot_coords(
                center_x,
                center_y,
                distance,
                original_size[0],
                original_size[1],
            )

            class_id = int(class_ids[idx])
            class_name = (
                self.CLASS_NAMES[class_id]
                if class_id < len(self.CLASS_NAMES)
                else "unknown"
            )

            detection = Detection(
                class_id=class_id,
                class_name=class_name,
                confidence=float(scores[idx]),
                bbox=bbox,
                x=x,
                y=y,
                distance=distance,
                angle=angle,
            )
            detections.append(detection)

        return detections

    def get_best_fuel(self, detections: List[Detection]) -> Optional[Detection]:
        """
        Get the best FUEL target for intake.

        Prioritizes FUEL that is:
        1. In intake range (close and centered)
        2. Closest if none in intake range

        Args:
            detections: List of FUEL detections

        Returns:
            Best FUEL detection or None
        """
        if not detections:
            return None

        # Find FUEL in intake range (close and roughly centered)
        intake_ready = [
            d for d in detections
            if self.is_intake_ready(d)
        ]

        if intake_ready:
            # Return the one with highest confidence in intake range
            return max(intake_ready, key=lambda d: d.confidence)

        # Otherwise return closest
        return min(detections, key=lambda d: d.distance)

    def is_in_intake_range(self, detection: Detection) -> bool:
        """
        Check if a FUEL is within intake range.

        Args:
            detection: FUEL detection

        Returns:
            True if within range
        """
        return detection.distance <= INTAKE_RANGE_DISTANCE

    def is_intake_ready(self, detection: Detection) -> bool:
        """
        Check if a FUEL is ready for intake (close and centered).

        Args:
            detection: FUEL detection

        Returns:
            True if ready for intake
        """
        # Must be in range
        if not self.is_in_intake_range(detection):
            return False

        # Must be roughly centered (within 15 degrees of center)
        if abs(detection.angle) > 15.0:
            return False

        return True

    def has_fuel_targets(self, detections: List[Detection]) -> bool:
        """
        Check if any FUEL is detected.

        Args:
            detections: List of FUEL detections

        Returns:
            True if any FUEL detected
        """
        return len(detections) > 0
