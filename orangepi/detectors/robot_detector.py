"""
Robot detection using YOLOv8 on RKNN.
"""

import time
from typing import Dict, List, Optional, Tuple
import numpy as np

from .base_detector import BaseDetector, Detection, nms
from config import ROBOT_CONFIDENCE_THRESHOLD, ROBOT_MODEL_PATH, ROBOT_WIDTH_ESTIMATE
from utils.geometry import (
    pixel_to_robot_coords,
    estimate_distance,
    get_bbox_center,
    get_bbox_dimensions,
    calculate_velocity,
)


class RobotDetector(BaseDetector):
    """Detector for opposing robots on the field."""

    # Class names for robot detection model
    CLASS_NAMES = ["robot"]

    def __init__(
        self,
        model_path: str = ROBOT_MODEL_PATH,
        confidence_threshold: float = ROBOT_CONFIDENCE_THRESHOLD,
    ):
        super().__init__(
            model_path=model_path,
            confidence_threshold=confidence_threshold,
        )

        # Tracking state for velocity estimation
        self._prev_detections: Dict[int, Tuple[float, float, float]] = {}
        self._prev_time: float = 0.0
        self._track_id_counter = 0

    def postprocess(
        self,
        outputs: List[np.ndarray],
        original_size: Tuple[int, int],
    ) -> List[Detection]:
        """
        Post-process YOLOv8 outputs.

        Args:
            outputs: Raw model outputs
            original_size: Original frame size (width, height)

        Returns:
            List of Detection objects with robot-relative coordinates
        """
        if outputs is None or len(outputs) == 0:
            return []

        # YOLOv8 output format: [1, 84, 8400] or [1, 5+num_classes, num_boxes]
        # Transpose to [8400, 84] for easier processing
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
            # YOLOv8 format: [x_center, y_center, width, height, class_scores...]
            x_center, y_center, w, h = row[:4]
            class_scores = row[4:]

            max_score = np.max(class_scores)
            class_id = np.argmax(class_scores)

            if max_score >= self.confidence_threshold:
                # Convert center format to corner format
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
        current_time = time.time()
        dt = current_time - self._prev_time if self._prev_time > 0 else 0.0

        for idx in keep:
            bbox = tuple(boxes[idx])
            center_x, center_y = get_bbox_center(*bbox)
            _, bbox_height = get_bbox_dimensions(*bbox)

            # Estimate distance from bounding box height
            distance = estimate_distance(
                bbox_height,
                ROBOT_WIDTH_ESTIMATE,  # Using width as height estimate
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

            # Velocity estimation (simple nearest-neighbor tracking)
            vel_x, vel_y = self._estimate_velocity(x, y, dt)

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
                vel_x=vel_x,
                vel_y=vel_y,
            )
            detections.append(detection)

        # Update tracking state
        self._update_tracking(detections, current_time)

        return detections

    def _estimate_velocity(self, x: float, y: float, dt: float) -> Tuple[float, float]:
        """
        Estimate velocity by matching to previous detections.

        Uses simple nearest-neighbor matching for now.
        """
        if dt <= 0 or len(self._prev_detections) == 0:
            return 0.0, 0.0

        # Find closest previous detection
        min_dist = float("inf")
        closest_prev = None

        for track_id, (prev_x, prev_y, _) in self._prev_detections.items():
            dist = ((x - prev_x) ** 2 + (y - prev_y) ** 2) ** 0.5
            if dist < min_dist:
                min_dist = dist
                closest_prev = (prev_x, prev_y)

        # Only compute velocity if match is reasonably close
        if closest_prev is not None and min_dist < 2.0:  # 2 meter threshold
            return calculate_velocity(closest_prev[0], closest_prev[1], x, y, dt)

        return 0.0, 0.0

    def _update_tracking(self, detections: List[Detection], current_time: float):
        """Update tracking state with current detections."""
        self._prev_detections.clear()

        for i, det in enumerate(detections):
            self._prev_detections[i] = (det.x, det.y, current_time)

        self._prev_time = current_time

    def get_closest_robot(
        self, detections: List[Detection]
    ) -> Optional[Detection]:
        """
        Get the closest detected robot.

        Args:
            detections: List of robot detections

        Returns:
            Closest robot detection or None
        """
        if not detections:
            return None

        return min(detections, key=lambda d: d.distance)
