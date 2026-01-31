"""
Abstract base class for RKNN-based object detectors.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Optional, Tuple
import numpy as np


@dataclass
class Detection:
    """Represents a single object detection."""

    class_id: int
    class_name: str
    confidence: float
    bbox: Tuple[float, float, float, float]  # x1, y1, x2, y2 in pixels

    # Robot-relative coordinates (populated after transformation)
    x: float = 0.0  # meters, positive = forward
    y: float = 0.0  # meters, positive = left
    distance: float = 0.0  # meters
    angle: float = 0.0  # degrees, positive = counterclockwise

    # Velocity (for tracked objects)
    vel_x: float = 0.0  # m/s
    vel_y: float = 0.0  # m/s


class BaseDetector(ABC):
    """Abstract base class for RKNN object detectors."""

    def __init__(
        self,
        model_path: str,
        confidence_threshold: float = 0.5,
        nms_threshold: float = 0.45,
        input_size: Tuple[int, int] = (640, 640),
    ):
        """
        Initialize the detector.

        Args:
            model_path: Path to the RKNN model file
            confidence_threshold: Minimum confidence for detections
            nms_threshold: NMS IoU threshold
            input_size: Model input size (width, height)
        """
        self.model_path = model_path
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.input_size = input_size

        self.rknn = None
        self._loaded = False

    def load(self) -> bool:
        """
        Load the RKNN model.

        Returns:
            True if model loaded successfully
        """
        try:
            from rknnlite.api import RKNNLite

            self.rknn = RKNNLite()

            ret = self.rknn.load_rknn(self.model_path)
            if ret != 0:
                print(f"Failed to load RKNN model: {self.model_path}")
                return False

            # Initialize runtime with all NPU cores
            ret = self.rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2)
            if ret != 0:
                print("Failed to initialize RKNN runtime")
                return False

            self._loaded = True
            print(f"Model loaded: {self.model_path}")
            return True

        except ImportError:
            print("RKNN Lite not available - running in simulation mode")
            self._loaded = True  # Allow simulation mode
            return True
        except Exception as e:
            print(f"Error loading model: {e}")
            return False

    def preprocess(self, frame: np.ndarray) -> np.ndarray:
        """
        Preprocess frame for inference.

        Args:
            frame: Input frame (BGR, HWC format)

        Returns:
            Preprocessed frame ready for inference
        """
        import cv2

        # Resize to model input size
        resized = cv2.resize(frame, self.input_size)

        # Convert BGR to RGB
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)

        return rgb

    def infer(self, frame: np.ndarray) -> Optional[List[np.ndarray]]:
        """
        Run inference on a preprocessed frame.

        Args:
            frame: Preprocessed frame

        Returns:
            Raw model outputs or None if inference failed
        """
        if not self._loaded or self.rknn is None:
            return None

        try:
            outputs = self.rknn.inference(inputs=[frame])
            return outputs
        except Exception as e:
            print(f"Inference error: {e}")
            return None

    @abstractmethod
    def postprocess(
        self,
        outputs: List[np.ndarray],
        original_size: Tuple[int, int],
    ) -> List[Detection]:
        """
        Post-process model outputs to extract detections.

        Args:
            outputs: Raw model outputs
            original_size: Original frame size (width, height)

        Returns:
            List of Detection objects
        """
        pass

    def detect(self, frame: np.ndarray) -> List[Detection]:
        """
        Run full detection pipeline on a frame.

        Args:
            frame: Input frame (BGR, HWC format)

        Returns:
            List of Detection objects
        """
        if not self._loaded:
            return []

        original_size = (frame.shape[1], frame.shape[0])

        preprocessed = self.preprocess(frame)
        outputs = self.infer(preprocessed)

        if outputs is None:
            return []

        detections = self.postprocess(outputs, original_size)
        return detections

    def release(self):
        """Release model resources."""
        if self.rknn is not None:
            try:
                self.rknn.release()
            except Exception:
                pass
            self.rknn = None
        self._loaded = False

    @property
    def is_loaded(self) -> bool:
        """Check if model is loaded."""
        return self._loaded

    def __enter__(self):
        self.load()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.release()
        return False


def nms(
    boxes: np.ndarray,
    scores: np.ndarray,
    iou_threshold: float,
) -> List[int]:
    """
    Non-maximum suppression.

    Args:
        boxes: Array of bounding boxes [N, 4] as (x1, y1, x2, y2)
        scores: Array of confidence scores [N]
        iou_threshold: IoU threshold for suppression

    Returns:
        List of indices to keep
    """
    if len(boxes) == 0:
        return []

    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]

    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)

        if order.size == 1:
            break

        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w = np.maximum(0, xx2 - xx1)
        h = np.maximum(0, yy2 - yy1)
        intersection = w * h

        iou = intersection / (areas[i] + areas[order[1:]] - intersection)

        mask = iou <= iou_threshold
        order = order[1:][mask]

    return keep
