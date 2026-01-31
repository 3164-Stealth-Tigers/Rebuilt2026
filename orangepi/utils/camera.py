"""
Camera capture utilities for USB cameras with OpenCV.
"""

import cv2
import numpy as np
import threading
import time
from typing import Optional, Tuple

from config import CAMERA_ID, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS


class Camera:
    """USB camera capture with frame buffering for consistent FPS."""

    def __init__(
        self,
        camera_id: int = CAMERA_ID,
        width: int = CAMERA_WIDTH,
        height: int = CAMERA_HEIGHT,
        fps: int = CAMERA_FPS,
    ):
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.fps = fps

        self._cap: Optional[cv2.VideoCapture] = None
        self._frame: Optional[np.ndarray] = None
        self._frame_time: float = 0.0
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self) -> bool:
        """Start camera capture in a background thread."""
        self._cap = cv2.VideoCapture(self.camera_id)

        if not self._cap.isOpened():
            print(f"Failed to open camera {self.camera_id}")
            return False

        # Configure camera
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap.set(cv2.CAP_PROP_FPS, self.fps)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize latency

        # Verify settings
        actual_width = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self._cap.get(cv2.CAP_PROP_FPS)

        print(f"Camera initialized: {actual_width}x{actual_height} @ {actual_fps} FPS")

        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

        return True

    def _capture_loop(self):
        """Background thread for continuous frame capture."""
        while self._running:
            ret, frame = self._cap.read()
            if ret:
                with self._lock:
                    self._frame = frame
                    self._frame_time = time.time()
            else:
                time.sleep(0.001)  # Brief sleep on failed read

    def get_frame(self) -> Tuple[Optional[np.ndarray], float]:
        """
        Get the latest captured frame.

        Returns:
            Tuple of (frame, timestamp) or (None, 0.0) if no frame available
        """
        with self._lock:
            if self._frame is not None:
                return self._frame.copy(), self._frame_time
            return None, 0.0

    def stop(self):
        """Stop camera capture and release resources."""
        self._running = False

        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

        if self._cap is not None:
            self._cap.release()
            self._cap = None

        print("Camera stopped")

    @property
    def is_running(self) -> bool:
        """Check if camera is currently capturing."""
        return self._running and self._cap is not None and self._cap.isOpened()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()
        return False
