#!/usr/bin/env python3
"""
Orange Pi Vision Service - Main Entry Point

ML inference service for detecting robots and FUEL game pieces.
Publishes results to NetworkTables for roboRIO consumption.
"""

import argparse
import signal
import sys
import time
from typing import Optional

from config import CAMERA_FPS
from detectors import RobotDetector, FuelDetector
from network import NTPublisher
from utils import Camera


class VisionService:
    """Main vision service orchestrating detection and publishing."""

    def __init__(self, simulate: bool = False):
        """
        Initialize the vision service.

        Args:
            simulate: If True, run without actual camera/models
        """
        self.simulate = simulate
        self._running = False

        # Components
        self.camera: Optional[Camera] = None
        self.robot_detector: Optional[RobotDetector] = None
        self.fuel_detector: Optional[FuelDetector] = None
        self.publisher: Optional[NTPublisher] = None

        # Statistics
        self._frame_count = 0
        self._start_time = 0.0
        self._last_fps_time = 0.0
        self._fps = 0.0

    def initialize(self) -> bool:
        """
        Initialize all components.

        Returns:
            True if all components initialized successfully
        """
        print("Initializing Vision Service...")

        # Initialize camera
        self.camera = Camera()
        if not self.simulate:
            if not self.camera.start():
                print("Failed to start camera")
                return False
        else:
            print("Camera simulation mode")

        # Initialize detectors
        self.robot_detector = RobotDetector()
        self.fuel_detector = FuelDetector()

        if not self.simulate:
            if not self.robot_detector.load():
                print("Warning: Robot detector failed to load")
            if not self.fuel_detector.load():
                print("Warning: FUEL detector failed to load")
        else:
            print("Detector simulation mode")

        # Initialize NetworkTables publisher
        self.publisher = NTPublisher()
        if not self.publisher.connect():
            print("Warning: NetworkTables connection failed")

        print("Vision Service initialized")
        return True

    def run(self):
        """Run the main inference loop."""
        print("Starting inference loop...")
        self._running = True
        self._start_time = time.time()
        self._last_fps_time = self._start_time

        target_frame_time = 1.0 / CAMERA_FPS

        while self._running:
            loop_start = time.time()

            self._process_frame()

            # Maintain target FPS
            elapsed = time.time() - loop_start
            sleep_time = target_frame_time - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

            # Update FPS counter
            self._frame_count += 1
            if time.time() - self._last_fps_time >= 1.0:
                self._fps = self._frame_count / (time.time() - self._last_fps_time)
                self._frame_count = 0
                self._last_fps_time = time.time()

                if self._frame_count == 0:  # Print every second
                    print(f"FPS: {self._fps:.1f}, NT Connected: {self.publisher.is_connected if self.publisher else False}")

    def _process_frame(self):
        """Process a single frame."""
        if self.simulate:
            self._process_simulated_frame()
            return

        # Get frame from camera
        frame, timestamp = self.camera.get_frame()
        if frame is None:
            return

        # Run robot detection
        robot_detections = []
        if self.robot_detector and self.robot_detector.is_loaded:
            robot_detections = self.robot_detector.detect(frame)

        closest_robot = None
        if robot_detections:
            closest_robot = self.robot_detector.get_closest_robot(robot_detections)

        # Run FUEL detection
        fuel_detections = []
        if self.fuel_detector and self.fuel_detector.is_loaded:
            fuel_detections = self.fuel_detector.detect(frame)

        best_fuel = None
        intake_ready = False
        if fuel_detections:
            best_fuel = self.fuel_detector.get_best_fuel(fuel_detections)
            if best_fuel:
                intake_ready = self.fuel_detector.is_intake_ready(best_fuel)

        # Publish to NetworkTables
        if self.publisher:
            self.publisher.publish_robots(robot_detections, closest_robot, timestamp)
            self.publisher.publish_fuel(fuel_detections, best_fuel, intake_ready, timestamp)

    def _process_simulated_frame(self):
        """Process a simulated frame for testing."""
        timestamp = time.time()

        # Publish empty data in simulation mode
        if self.publisher:
            self.publisher.publish_robots([], None, timestamp)
            self.publisher.publish_fuel([], None, False, timestamp)

    def stop(self):
        """Stop the vision service."""
        print("Stopping Vision Service...")
        self._running = False

        if self.camera:
            self.camera.stop()

        if self.robot_detector:
            self.robot_detector.release()

        if self.fuel_detector:
            self.fuel_detector.release()

        if self.publisher:
            self.publisher.disconnect()

        print("Vision Service stopped")

    def print_stats(self):
        """Print runtime statistics."""
        runtime = time.time() - self._start_time
        print(f"\n--- Vision Service Statistics ---")
        print(f"Runtime: {runtime:.1f} seconds")
        print(f"Average FPS: {self._fps:.1f}")
        print(f"NT Connected: {self.publisher.is_connected if self.publisher else False}")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description="Orange Pi Vision Service")
    parser.add_argument(
        "--simulate",
        action="store_true",
        help="Run in simulation mode (no camera/models)",
    )
    args = parser.parse_args()

    # Create service
    service = VisionService(simulate=args.simulate)

    # Handle shutdown signals
    def signal_handler(sig, frame):
        print("\nShutdown signal received")
        service.stop()
        service.print_stats()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Initialize and run
    if not service.initialize():
        print("Failed to initialize service")
        sys.exit(1)

    try:
        service.run()
    except Exception as e:
        print(f"Error in main loop: {e}")
        service.stop()
        sys.exit(1)


if __name__ == "__main__":
    main()
