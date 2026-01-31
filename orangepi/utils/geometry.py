"""
Coordinate transformations and geometry utilities.

Coordinate System (robot-relative, matching Vision.java):
- +X = front of robot
- +Y = left of robot
- Angles: 0 = directly ahead, positive = counterclockwise
"""

import math
from typing import Tuple

from config import (
    CAMERA_WIDTH,
    CAMERA_HEIGHT,
    CAMERA_FOV_HORIZONTAL,
    CAMERA_FOV_VERTICAL,
    CAMERA_MOUNT_HEIGHT,
    CAMERA_MOUNT_PITCH,
)


def pixel_to_angle(
    pixel_x: float,
    pixel_y: float,
    image_width: int = CAMERA_WIDTH,
    image_height: int = CAMERA_HEIGHT,
    fov_horizontal: float = CAMERA_FOV_HORIZONTAL,
    fov_vertical: float = CAMERA_FOV_VERTICAL,
) -> Tuple[float, float]:
    """
    Convert pixel coordinates to angular offset from camera center.

    Args:
        pixel_x: X coordinate in pixels (0 = left edge)
        pixel_y: Y coordinate in pixels (0 = top edge)
        image_width: Image width in pixels
        image_height: Image height in pixels
        fov_horizontal: Horizontal field of view in degrees
        fov_vertical: Vertical field of view in degrees

    Returns:
        Tuple of (horizontal_angle, vertical_angle) in degrees
        - horizontal_angle: positive = left of center
        - vertical_angle: positive = above center
    """
    # Normalize to [-0.5, 0.5] range (center = 0)
    norm_x = (pixel_x / image_width) - 0.5
    norm_y = (pixel_y / image_height) - 0.5

    # Convert to angles
    # Negate horizontal because positive X pixel is right, but positive angle is left
    horizontal_angle = -norm_x * fov_horizontal
    # Negate vertical because positive Y pixel is down, but positive angle is up
    vertical_angle = -norm_y * fov_vertical

    return horizontal_angle, vertical_angle


def estimate_distance(
    bbox_height_pixels: float,
    known_height_meters: float,
    image_height: int = CAMERA_HEIGHT,
    fov_vertical: float = CAMERA_FOV_VERTICAL,
) -> float:
    """
    Estimate distance to object using its apparent size.

    Uses pinhole camera model: distance = (known_size * image_size) / (apparent_size * 2 * tan(fov/2))

    Args:
        bbox_height_pixels: Height of bounding box in pixels
        known_height_meters: Known real-world height of object in meters
        image_height: Image height in pixels
        fov_vertical: Vertical field of view in degrees

    Returns:
        Estimated distance in meters
    """
    if bbox_height_pixels <= 0:
        return float("inf")

    fov_rad = math.radians(fov_vertical)
    focal_length_pixels = image_height / (2 * math.tan(fov_rad / 2))

    distance = (known_height_meters * focal_length_pixels) / bbox_height_pixels
    return distance


def pixel_to_robot_coords(
    pixel_x: float,
    pixel_y: float,
    distance: float,
    image_width: int = CAMERA_WIDTH,
    image_height: int = CAMERA_HEIGHT,
) -> Tuple[float, float, float]:
    """
    Convert pixel position and distance to robot-relative coordinates.

    Args:
        pixel_x: X coordinate in pixels (center of detection)
        pixel_y: Y coordinate in pixels (center of detection)
        distance: Estimated distance to object in meters
        image_width: Image width in pixels
        image_height: Image height in pixels

    Returns:
        Tuple of (x, y, angle) in robot coordinates
        - x: distance forward (positive = in front of robot)
        - y: distance sideways (positive = left of robot)
        - angle: angle from robot forward direction in degrees (positive = counterclockwise)
    """
    horizontal_angle, _ = pixel_to_angle(
        pixel_x, pixel_y, image_width, image_height
    )

    angle_rad = math.radians(horizontal_angle)

    # Convert to robot-relative coordinates
    # X is forward, Y is left
    x = distance * math.cos(angle_rad)
    y = distance * math.sin(angle_rad)

    return x, y, horizontal_angle


def calculate_velocity(
    prev_x: float,
    prev_y: float,
    curr_x: float,
    curr_y: float,
    dt: float,
) -> Tuple[float, float]:
    """
    Calculate velocity from two position measurements.

    Args:
        prev_x: Previous X position
        prev_y: Previous Y position
        curr_x: Current X position
        curr_y: Current Y position
        dt: Time difference in seconds

    Returns:
        Tuple of (vel_x, vel_y) in meters per second
    """
    if dt <= 0:
        return 0.0, 0.0

    vel_x = (curr_x - prev_x) / dt
    vel_y = (curr_y - prev_y) / dt

    return vel_x, vel_y


def get_bbox_center(
    x1: float, y1: float, x2: float, y2: float
) -> Tuple[float, float]:
    """
    Get center point of a bounding box.

    Args:
        x1, y1: Top-left corner
        x2, y2: Bottom-right corner

    Returns:
        Tuple of (center_x, center_y)
    """
    return (x1 + x2) / 2, (y1 + y2) / 2


def get_bbox_dimensions(
    x1: float, y1: float, x2: float, y2: float
) -> Tuple[float, float]:
    """
    Get width and height of a bounding box.

    Args:
        x1, y1: Top-left corner
        x2, y2: Bottom-right corner

    Returns:
        Tuple of (width, height)
    """
    return abs(x2 - x1), abs(y2 - y1)
