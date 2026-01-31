"""
Configuration constants for Orange Pi vision system.
Matches NetworkTables keys defined in Vision.java.
"""

# NetworkTables Configuration
NT_SERVER_IP = "10.31.64.2"  # roboRIO IP address
NT_TABLE_NAME = "Vision"

# Camera Configuration
CAMERA_ID = 0  # USB camera index
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30

# Detection Thresholds
ROBOT_CONFIDENCE_THRESHOLD = 0.5
FUEL_CONFIDENCE_THRESHOLD = 0.4
INTAKE_RANGE_DISTANCE = 0.5  # meters - distance considered "in intake range"

# Model Paths
ROBOT_MODEL_PATH = "models/robot_detector.rknn"
FUEL_MODEL_PATH = "models/fuel_detector.rknn"

# Camera Calibration (placeholder values - calibrate for actual camera)
CAMERA_FOV_HORIZONTAL = 68.0  # degrees
CAMERA_FOV_VERTICAL = 41.0  # degrees
CAMERA_MOUNT_HEIGHT = 0.3  # meters above ground
CAMERA_MOUNT_PITCH = 0.0  # degrees (0 = level, positive = tilted down)

# Known object sizes for distance estimation (meters)
ROBOT_WIDTH_ESTIMATE = 0.7  # approximate robot width
FUEL_DIAMETER = 0.178  # 7 inch FUEL ball diameter


class NTKeys:
    """NetworkTables keys matching Vision.java contract."""

    # Robot detection keys (Vision/Robots/)
    class Robots:
        TABLE = "Vision/Robots"
        COUNT = "robotCount"
        CLOSEST_X = "closestRobotX"
        CLOSEST_Y = "closestRobotY"
        CLOSEST_DISTANCE = "closestRobotDistance"
        CLOSEST_VEL_X = "closestRobotVelX"
        CLOSEST_VEL_Y = "closestRobotVelY"
        CLOSEST_CONFIDENCE = "closestConfidence"
        TIMESTAMP = "timestamp"

    # FUEL detection keys (Vision/Fuel/)
    class Fuel:
        TABLE = "Vision/Fuel"
        COUNT = "fuelCount"
        HAS_FUEL = "hasFuel"
        BEST_X = "bestFuelX"
        BEST_Y = "bestFuelY"
        BEST_DISTANCE = "bestFuelDistance"
        BEST_ANGLE = "bestFuelAngle"
        BEST_CONFIDENCE = "bestFuelConfidence"
        IN_INTAKE_RANGE = "bestFuelInIntakeRange"
        INTAKE_READY = "intakeReady"
        TIMESTAMP = "timestamp"
