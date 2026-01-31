package frc.robot;
//michael was here
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Constants {
    public static final class ShooterConstants {
        public static final Alliance DEFAULT_ALLIANCE = Alliance.Blue; // Default alliance

        public static final double BLUE_X = 11.915394;
        public static final double RED_X = 4.625594;
        public static final double UNIV_Y = 4.034663;

        public static final double HUB_RIM_HEIGHT = 1.8288;

        public static final double LAUNCH_ANGLE = 60.0; // degrees (placeholder TODO)
        public static final double G_ACCEL = 9.8067; // acceleration due to gravity (g) m/s^2
        public static final double Z_OFFSET = 0.5; // meters (placeholder TODO)

        public static final double ANGLE_TOLERANCE = 10.0; // degrees (placeholder TODO)
    }

    public static final class VisionConstants {
        // Camera names (must match what's in PhotonVision)
        public static final String[] CAMERA_NAMES = { "example_cam_1", "example_cam_2" }; // (placeholder TODO)
        public static final double AMBIGUITY_THRESHOLD = 0.4; // (placeholder TODO)

        // Filters
        public static final double MAX_TAG_DISTANCE = 100; // meters (placeholder TODO)
        public static final double MAX_FRAME_AGE = 100; // seconds (placeholder TODO)
        public static final double MIN_TAG_COUNT = 1; // (placeholder TODO)
        public static final double MIN_AREA = 1.0; // meters squared (placeholder TODO)
        public static final double MAX_POSE_DIFFERENCE = 1000.0; // meters (placeholder TODO)

        /**
         * Camera positions relative to robot center.
         * Transform3d(x, y, z, rotation)
         * - x: forward/back from center (positive = forward)
         * - y: left/right from center (positive = left)
         * - z: up/down from ground (positive = up)
         */
        public static final Transform3d EXAMPLE_CAMERA_TRANSFORM_1 = new Transform3d(
                new Translation3d(Units.inchesToMeters(12.0), 0.0, Units.inchesToMeters(24.0)),
                new Rotation3d(0.0, Math.toRadians(-15.0), 0.0) // Roll, Pitch, and Yaw = X, Y, Z axis rotations
        ); // (placeholder TODO)

        public static final Transform3d EXAMPLE_CAMERA_TRANSFORM_2 = new Transform3d(
                new Translation3d(Units.inchesToMeters(-12.0), 0.0, Units.inchesToMeters(24.0)),
                new Rotation3d(0.0, Math.toRadians(-15.0), Math.toRadians(180.0)) // Facing backward
        ); // (placeholder TODO)
    }

    public static final class DrivingConstants {
        public static final int CONTROLLER_PORT = 0;

        public static final boolean OPEN_LOOP = true; // Motor runs @ % (true) or exact speed w/ encoder (false)
        public static final double NORMAL_SPEED_MULTIPLIER = 1.0; // Scale according to joystick
        public static final double SLOW_SPEED_MULTIPLIER = 0.3;
    }

    public static final class FieldConstants {
        // Field size: 651.2in x 317.7in (from REBUILT 2026 game manual)
        public static final double FIELD_LENGTH = 16.5405;  // meters
        public static final double FIELD_WIDTH = 8.0696;    // meters

        // Field center
        public static final double CENTER_X = FIELD_LENGTH / 2.0;
        public static final double CENTER_Y = FIELD_WIDTH / 2.0;

        // Alliance Zone boundary (158.6 inches from each alliance wall)
        public static final double ALLIANCE_ZONE_DEPTH = 4.03;  // meters

        // ================================================================
        // HUB (2 per field, one per alliance)
        // ================================================================
        public static final double HUB_SIZE = 1.194;  // 47in x 47in
        public static final double HUB_HEIGHT = 1.83; // 72in opening height
        public static final double HUB_DISTANCE_FROM_WALL = 4.03;

        // Red HUB position (near red alliance wall, x = max)
        public static final double RED_HUB_X = FIELD_LENGTH - HUB_DISTANCE_FROM_WALL;
        public static final double RED_HUB_Y = CENTER_Y;

        // Blue HUB position (near blue alliance wall, x = 0)
        public static final double BLUE_HUB_X = HUB_DISTANCE_FROM_WALL;
        public static final double BLUE_HUB_Y = CENTER_Y;

        // ================================================================
        // BUMP (flanking each HUB)
        // ================================================================
        public static final double BUMP_LENGTH = 1.854;
        public static final double BUMP_WIDTH = 1.128;
        public static final double BUMP_HEIGHT = 0.165;

        // ================================================================
        // TOWER (climbing structure, one per alliance)
        // ================================================================
        public static final double TOWER_DISTANCE_FROM_WALL = 0.625;

        public static final double RED_TOWER_X = FIELD_LENGTH - TOWER_DISTANCE_FROM_WALL;
        public static final double RED_TOWER_Y = CENTER_Y + 2.0;

        public static final double BLUE_TOWER_X = TOWER_DISTANCE_FROM_WALL;
        public static final double BLUE_TOWER_Y = CENTER_Y + 2.0;

        // Tower engagement radius
        public static final double TOWER_ENGAGEMENT_RADIUS = 2.0;  // meters

        // ================================================================
        // DEPOT (FUEL collection area, one per alliance)
        // ================================================================
        public static final double RED_DEPOT_X = FIELD_LENGTH - 1.0;
        public static final double RED_DEPOT_Y = 1.5;

        public static final double BLUE_DEPOT_X = 1.0;
        public static final double BLUE_DEPOT_Y = 1.5;

        // ================================================================
        // NEUTRAL ZONE (field center area)
        // ================================================================
        public static final double NEUTRAL_ZONE_X = CENTER_X;
        public static final double NEUTRAL_ZONE_Y = CENTER_Y;
        public static final double NEUTRAL_CLOSE_OFFSET = 2.0;  // closer collection point
        public static final double NEUTRAL_FAR_OFFSET = 3.0;    // farther collection point
    }

    public static final class RobotPhysicalConstants {
        public static final double ROBOT_LENGTH = Units.inchesToMeters(29.75);
        public static final double ROBOT_WIDTH = Units.inchesToMeters(29.75);
        public static final double BUMPER_THICKNESS = Units.inchesToMeters(3.5);

        // Total size including bumpers
        public static final double ROBOT_LENGTH_WITH_BUMPERS = ROBOT_LENGTH + 2 * BUMPER_THICKNESS;
        public static final double ROBOT_WIDTH_WITH_BUMPERS = ROBOT_WIDTH + 2 * BUMPER_THICKNESS;
    }

    public static final class SwerveConstants {
        public static final double TRACK_WIDTH = Units.inchesToMeters(17.75); // Left to right
        public static final double WHEEL_BASE = Units.inchesToMeters(29.75); // Front to back

        public static final double MAX_SPEED = 4.2; // meters per second
        public static final double MAX_ANGULAR_VELOCITY = 9.547; // rad per second

        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        // SDS Mk4i L2 Gear Ratios
        public static final double DRIVE_GEAR_RATIO = 6.75; // motor spins 6.75 times per wheel rotation
        public static final double AZIMUTH_GEAR_RATIO = 150.0 / 7.0;

        // Front Left Module
        public static final int FL_DRIVE_ID = 7;
        public static final int FL_AZIMUTH_ID = 8;

        // Front Right Module
        public static final int FR_DRIVE_ID = 5;
        public static final int FR_AZIMUTH_ID = 6;

        // Rear Left Module
        public static final int RL_DRIVE_ID = 3;
        public static final int RL_AZIMUTH_ID = 4;

        // Rear Right Module
        public static final int RR_DRIVE_ID = 1;
        public static final int RR_AZIMUTH_ID = 2;

        // Absolute encoders
        public static final int FL_CANCODER_ID = 1;
        public static final int FR_CANCODER_ID = 2;
        public static final int RL_CANCODER_ID = 3;
        public static final int RR_CANCODER_ID = 4;

        // Gyro ID (Pigeon2)
        public static final int PIGEON_ID = 0;

        // ENCODER OFFSETS - Calibration for wheel angles

        // These values tell the robot what "forward" looks like for each wheel.
        //
        // HOW TO CALIBRATE:
        // 1. Put robot on blocks (wheels off ground)
        // 2. Rotate each wheel so it points forward
        // 3. Read the CANCoder value in Phoenix Tuner
        // 4. Put that value here (may need to add/subtract 180)

        public static final double FL_ENCODER_OFFSET = 19.072266 + 180.0;
        public static final double FR_ENCODER_OFFSET = 269.208984 - 180.0;
        public static final double RL_ENCODER_OFFSET = 244.863281 - 180.0;
        public static final double RR_ENCODER_OFFSET = 217.529297 - 180.0;

        // DRIVE MOTOR PID - Tuning for wheel speed control

        // These values were found using SysId characterization
        public static final double DRIVE_kP = 0.064395;
        public static final double DRIVE_kI = 0.0;
        public static final double DRIVE_kD = 0.0;

        // Feedforward values (physics-based compensation)
        public static final double DRIVE_kS = 0.18656; // Static friction
        public static final double DRIVE_kV = 2.5833; // Velocity factor
        public static final double DRIVE_kA = 0.40138; // Acceleration factor

        // AZIMUTH MOTOR PID - Tuning for wheel angle control

        public static final double AZIMUTH_kP = 0.01;
        public static final double AZIMUTH_kI = 0.0;
        public static final double AZIMUTH_kD = 0.0;

        // CURRENT LIMITS - Protects motors from overheating

        public static final int DRIVE_CURRENT_LIMIT = 60; // Amps
        public static final int AZIMUTH_CURRENT_LIMIT = 30; // Amps

        // RAMP RATES - How quickly motors speed up

        // Open loop = teleop driving, Closed loop = auto
        public static final double DRIVE_OPEN_LOOP_RAMP = 0.25; // Seconds to full power
        public static final double DRIVE_CLOSED_LOOP_RAMP = 0.0; // No ramp for precision

        // AUTONOMOUS PATH FOLLOWING - PID for auto routines

        public static final double AUTO_THETA_kP = 4.0; // Rotation correction
        public static final double AUTO_XY_kP = 2.0; // Position correction
    }

    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final int BUTTON_BOARD_PORT = 2;
        public static final double JOYSTICK_DEADBAND = 0.1;
    }

    public static final class IntakeConstants {
        public static final int DEPLOY_MOTOR_ID = 9;
        public static final int ROLLER_MOTOR_ID = 10;

        public static final double DEPLOYED_POSITION = 1.0;
        public static final double STOWED_POSITION = 0.0;
        public static final double POSITION_TOLERANCE = 0.05;

        public static final double DEPLOY_kP = 0.1;
        public static final double INTAKE_SPEED = 0.8;
        public static final double OUTTAKE_SPEED = -0.5;
    }

    public static final class AutoConstants {
        // ================================================================
        // DIP SWITCH CONFIGURATION
        // ================================================================
        // 5-bit selector allows up to 32 auto modes (we use 20)
        public static final int DIP_SWITCH_BIT_0_PORT = 1;  // LSB (1)
        public static final int DIP_SWITCH_BIT_1_PORT = 2;  // (2)
        public static final int DIP_SWITCH_BIT_2_PORT = 3;  // (4)
        public static final int DIP_SWITCH_BIT_3_PORT = 4;  // (8)
        public static final int DIP_SWITCH_BIT_4_PORT = 5;  // MSB (16)

        public static final int NUM_AUTO_MODES = 20;

        // ================================================================
        // AUTO MODE IDENTIFIERS
        // ================================================================
        // Original Modes (0-9)
        public static final int AUTO_DO_NOTHING = 0;           // Safety default
        public static final int AUTO_SCORE_AND_COLLECT = 1;    // Score preload, collect from neutral, score
        public static final int AUTO_QUICK_CLIMB = 2;          // Score preload, climb L1
        public static final int AUTO_SCORE_THEN_CLIMB = 3;     // Score preload, climb L1 (optimized)
        public static final int AUTO_DEPOT_RAID = 4;           // Collect from depot, score all
        public static final int AUTO_FAR_NEUTRAL = 5;          // Score, drive to far neutral, collect, score
        public static final int AUTO_PRELOAD_ONLY = 6;         // Just score preload, hold position
        public static final int AUTO_MAX_CYCLES = 7;           // Pure scoring: shoot-collect-shoot loop
        public static final int AUTO_CLIMB_SUPPORT = 8;        // Score preload, position for TELEOP climb
        public static final int AUTO_WIN_AUTO = 9;             // Aggressive scoring to win AUTO period

        // Optimized Modes (10-14) - Discovered via Simulator Benchmarking
        public static final int AUTO_SCORE_COLLECT_CLIMB = 10; // Score, quick collect, score, climb
        public static final int AUTO_FAST_CLIMB = 11;          // Drive to tower, climb immediately
        public static final int AUTO_BALANCED = 12;            // Score preload + climb (18 pts)
        public static final int AUTO_DEPOT_CLIMB = 13;         // ⭐ BEST: Depot collection then climb (20 pts)
        public static final int AUTO_MAX_POINTS = 14;          // Dynamic: maximize total points

        // Additional Strategic Modes (15-19)
        public static final int AUTO_SAFE_CLIMB = 15;          // Conservative climb with fallback
        public static final int AUTO_DUAL_CYCLE = 16;          // Two full scoring cycles
        public static final int AUTO_DENY_FUEL = 17;           // Collect FUEL to deny opponents
        public static final int AUTO_CENTER_CONTROL = 18;      // Control neutral zone
        public static final int AUTO_ALLIANCE_SUPPORT = 19;    // Support alliance scoring

        // ================================================================
        // AUTO MODE NAMES (for SmartDashboard display)
        // ================================================================
        public static final String[] AUTO_MODE_NAMES = {
            "0: Do Nothing",
            "1: Score & Collect (8 pts)",
            "2: Quick Climb (18 pts)",
            "3: Score Then Climb (18 pts)",
            "4: Depot Raid (5 pts)",
            "5: Far Neutral (3-4 pts)",
            "6: Preload Only (3 pts)",
            "7: Max Cycles (8 pts)",
            "8: Climb Support (3 pts)",
            "9: Win AUTO (4 pts)",
            "10: Score+Collect+Climb (18 pts)",
            "11: Fast Climb (15 pts)",
            "12: Balanced (18 pts)",
            "13: Depot+Climb ⭐ (20 pts)",
            "14: Max Points (18 pts)",
            "15: Safe Climb (15-18 pts)",
            "16: Dual Cycle (6-8 pts)",
            "17: Deny FUEL (strategic)",
            "18: Center Control (strategic)",
            "19: Alliance Support (strategic)"
        };

        // ================================================================
        // TIMING CONSTANTS (seconds)
        // ================================================================
        public static final double SHOOT_TIME_PER_FUEL = 0.75;
        public static final double DRIVE_TO_NEUTRAL_TIME = 8.0;  // Increased for reliability
        public static final double INTAKE_TIMEOUT = 4.0;         // Reduced for faster cycles
        public static final double CLIMB_TIMEOUT = 12.0;
        public static final double DRIVE_TO_TOWER_TIME = 5.0;
        public static final double DEPOT_COLLECTION_TIME = 3.0;

        // ================================================================
        // SPEED CONSTANTS (m/s)
        // ================================================================
        public static final double AUTO_DRIVE_SPEED = 2.0;
        public static final double AUTO_FAST_DRIVE_SPEED = 2.5;
        public static final double AUTO_INTAKE_DRIVE_SPEED = 1.5;
        public static final double AUTO_SLOW_DRIVE_SPEED = 1.0;
    }
}