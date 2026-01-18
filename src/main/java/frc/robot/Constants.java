package frc.robot;

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

        public static final double ANGLE_TOLERANCE = 10.0; // degrees
    }

    public static final class VisionConstants {
        // Camera names (must match what's in PhotonVision)
        public static final String[] CAMERA_NAMES = { "front_camera", "back_camera" };

        /**
         * Camera positions relative to robot center.
         * Transform3d(x, y, z, rotation)
         * - x: forward/back from center (positive = forward)
         * - y: left/right from center (positive = left)
         * - z: up/down from ground (positive = up)
         */
        public static final Transform3d FRONT_CAMERA_TRANSFORM = new Transform3d(
                new Translation3d(Units.inchesToMeters(12.0), 0.0, Units.inchesToMeters(24.0)),
                new Rotation3d(0.0, Math.toRadians(-15.0), 0.0) // Tilted down 15°
        );

        public static final Transform3d BACK_CAMERA_TRANSFORM = new Transform3d(
                new Translation3d(Units.inchesToMeters(-12.0), 0.0, Units.inchesToMeters(24.0)),
                new Rotation3d(0.0, Math.toRadians(-15.0), Math.toRadians(180.0)) // Facing backward
        );
    }

    public static final class DrivingConstants {
        public static final int CONTROLLER_PORT = 0;

        public static final boolean OPEN_LOOP = true; // Motor runs @ % (true) or exact speed w/ encoder (false)
        public static final double NORMAL_SPEED_MULTIPLIER = 1.0; // Scale according to joystick
        public static final double SLOW_SPEED_MULTIPLIER = 0.3;
    }

    public static final class FieldConstants {
        // Standard FRC field size
        public static final double FIELD_LENGTH = Units.feetToMeters(54.0);
        public static final double FIELD_WIDTH = Units.feetToMeters(27.0);
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
}