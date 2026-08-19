package frc.robot.subsystems.swerve;// ethan feet was here

import edu.wpi.first.math.util.Units;

public final class SwerveConstants {
  // Pose Estimation TODO-tune w/ practice
  public static double XY_BASE_STDDEV = 0.1;
  public static double XY_DIST_FACTOR = 0.1;

  public static double HEADING_BASE_STDDEV = 0.1;
  public static double HEADING_DIST_FACTOR = 0.5;


  // Hardware
      //From wheel center axle
  public static final double TRACK_WIDTH = Units.inchesToMeters(18.75); // Left to right
  public static final double WHEEL_BASE = Units.inchesToMeters(18.75); // Front to back (square frame)

  public static final double MAX_SPEED = 4.5; // meters per second — theoretical free speed on
      // SDS MK4i L2 (6.75:1) with a NEO (5676 RPM free / 6.75 * wheel circumference ≈ 4.47 m/s).
      // TODO: measure the real sustainable top speed under load once it's on the robot.
  public static final double MAX_ANGULAR_VELOCITY = 13.4; // rad per second — estimated from
      // MAX_SPEED / (distance from robot center to a module, ~0.337 m on this 18.75in square
      // frame). TODO: re-verify once MAX_SPEED is measured on the real robot.

  public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
  public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

  // SDS MK4i L2 gear ratios
  public static final double DRIVE_GEAR_RATIO = 6.75; // motor rotations per wheel rotation
  public static final double AZIMUTH_GEAR_RATIO = 150.0 / 7.0; // motor rotations per wheel rotation

  // Front Left Module
  public static final int FL_DRIVE_ID = 1;
  public static final int FL_AZIMUTH_ID = 2;

  // Front Right Module
  public static final int FR_DRIVE_ID = 3;
  public static final int FR_AZIMUTH_ID = 4;

  // Rear Left Module
  public static final int RL_DRIVE_ID = 5;
  public static final int RL_AZIMUTH_ID = 6;

  // Rear Right Module
  public static final int RR_DRIVE_ID = 7;
  public static final int RR_AZIMUTH_ID = 8;

  // Gyro ID (Pigeon2)
  public static final int PIGEON_ID = 0;

  // AZIMUTH ZEROING
  //
  // There is no absolute encoder anymore (built-in NEO relative encoder only), so the azimuth
  // encoder is zeroed wherever the module happens to be on boot. That means:
  //   1. Before each match/test, physically point every wheel straight forward (bevel gear
  //      facing the same consistent direction across all 4 modules) BEFORE enabling the robot.
  //   2. The relative encoder is zeroed to 0 = "forward" the instant SwerveModule is constructed.
  //   3. If the robot is bumped/wheels turned by hand after boot, re-zero (re-deploy or add a
  //      "reset azimuth to forward" button binding) before trusting field-relative driving.
  //
  // This replaces the old CANCoder MagnetOffset calibration block — there's nothing to paste
  // from Phoenix Tuner anymore.

  // DRIVE MOTOR PID - Tuning for wheel speed control (SparkMax closed-loop velocity, RPM-based
  // internally but we set velocityConversionFactor so units below are m/s and m/s^2)
  // TODO: re-tune on the new robot — old values were characterized for TalonFX/Mk4i, not NEO/SparkMax.
  public static final double DRIVE_kP = 0.04;
  public static final double DRIVE_kI = 0.0;
  public static final double DRIVE_kD = 0.0;

  // Feedforward values (physics-based compensation) — TODO: re-run SysId for NEO drive motors
  public static final double DRIVE_kS = 0.1; // Static friction (volts)
  public static final double DRIVE_kV = 2.5; // Velocity factor (volts per m/s)
  public static final double DRIVE_kA = 0.3; // Acceleration factor (volts per m/s^2)

  // AZIMUTH MOTOR PID - Tuning for wheel angle control (SparkMax closed-loop position, in
  // wheel rotations — the ~21.4:1 azimuth gear ratio is already folded into the encoder's
  // positionConversionFactor, so error here is always 0-1 wheel rotations regardless of ratio)
  // TODO: re-tune on the new robot — SparkMax closed-loop output is duty cycle (-1 to 1), NOT
  // volts, so this kP will be a very different magnitude than the old TalonFX value (50.0).
  // Start small (e.g. 0.5-1.0) and increase until the wheel tracks setpoints without oscillating.
  public static final double AZIMUTH_kP = 0.6;
  public static final double AZIMUTH_kI = 0.0;
  public static final double AZIMUTH_kD = 0.0;

  // CURRENT LIMITS - Protects motors from overheating
  // SparkMax smartCurrentLimit is a single stall-current limit in amps (REV recommends 40-60A
  // for NEO drive, 20-30A for NEO azimuth). No separate stator/supply split like Phoenix6.

  public static final int DRIVE_CURRENT_LIMIT = 50; // Amps, NEO drive
  public static final int AZIMUTH_CURRENT_LIMIT = 20; // Amps, NEO azimuth

  // COAST OR BRAKE MOTOR - whether or not motor resists rotation when neutral
    // True = Coast, False = brake
  public static final boolean DRIVE_COAST = true;
  public static final boolean AZIMUTH_COAST = false;

  // INVERT MOTORS / CANcoders - whether or not a motor is inverted
    // True = CW +, False = CCW +  
    //            or  
    // True = Invert, False = no Invert

  // TODO: these were tuned for the old TalonFX modules — re-check wheel direction on the new
  // robot with wheels off the ground before trusting these.
  public static final boolean FL_DRIVE_INVERT = true;
  public static final boolean FL_AZIMUTH_INVERT = true;

  public static final boolean FR_DRIVE_INVERT = true;
  public static final boolean FR_AZIMUTH_INVERT = true;

  public static final boolean BL_DRIVE_INVERT = true;
  public static final boolean BL_AZIMUTH_INVERT = true;

  public static final boolean BR_DRIVE_INVERT = true;
  public static final boolean BR_AZIMUTH_INVERT = true;

  // RAMP RATES - How quickly motors speed up

  // Open loop = teleop driving, Closed loop = auto
  public static final double DRIVE_OPEN_LOOP_RAMP = 0.25; // Seconds to full power
  public static final double DRIVE_CLOSED_LOOP_RAMP = 0.0; // No ramp for precision

  // AUTONOMOUS PATH FOLLOWING - PID for auto routines

  public static final double AUTO_THETA_kP = 4.0; // Rotation correction
  public static final double AUTO_XY_kP = 2.0; // Position correction

  // VISION AIM - PID for rotating to face a target (hub/AprilTag)
  public static final double AIM_kP = 0.06; // TODO: tune — (rad/s) per degree of error
  public static final double AIM_kI = 0.0;
  public static final double AIM_kD = 0.004;
  public static final double AIM_TOLERANCE_DEG = 2.0; // degrees — "close enough" to target
}