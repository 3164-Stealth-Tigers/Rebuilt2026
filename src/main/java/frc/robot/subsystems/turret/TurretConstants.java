package frc.robot.subsystems.turret;

public final class TurretConstants {
  // Physical
  public static final double FLYWHEEL_RADIUS = 0.0508; // meters
  public static final double FLYWHEEL_EFFICIENCY = 1; // TODO: calibrate
  // ^ "Motors are moving, but how much of that is transferred to the ball?"
  //
  //
  // Yaw'll
  public static final double YAW_TOLERANCE = 2.0 + 1.5; // degrees
  public static final double MAX_YAW = 300 - YAW_TOLERANCE;
  public static final double MIN_YAW = 0.0 + YAW_TOLERANCE;
  public static final double YAW_NUDGE_DEGREES = 2.0; // manual POV nudge steps
  public static final double INIT_YAW = 0.0;
  public static final double OFFSET_YAW = 0.0; // degrees TODO: calibrate
  public static final double YAW_kP = 0.1; // TODO: placeholder
  public static final double YAW_kI = 0.0; // TODO: placeholder
  public static final double YAW_kD = 0.01; // TODO: placeholder
  //
  //
  // Pitch
  public static final double PITCH_TOLERANCE = 2.0 + 1.5; // degrees
  public static final double MAX_PITCH = 60.0 - PITCH_TOLERANCE;
  public static final double MIN_PITCH = 5.0 + PITCH_TOLERANCE;
  public static final double INIT_PITCH = 0.0;
  public static final double PITCH_DEGREE_RATIO = 3.0 / 25.0; // 25 motor rot = 3 big rot | 1:8.33
  public static final double PITCH_kP = 0.5; // (unused)
  public static final double PITCH_kI = 0.0; // (unused)
  public static final double PITCH_kD = 0.0; // (unused)
  public static final int PITCH_CURRENT_LIMIT = 80;
  public static final double OFFSET_PITCH = 0.0; // degrees TODO: calibrate
  // ^ Degrees changed in pitch per encoder rotation
  //
  //
  //
  // CAN IDs
  public static final int HOOD_MOTOR_ID = 21; // Pitch
  public static final int FLYWHEEL_MOTOR_ID = 20; // Shooter
  public static final int KICKER_MOTOR_ID = 18; // Kicker
  public static final int TURN_MOTOR_ID = 22; // Yasw
  //
  //
  // Flywheel Motor Configuration
  public static final double FLYWHEEL_RPM_LIMIT = 8000;
  public static final double FLYWHEEL_SHOT_RPM = FLYWHEEL_RPM_LIMIT;
  public static final double FLYWHEEL_RPM_TOLERANCE = 50.0;

  public static final double FLYWHEEL_RAMP_RATE = 0.75; // seconds (0, 1]

  // Look these up if you're interested.
  // It's called PID control. It's for error correction & smoothing.
  public static final double FLYWHEEL_kP = 0.2; // Proportional Gain
  public static final double FLYWHEEL_kI = 0.0; // Integral Gain
  public static final double FLYWHEEL_kD = 0.0005; // Derivative Gain
  public static final double FLYWHEEL_kS = 0.15; // Signum Gain (static friction)
  public static final double FLYWHEEL_kA = 0.015; // Acceleration Gain
  public static final double FLYWHEEL_kV = 12 / FLYWHEEL_SHOT_RPM * 60; // Velocity Gain

  public static final int FLYWHEEL_STATOR_CURRENT_LIMIT = 120; // Amps stator
  public static final int FLYWHEEL_SUPPLY_CURRENT_LIMIT = 80; // Amps from battery
  //
  //
  // Kicker Motor Configuration
  public static final int KICKER_CURRENT_LIMIT = 80;
}