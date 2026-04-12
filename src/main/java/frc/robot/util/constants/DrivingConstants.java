package frc.robot.util.constants;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swerve.SwerveConstants;

public final class DrivingConstants {
  // Controller stuff
  public static final int DRIVER_PORT = 0;
  public static final int TESTER_PORT = 1;

  public static final double JOYSTICK_DEADBAND = 0.1;

  public static final boolean OPEN_LOOP = false;
  //
  //
  // CSPathing path constraints
  // meters & seconds
  public static final double DRIVE_MAX_VEL = SwerveConstants.MAX_SPEED;
  public static final double DRIVE_MAX_ACC = 3.0;

  // radians & seconds
  public static final double TURN_MAX_VEL = SwerveConstants.MAX_ANGULAR_VELOCITY;
  public static final double TURN_MAX_ACC = 12.0;
  //
  //
  // Action Sets
  public interface DriverActionSet {

    double forward();

    double strafe();

    double turn();

    // Trigger resetGyroTo();

    // Trigger resetGyroAway();

    Trigger zeroHeading();

    Trigger toggleSpeed();

    Trigger toggleFieldRelative();

    Trigger skiStop();

    boolean isMovementCommanded();
  }
  //
  //
  //
  public interface OperatorActionSet {
    Trigger runFlywheel();

    Trigger deployIntake();

    Trigger retractIntake();

    Trigger runIntakeForward();

    Trigger runIntakeReverse();

    Trigger turretLeft();

    Trigger turretRight();

    Trigger runSpintake();
  }
}