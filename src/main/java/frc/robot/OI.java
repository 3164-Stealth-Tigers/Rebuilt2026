// CHECK //

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.constants.DrivingConstants;

public final class OI {
  private OI() {
  }

  private static double deadband(double value, double band) {
    return Math.abs(value) > band ? value : 0;
  }

  public static class XboxDriver implements DrivingConstants.DriverActionSet {
    private final CommandXboxController stick;

    public XboxDriver(int port) {
      this.stick = new CommandXboxController(port);
    }

    @Override
    public double forward() {
      return deadband(-stick.getLeftY(), DrivingConstants.JOYSTICK_DEADBAND) * .45;
    }

    @Override
    public double strafe() {
      return deadband(-stick.getLeftX(), DrivingConstants.JOYSTICK_DEADBAND) * .45;
    }

    @Override
    public double turn() {
      return deadband(-stick.getRightX(), DrivingConstants.JOYSTICK_DEADBAND) * 0.4;
    }

    @Override
    public Trigger toggleSpeed() {
      return stick.b();
    }

    // @Override
    // public Trigger resetGyroTo() {
    //   return stick.povUp();
    // }

    // @Override
    // public Trigger resetGyroAway() {
    //   return stick.povDown();
    // }

    @Override
    public Trigger zeroHeading() {
      return stick.povUp();
    }

    @Override
    public Trigger toggleFieldRelative() {
      return stick.povDown();
    }

    @Override
    public Trigger skiStop() {
      return stick.rightBumper();
    }

    @Override
    public boolean isMovementCommanded() {
      return Math.abs(forward()) + Math.abs(strafe()) + Math.abs(turn()) > 0.01;
    }
  }

  public static class XboxOperator implements DrivingConstants.OperatorActionSet {
    private final CommandXboxController stick;

    public XboxOperator(int port) {
      stick = new CommandXboxController(port);
    }

    @Override
    public Trigger runFlywheel() {
      return stick.x();
    }

    @Override
    public Trigger deployIntake() {
      return stick.leftBumper();
    }

    @Override
    public Trigger retractIntake() {
      return stick.rightBumper();
    }

    @Override
    public Trigger runIntakeForward() {
      return stick.leftTrigger();
    }

    @Override
    public Trigger runIntakeReverse() {
      return stick.rightTrigger();
    }

    @Override
    public Trigger turretLeft() {
      return stick.povLeft();
    }

    @Override
    public Trigger turretRight() {
      return stick.povRight();
    }

    @Override
    public Trigger runSpintake() {
      return stick.a();
    }
  }
}