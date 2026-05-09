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
      return deadband(stick.getLeftY(), DrivingConstants.JOYSTICK_DEADBAND);
    }

    @Override
    public double strafe() {
      return deadband(stick.getLeftX(), DrivingConstants.JOYSTICK_DEADBAND);
    }

    @Override
    public double turn() {
      return deadband(stick.getRightX(), DrivingConstants.JOYSTICK_DEADBAND);
    }

    @Override
    public Trigger toggleSpeedExponent() {
      return stick.povLeft();
    }

    @Override
    public Trigger toggleFieldRelative() {
      return stick.povUp();
    }

    @Override
    public Trigger resetGyro() {
      return stick.povDown();
    }

    // @Override
    // public Trigger zeroHeading() {
    // return stick.povUp();
    // }

    // @Override
    // public Trigger skiStop() {
    // return stick.rightBumper();
    // }

    // @Override
    // public boolean isMovementCommanded() {
    // return Math.abs(forward()) + Math.abs(strafe()) + Math.abs(turn()) > 0.01;
    // }
  }

  public static class XboxOperator implements DrivingConstants.OperatorActionSet {
    private final CommandXboxController stick;

    public XboxOperator(int port) {
      stick = new CommandXboxController(port);
    }

    @Override
    public Trigger runFlywheelKicker() {
      return stick.y();
    }

    @Override
    public Trigger deployIntake() {
      return stick.x();
    }

    @Override
    public Trigger runIntake() {
      return stick.a();
    }

    @Override
    public Trigger runSpindexer() {
      return stick.b();
    }

    @Override
    public Trigger posPitch() {
      return stick.povUp();
    }

    @Override
    public Trigger negPitch() {
      return stick.povDown();
    }

    @Override
    public Trigger posYaw() {
      return stick.povLeft();
    }

    @Override
    public Trigger negYaw() {
      return stick.povRight();
    }

    @Override
    public Trigger toggleAutoAim() {
      return stick.rightTrigger();
    }

    @Override
    public Trigger zeroYaw() {
      return stick.leftTrigger();
    }
/* 
    @Override
    public Trigger chatClipThat() {
      return stick.povLeft();
    }
      */
  }
  
  public static class XboxTester implements DrivingConstants.TesterActionSet {
    private final CommandXboxController stick;

    public XboxTester(int port) {
      this.stick = new CommandXboxController(port);
    }

    @Override
    public Trigger runIntake() {
      return stick.a();
    }

    @Override
    public Trigger runSpindexer() {
      return stick.b();
    }

    @Override
    public Trigger runKicker() {
      return stick.y();
    }

    @Override
    public Trigger runFlywheel() {
      return stick.x();
    }

    @Override
    public Trigger DeployIntake() {
      return stick.povLeft();
    }

    @Override
    public Trigger posPitch() {
      return stick.povUp();
    }

    @Override
    public Trigger negPitch() {
      return stick.povDown();
    }
  }
}