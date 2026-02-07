package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class OI {
  private OI() {}

  public static double deadband(double value, double band) {
    return Math.abs(value) > band ? value : 0;
  }

  public interface DriverActionSet {
    /**
     * Movement along the X axis (forward/backward).
     *
     * @return Value from -1 (full backward) to 1 (full forward)
     */
    double forward();

    /**
     * Movement along the Y axis (left/right strafe).
     *
     * @return Value from -1 (full left) to 1 (full right)
     */
    double strafe();

    /**
     * Rotation around the Z axis (turning).
     *
     * @return Value from -1 (full clockwise) to 1 (full counter-clockwise)
     */
    double turn();

    /** Reset the gyroscope to know which way is "forward" */
    Trigger resetGyro();

    /** Toggle between fast and slow driving modes */
    Trigger toggleSpeed();

    /** Toggle between field-relative and robot-relative control */
    Trigger toggleFieldRelative();

    /** Lock wheels in X pattern to resist pushing (ski stop) */
    Trigger skiStop();

    /**
     * Check if the driver is commanding any movement. Used to release ski stop when driver wants to
     * move.
     *
     * @return true if any movement axis is non-zero
     */
    boolean isMovementCommanded();

    // ================================================================
    // INTAKE CONTROLS
    // ================================================================

    /** Hold to deploy intake and run rollers */
    Trigger intake();

    /** Hold to run outtake (eject FUEL) */
    Trigger outtake();
  }

  public static class XboxDriver implements DriverActionSet {
    private final CommandXboxController stick;

    public XboxDriver(int port) {
      this.stick = new CommandXboxController(port);
    }

    @Override
    public double forward() {
      // Negative because Y axis is inverted on controllers
      return deadband(-stick.getLeftY(), 0.08);
    }

    @Override
    public double strafe() {
      // Negative to match field coordinate system (left = positive Y)
      return deadband(-stick.getLeftX(), 0.08);
    }

    @Override
    public double turn() {
      // 70% speed multiplier for more controlled turning
      return deadband(-stick.getRightX(), 0.08) * 0.7;
    }

    @Override
    public Trigger toggleSpeed() {
      return stick.x();
    }

    @Override
    public Trigger resetGyro() {
      return stick.start();
    }

    @Override
    public Trigger toggleFieldRelative() {
      return stick.back();
    }

    @Override
    public Trigger skiStop() {
      return stick.y();
    }

    @Override
    public boolean isMovementCommanded() {
      return forward() + strafe() + turn() != 0;
    }

    // ================================================================
    // INTAKE CONTROLS
    // ================================================================

    @Override
    public Trigger intake() {
      return stick.rightBumper();
    }

    @Override
    public Trigger outtake() {
      return stick.leftBumper();
    }
  }
}
