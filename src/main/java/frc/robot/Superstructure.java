package frc.robot;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * Superstructure holds references to all robot subsystems.
 *
 * <p>This provides a centralized way to access subsystems for coordination and complex sequences
 * that span multiple mechanisms.
 */
public class Superstructure {
  private final SwerveDrive swerve;
  private final Shooter shooter;
  private final Vision vision;
  private final Intake intake;

  public Superstructure(SwerveDrive swerve, Vision vision, Shooter shooter, Intake intake) {
    this.swerve = swerve;
    this.shooter = shooter;
    this.vision = vision;
    this.intake = intake;
  }

  public SwerveDrive getSwerve() {
    return swerve;
  }

  public Shooter getShooter() {
    return shooter;
  }

  public Vision getVision() {
    return vision;
  }

  public Intake getIntake() {
    return intake;
  }
}
