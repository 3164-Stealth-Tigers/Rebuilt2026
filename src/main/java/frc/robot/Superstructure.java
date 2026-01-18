package frc.robot;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * Superstructure holds references to all robot subsystems.
 *
 * This provides a centralized way to access subsystems for
 * coordination and complex sequences that span multiple mechanisms.
 */
public class Superstructure {
    private final SwerveDrive swerve;
    private final Shooter shooter;
    private final Vision vision;
    private final Intake intake;
    private final Climber climber;

    public Superstructure(SwerveDrive swerve, Vision vision, Shooter shooter,
                          Intake intake, Climber climber) {
        this.swerve = swerve;
        this.shooter = shooter;
        this.vision = vision;
        this.intake = intake;
        this.climber = climber;
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

    public Climber getClimber() {
        return climber;
    }
}