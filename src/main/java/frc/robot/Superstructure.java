package frc.robot;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

public class Superstructure {
    private final SwerveDrive swerve;
    private final Shooter shooter;
    private final Vision vision;

    public Superstructure(SwerveDrive swerve, Vision vision, Shooter shooter) {
        this.swerve = swerve;
        this.shooter = shooter;
        this.vision = vision;
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

}