package frc.robot;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

public class Superstructure {
    private final SwerveDrive swerve;
    private final Shooter shooter;

    public Superstructure(SwerveDrive swerve, Vision vision, Shooter shooter) {
        this.swerve = swerve;
        this.shooter = shooter;
    }

    public SwerveDrive getSwerve() {
        return swerve;
    }

    public Shooter getShooter() {
        return shooter;
    }

}