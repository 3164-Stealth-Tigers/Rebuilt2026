package frc.robot;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

public class Superstructure {
    private final SwerveDrive swerve;
    private final Shooter shooter;

    public Superstructure(SwerveDrive swerve, Vision vision) {
        this.swerve = swerve;
        this.shooter = new Shooter(vision);
    }

    public SwerveDrive getSwerve() {
        return swerve;
    }

    public Shooter getShooter() {
        return shooter;
    }

}