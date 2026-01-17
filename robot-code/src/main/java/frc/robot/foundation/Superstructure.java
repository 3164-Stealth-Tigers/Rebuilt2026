package frc.robot.foundation;

import frc.robot.subsystems.swerve.SwerveDrive;


public class Superstructure {
    private final SwerveDrive swerve;

    public Superstructure(SwerveDrive swerve) {
        this.swerve = swerve;
    }

    public SwerveDrive getSwerve() {
        return swerve;
    }

}