package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.SwerveCommands;
import frc.robot.foundation.Constants;
import frc.robot.foundation.Superstructure;
import frc.robot.foundation.OI.DriverActionSet;
import frc.robot.foundation.OI.XboxDriver;
import frc.robot.subsystems.swerve.SwerveDrive;

public class RobotContainer {
    private final DriverActionSet driverJoystick;
    private final SwerveDrive swerve;
    private final Superstructure superstructure;

    private final SendableChooser<Command> autoChooser;

    private int speedExponent = 2;

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true); // "Joystick Not Connected"
        driverJoystick = new XboxDriver(Constants.DrivingConstants.CONTROLLER_PORT);

        autoChooser = new SendableChooser<>();
        SmartDashboard.putData("Auto Chooser", autoChooser); // Insert dropdown

        swerve = new SwerveDrive();
        superstructure = new Superstructure(swerve);

        // This command reads joystick inputs and drives the robot
        Command teleopDriveCommand = swerve.teleopCommand(
                () -> applySpeedCurve(driverJoystick.forward()),
                () -> applySpeedCurve(driverJoystick.strafe()),
                () -> applySpeedCurve(driverJoystick.turn()));
        swerve.setDefaultCommand(teleopDriveCommand);

        swerve.resetYaw(Rotation2d.fromDegrees(180));

        SmartDashboard.putData("TeleOp Command", teleopDriveCommand);

        configureButtonBindings();
    }

    private double applySpeedCurve(double input) {
        // |input| <= 1.0 so squaring it makes it slower & of course ^1 does nothing
        return (input != 0.0) ? (Math.pow(Math.abs(input), speedExponent) * ((input > 0) ? 1.0 : -1.0)) : 0;
    }

    private void configureButtonBindings() {
        driverJoystick.resetGyro().onTrue(swerve.resetGyroCommand());

        driverJoystick.toggleFieldRelative().onTrue(new InstantCommand(swerve::toggleFieldRelative));

        driverJoystick.skiStop().onTrue(
                SwerveCommands.skiStopCommand(swerve).until(driverJoystick::isMovementCommanded));

        // 1 = fast, 2 = precise
        driverJoystick.toggleSpeed().onTrue(
                new InstantCommand(() -> speedExponent = (speedExponent == 1) ? 2 : 1));
    }

    // private void buildForwardAuto() {
    // Command auto = Commands.run(
    // // Drive forward at 1.5 m/s, no strafe, no rotation
    // () -> swerve.drive(new Translation2d(1.5, 0), 0, false, false),
    // swerve
    // ).andThen(
    // // Stop after driving
    // Commands.run(() -> swerve.drive(new Translation2d(), 0, false, false),
    // swerve)
    // ).withTimeout(2); // Total runtime: 2 seconds

    // // Add to the auto chooser
    // autoChooser.setDefaultOption("Drive Forward", auto);
    // }

    private void exampleAuto() {
        Command auto = Commands.run(() -> swerve.drive(null, speedExponent, false, false));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void logData() {
        SmartDashboard.putBoolean("Slow Speed", speedExponent == 2);
    }

}
