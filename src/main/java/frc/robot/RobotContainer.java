package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auto.AutoRoutines;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.SwerveCommands;
import frc.robot.Superstructure;
import frc.robot.OI.DriverActionSet;
import frc.robot.OI.XboxDriver;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swerve.SwerveDrive;

public class RobotContainer {
    private final DriverActionSet driverJoystick;
    private final SwerveDrive swerve;
    private final Vision vision;
    private final Shooter shooter;
    private final Intake intake;
    private final Climber climber;
    private final Superstructure superstructure;

    private final SendableChooser<Command> autoChooser;

    private int speedExponent = 2;

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true); // "Joystick Not Connected"
        driverJoystick = new XboxDriver(Constants.DrivingConstants.CONTROLLER_PORT);

        autoChooser = new SendableChooser<>();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Initialize all subsystems
        swerve = new SwerveDrive();
        vision = new Vision();
        shooter = new Shooter(vision, swerve);
        intake = new Intake();
        climber = new Climber();
        superstructure = new Superstructure(swerve, vision, shooter, intake, climber);

        // This command reads joystick inputs and drives the robot
        Command teleopDriveCommand = swerve.teleopCommand(
                () -> applySpeedCurve(driverJoystick.forward()),
                () -> applySpeedCurve(driverJoystick.strafe()),
                () -> applySpeedCurve(driverJoystick.turn()));
        swerve.setDefaultCommand(teleopDriveCommand);

        swerve.resetYaw(Rotation2d.fromDegrees(180));

        SmartDashboard.putData("TeleOp Command", teleopDriveCommand);

        configureButtonBindings();
        registerAutoRoutines();
    }

    private double applySpeedCurve(double input) {
        // |input| <= 1.0 so squaring it makes it slower & of course ^1 does nothing
        return (input != 0.0) ? (Math.pow(Math.abs(input), speedExponent) * ((input > 0) ? 1.0 : -1.0)) : 0;
    }

    private void configureButtonBindings() {
        // Swerve controls
        driverJoystick.resetGyro().onTrue(swerve.resetGyroCommand());

        driverJoystick.toggleFieldRelative().onTrue(new InstantCommand(swerve::toggleFieldRelative));

        driverJoystick.skiStop().onTrue(
                SwerveCommands.skiStopCommand(swerve).until(driverJoystick::isMovementCommanded));

        // 1 = fast, 2 = precise
        driverJoystick.toggleSpeed().onTrue(
                new InstantCommand(() -> speedExponent = (speedExponent == 1) ? 2 : 1));

        // Intake controls (uses new trigger methods from OI)
        driverJoystick.intake().whileTrue(intake.holdToIntakeCommand());
        driverJoystick.outtake().whileTrue(intake.outtakeCommand());

        // Climber controls
        driverJoystick.climbL1().onTrue(climber.climbToL1Command());
        driverJoystick.climbL2().onTrue(climber.climbToL2Command());
        driverJoystick.climbL3().onTrue(climber.climbToL3Command());
        driverJoystick.stowClimber().onTrue(climber.stowCommand());
    }

    /**
     * Register all autonomous routines with the auto chooser.
     */
    private void registerAutoRoutines() {
        // Safety default
        autoChooser.setDefaultOption("Do Nothing", AutoRoutines.doNothing());

        // Simple movement autos
        autoChooser.addOption("Drive Forward", AutoRoutines.driveForwardAuto(swerve));
        autoChooser.addOption("Drive Backward", AutoRoutines.driveBackwardAuto(swerve));

        // Intake autos
        autoChooser.addOption("Drive and Intake", AutoRoutines.driveAndIntakeAuto(swerve, intake));
        autoChooser.addOption("Intake Then Drive", AutoRoutines.intakeThenDriveAuto(swerve, intake));

        // Scoring autos
        autoChooser.addOption("Intake and Score", AutoRoutines.intakeAndScoreAuto(swerve, intake, shooter));
        autoChooser.addOption("Two FUEL Auto", AutoRoutines.twoFuelAuto(swerve, intake, shooter));
        autoChooser.addOption("Three FUEL Auto", AutoRoutines.threeFuelAuto(swerve, intake, shooter));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void logData() {
        SmartDashboard.putBoolean("Slow Speed", speedExponent == 2);
    }
}