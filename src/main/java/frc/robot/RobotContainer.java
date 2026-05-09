// CHECK //
package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.OI.XboxDriver;
// import frc.robot.OI.XboxOperator;
// import frc.robot.OI.XboxTester;
// import frc.robot.commands.SwerveCommands;
// import frc.robot.commands.SwerveCommands;
// import frc.robot.subsystems.intake.Intake;
// import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
// import frc.robot.subsystems.turret.Turret;
// import frc.robot.subsystems.vision.Vision;
import frc.robot.util.Elastic;
import frc.robot.util.constants.DrivingConstants;

public class RobotContainer {
  // Controllers
  private final XboxDriver driverJoystick;
  // private final XboxOperator operatorJoystick;
  // private final XboxTester testerJoystick;

  // Auto
  // private final SendableChooser<Command> autoChooser;

  // Swerve
  private int speedExponent = 1;

  // Subsystems
  private final Swerve swerve;
  //private final Vision vision;
  // private final Turret turret;
  // private final Intake intake;
  // private final Spindexer spindexer;

  public RobotContainer() {
    // Controllers
    DriverStation.silenceJoystickConnectionWarning(true);
    driverJoystick = new XboxDriver(DrivingConstants.DRIVER_PORT);
    // operatorJoystick = new XboxOperator(DrivingConstants.OPERATOR_PORT);
    // testerJoystick = new XboxTester(DrivingConstants.TEST_PORT);

    // Auto
    // autoChooser = new SendableChooser<>();
    // SmartDashboard.putData("Auto Chooser", autoChooser);

    // Subsystems
    // swerve = new Swerve(vision);
    swerve = new Swerve();
    // vision = new Vision();
    // turret = new Turret(); // should be passing through vision
    // turret = new Turret(vision);
    // spindexer = new Spindexer();
    // intake = new Intake();

    // Init methods
    setupDrive();
    configureButtonBindings();

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      var controller = new PPHolonomicDriveController(
          new PIDConstants(SwerveConstants.DRIVE_kP, SwerveConstants.DRIVE_kI, SwerveConstants.DRIVE_kD),
          new PIDConstants(SwerveConstants.AIM_kP, SwerveConstants.AIM_kI, SwerveConstants.AIM_kD));

      AutoBuilder.configure(
          swerve::getPose, // Robot pose supplier
          (a) -> {
            swerve.zeroHeading();
            swerve.zeroGyro();
          }, // Method to reset odometry (will be called if your auto has a starting pose)
          swerve::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speeds, feedforwards) -> swerve.drive(speeds), // Method that will drive the robot given ROBOT RELATIVE
                                                          // ChassisSpeeds. Also optionally outputs individual
                                                          // module feedforwards
          controller,
          config, // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Blue;
            }
            return false;
          },

          swerve // Reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // registerAutoRoutines();
  }

  public Command getAutonomousCommand() {
    try {
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath path = PathPlannerPath.fromPathFile("simple straight");

      // Create a path following command using AutoBuilder. This will also trigger
      // event markers.
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  // On container init
  private void setupDrive() {
    // Drive command
    Command teleopDriveCommand = swerve.teleopCommand(
        () -> applySpeedCurve(driverJoystick.forward()),
        () -> applySpeedCurve(driverJoystick.strafe()),
        () -> applySpeedCurve(driverJoystick.turn()));
    swerve.setDefaultCommand(teleopDriveCommand);

    // Gyro
    swerve.getH().setYaw(new Rotation2d());
    swerve.zeroHeading();
    // turret.zeroTurretYaw();

    // .setYaw(Rotation2d.fromDegrees(
    // (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) ==
    // DriverStation.Alliance.Red)
    // ? 180
    // : 0));

    // SmartDashboard.putData("TeleOp Command", teleopDriveCommand);
    Elastic.sendNotification(new Elastic.Notification(Elastic.NotificationLevel.INFO, "Teleop Setup Complete",
        "drive controls were set up successfully"));
  }

  private double applySpeedCurve(double input) {
    if (input == 0.0) {
      return 0.0;
    }

    double magnitude = Math.pow(Math.abs(input), speedExponent);
    double sign = (input > 0) ? 1.0 : -1.0;
    return magnitude * sign;
  }

  private void configureButtonBindings() {
    // DRIVER
    driverJoystick.toggleFieldRelative().onTrue(new InstantCommand(() -> {
      swerve.setFieldRelative(!swerve.isFieldRelative());
    }));

    driverJoystick
        .toggleSpeedExponent()
        .onTrue(new InstantCommand(() -> {
          speedExponent = (speedExponent == 1) ? 2 : 1;
        }));

    driverJoystick.resetGyro().onTrue(new InstantCommand(() -> swerve.zeroGyro()));

    // driverJoystick
    // .skiStop()
    // .onTrue(SwerveCommands.SkiStop(swerve).until(driverJoystick::isMovementCommanded));

    /*
     * // OPERATOR
     * if (DrivingConstants.OPERATORorTEST) {
     * operatorJoystick.runSpindexer().whileTrue(Commands.startEnd(spindexer::
     * startFeed, spindexer::stopFeed, spindexer));
     * 
     * operatorJoystick.deployIntake().onTrue(new
     * InstantCommand(intake::deployIntakeMechanism));
     * 
     * operatorJoystick.runIntake().whileTrue(
     * Commands.startEnd(intake.getR()::runIntake, intake.getR()::stop,
     * intake));
     * 
     * // Shoot
     * operatorJoystick.runFlywheelKicker().whileTrue(
     * Commands.sequence(
     * Commands.runOnce(turret::spinFlywheel50, turret),
     * Commands.waitSeconds(1.5),
     * Commands.runOnce(turret::spinFlywheel100, turret),
     * Commands.waitSeconds(1.5),
     * Commands.runOnce(turret::startFlywheel, turret),
     * 
     * Commands.idle(turret).andThen(
     * Commands.runOnce(turret::stopFlywheel, turret))
     * )
     * );
     * 
     * // Toggle auto aim
     * operatorJoystick.toggleAutoAim().onTrue(new
     * InstantCommand(turret::toggleAutoAimEnabled));
     * 
     * // Pitch
     * operatorJoystick.posPitch().onTrue(
     * new InstantCommand(() -> {
     * if (turret.getAutoAimEnabled()) {
     * turret.turnPitchTo(turret.getPitch() +
     * DrivingConstants.PITCH_INCREMENT_MAGNITUDE);
     * }
     * }));
     * 
     * operatorJoystick.negPitch().onTrue(
     * new InstantCommand(() -> {
     * if (turret.getAutoAimEnabled()) {
     * turret.turnPitchTo(turret.getPitch() -
     * DrivingConstants.PITCH_INCREMENT_MAGNITUDE);
     * }
     * }));
     * 
     * // Yaw
     * operatorJoystick.posYaw().onTrue(
     * new InstantCommand(() -> {
     * if (turret.getAutoAimEnabled()) {
     * turret.moveYawTo(turret.getYaw() + DrivingConstants.YAW_INCREMENT_MAGNITUDE);
     * }
     * }));
     * 
     * operatorJoystick.negYaw().onTrue(
     * new InstantCommand(() -> {
     * if (turret.getAutoAimEnabled()) {
     * turret.moveYawTo(turret.getYaw() - DrivingConstants.YAW_INCREMENT_MAGNITUDE);
     * }
     * }));
     * 
     * operatorJoystick.zeroYaw().onTrue(
     * new InstantCommand(() -> {
     * if (turret.getAutoAimEnabled()) {
     * turret.zeroTurretYaw();
     * }
     * }));
     * 
     * // gone but not forgotten :( fly high chatClipThat command
     * // operatorJoystick.chatClipThat().onTrue(
     * // new RunCommand(() -> {
     * // vision.getL().rewindRecord(5);
     * // }, vision));
     * } else {
     * testerJoystick.runIntake().whileTrue(
     * Commands.startEnd(intake.getR()::runIntake, intake.getR()::stop, intake));
     * 
     * testerJoystick.runSpindexer().whileTrue(
     * Commands.startEnd(spindexer::startFeed, spindexer::stopFeed, spindexer));
     * 
     * testerJoystick.runKicker().whileTrue(
     * Commands.startEnd(turret.getKicker()::run, turret.getKicker()::stop,
     * turret));
     * 
     * testerJoystick.runFlywheel().whileTrue(
     * Commands.sequence(
     * Commands.runOnce(turret::spinFlywheel50, turret),
     * Commands.waitSeconds(1.5),
     * Commands.runOnce(turret::spinFlywheel100, turret),
     * Commands.waitSeconds(1.5),
     * Commands.runOnce(turret::startFlywheel, turret),
     * 
     * Commands.idle(turret).andThen(
     * Commands.runOnce(turret::stopFlywheel, turret))
     * )
     * );
     * 
     * testerJoystick.runFlywheel().onFalse(
     * new InstantCommand(turret::stopFlywheel)
     * );
     * 
     * testerJoystick.DeployIntake().onTrue(new
     * InstantCommand(intake::deployIntakeMechanism));
     * 
     * // Pitch
     * testerJoystick.posPitch().onTrue(
     * new InstantCommand(() -> {
     * if (turret.getAutoAimEnabled()) {
     * turret.turnPitchTo(turret.getPitch() +
     * DrivingConstants.PITCH_INCREMENT_MAGNITUDE);
     * }
     * }));
     * 
     * testerJoystick.negPitch().onTrue(
     * new InstantCommand(() -> {
     * if (turret.getAutoAimEnabled()) {
     * turret.turnPitchTo(turret.getPitch() -
     * DrivingConstants.PITCH_INCREMENT_MAGNITUDE);
     * }
     * }));
     * }
     */
  }

  // private void registerAutoRoutines() {
  // // autoChooser.setDefaultOption(NAME, REF);
  // // autoChooser.addOption(NAME, REF);
  // }

  // public Command getAutonomousCommand() {
  // return autoChooser.getSelected();
  // }

  public void onDisabled() {
  }

  public void onEnabled() {
    swerve.zeroHeading();
  }
}