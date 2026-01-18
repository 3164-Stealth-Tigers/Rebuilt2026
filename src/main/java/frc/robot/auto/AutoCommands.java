package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SwerveCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * Reusable building blocks for autonomous routines.
 *
 * This class provides atomic commands that can be combined
 * to create complete autonomous sequences in AutoRoutines.
 *
 * Categories:
 * - Movement: Drive forward, drive to pose, follow path
 * - Intake: Collect FUEL while moving or stationary
 * - Shooter: Score FUEL in the hub
 * - Utility: Wait, log messages
 */
public final class AutoCommands {

    private AutoCommands() {
        // Utility class - prevent instantiation
    }

    // ================================================================
    // MOVEMENT COMMANDS
    // ================================================================

    /**
     * Drive forward a specified distance at a given speed.
     *
     * @param swerve The swerve drive subsystem
     * @param distanceMeters Distance to travel (positive = forward)
     * @param speedMps Speed in meters per second
     * @return Command that drives forward
     */
    public static Command driveForward(SwerveDrive swerve, double distanceMeters, double speedMps) {
        return new SwerveCommands.DriveDistanceCommand(swerve, speedMps, 0, distanceMeters)
            .withName("Drive Forward " + distanceMeters + "m");
    }

    /**
     * Drive backward a specified distance.
     *
     * @param swerve The swerve drive subsystem
     * @param distanceMeters Distance to travel (positive value)
     * @param speedMps Speed in meters per second (positive value)
     * @return Command that drives backward
     */
    public static Command driveBackward(SwerveDrive swerve, double distanceMeters, double speedMps) {
        return new SwerveCommands.DriveDistanceCommand(swerve, -speedMps, 0, distanceMeters)
            .withName("Drive Backward " + distanceMeters + "m");
    }

    /**
     * Strafe left a specified distance.
     *
     * @param swerve The swerve drive subsystem
     * @param distanceMeters Distance to travel
     * @param speedMps Speed in meters per second
     * @return Command that strafes left
     */
    public static Command strafeLeft(SwerveDrive swerve, double distanceMeters, double speedMps) {
        return new SwerveCommands.DriveDistanceCommand(swerve, 0, speedMps, distanceMeters)
            .withName("Strafe Left " + distanceMeters + "m");
    }

    /**
     * Strafe right a specified distance.
     *
     * @param swerve The swerve drive subsystem
     * @param distanceMeters Distance to travel
     * @param speedMps Speed in meters per second
     * @return Command that strafes right
     */
    public static Command strafeRight(SwerveDrive swerve, double distanceMeters, double speedMps) {
        return new SwerveCommands.DriveDistanceCommand(swerve, 0, -speedMps, distanceMeters)
            .withName("Strafe Right " + distanceMeters + "m");
    }

    /**
     * Drive to a specific pose on the field.
     *
     * @param swerve The swerve drive subsystem
     * @param targetPose Target position and heading
     * @return Command that drives to the pose
     */
    public static Command driveToPose(SwerveDrive swerve, Pose2d targetPose) {
        return new SwerveCommands.DriveToPoseCommand(swerve, targetPose)
            .until(() -> isAtPose(swerve, targetPose, 0.1, 5.0))
            .withName("Drive to Pose");
    }

    /**
     * Drive to a pose with custom tolerances.
     *
     * @param swerve The swerve drive subsystem
     * @param targetPose Target position and heading
     * @param positionToleranceMeters How close to get (meters)
     * @param angleTolerance How close to get (degrees)
     * @return Command that drives to the pose
     */
    public static Command driveToPose(SwerveDrive swerve, Pose2d targetPose,
                                       double positionToleranceMeters, double angleTolerance) {
        return new SwerveCommands.DriveToPoseCommand(swerve, targetPose)
            .until(() -> isAtPose(swerve, targetPose, positionToleranceMeters, angleTolerance))
            .withName("Drive to Pose");
    }

    /**
     * Follow a PathPlanner path.
     *
     * Note: Requires PathPlanner auto builder to be configured in SwerveDrive.
     *
     * @param pathName Name of the path file (without extension)
     * @return Command that follows the path, or does nothing if path not found
     */
    public static Command followPath(String pathName) {
        // TODO: Implement PathPlanner integration
        // return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName));
        return Commands.print("PathPlanner path '" + pathName + "' not implemented yet")
            .withName("Follow Path: " + pathName);
    }

    // ================================================================
    // INTAKE COMMANDS
    // ================================================================

    /**
     * Deploy intake and collect FUEL until detected.
     *
     * @param intake The intake subsystem
     * @return Command that collects one FUEL
     */
    public static Command intakeFuel(Intake intake) {
        return IntakeCommands.fullIntakeSequence(intake)
            .withName("Auto Intake FUEL");
    }

    /**
     * Drive forward while intaking FUEL.
     *
     * @param intake The intake subsystem
     * @param swerve The swerve drive subsystem
     * @param distanceMeters Distance to travel
     * @param speedMps Speed in meters per second
     * @return Command that drives and intakes simultaneously
     */
    public static Command intakeWhileDriving(Intake intake, SwerveDrive swerve,
                                              double distanceMeters, double speedMps) {
        return Commands.parallel(
            driveForward(swerve, distanceMeters, speedMps),
            IntakeCommands.continuousIntakeCommand(intake)
        ).withName("Intake While Driving");
    }

    /**
     * Deploy intake and collect FUEL with timeout.
     *
     * @param intake The intake subsystem
     * @param timeoutSeconds Maximum time to wait
     * @return Command that collects FUEL or times out
     */
    public static Command intakeFuelWithTimeout(Intake intake, double timeoutSeconds) {
        return IntakeCommands.fullIntakeSequence(intake)
            .withTimeout(timeoutSeconds)
            .withName("Auto Intake (timeout)");
    }

    // ================================================================
    // SHOOTER COMMANDS
    // ================================================================

    /**
     * Shoot all collected FUEL.
     *
     * @param shooter The shooter subsystem
     * @param intake The intake subsystem (for feeding)
     * @return Command that shoots all FUEL
     */
    public static Command shootAllFuel(Shooter shooter, Intake intake) {
        return Commands.sequence(
            // Spin up shooter
            Commands.runOnce(shooter::shootHub, shooter),
            Commands.waitSeconds(0.5), // Wait for shooter to spin up
            // Feed all FUEL
            IntakeCommands.feedAllCommand(intake, shooter),
            // Stop shooter
            Commands.waitSeconds(0.5)
        ).withName("Shoot All FUEL");
    }

    /**
     * Shoot a single FUEL.
     *
     * @param shooter The shooter subsystem
     * @param intake The intake subsystem
     * @return Command that shoots one FUEL
     */
    public static Command shootOneFuel(Shooter shooter, Intake intake) {
        return Commands.sequence(
            Commands.runOnce(shooter::shootHub, shooter),
            Commands.waitSeconds(0.5),
            IntakeCommands.feedOneCommand(intake),
            Commands.waitSeconds(0.3)
        ).withName("Shoot One FUEL");
    }

    // ================================================================
    // UTILITY COMMANDS
    // ================================================================

    /**
     * Wait for a specified duration.
     *
     * @param seconds Time to wait
     * @return Command that waits
     */
    public static Command waitSeconds(double seconds) {
        return Commands.waitSeconds(seconds)
            .withName("Wait " + seconds + "s");
    }

    /**
     * Log a message to the console.
     *
     * @param message Message to log
     * @return Instant command that prints the message
     */
    public static Command logMessage(String message) {
        return Commands.runOnce(() ->
            DriverStation.reportWarning("[AUTO] " + message, false)
        ).withName("Log: " + message);
    }

    /**
     * Set wheels to X pattern (ski stop) for a duration.
     *
     * @param swerve The swerve drive subsystem
     * @param seconds Duration to hold
     * @return Command that holds X pattern
     */
    public static Command holdPosition(SwerveDrive swerve, double seconds) {
        return SwerveCommands.skiStopCommand(swerve)
            .withTimeout(seconds)
            .withName("Hold Position");
    }

    /**
     * Stop all drive motors.
     *
     * @param swerve The swerve drive subsystem
     * @return Instant command that stops driving
     */
    public static Command stopDriving(SwerveDrive swerve) {
        return Commands.runOnce(() -> swerve.drive(new Translation2d(), 0, false, false), swerve)
            .withName("Stop Driving");
    }

    // ================================================================
    // HELPER METHODS
    // ================================================================

    /**
     * Check if robot is at a target pose within tolerances.
     *
     * @param swerve The swerve drive subsystem
     * @param target Target pose
     * @param positionTolerance Position tolerance in meters
     * @param angleTolerance Angle tolerance in degrees
     * @return True if at pose within tolerances
     */
    private static boolean isAtPose(SwerveDrive swerve, Pose2d target,
                                     double positionTolerance, double angleTolerance) {
        Pose2d current = swerve.getPose();
        double positionError = current.getTranslation().getDistance(target.getTranslation());
        double angleError = Math.abs(current.getRotation().minus(target.getRotation()).getDegrees());
        return positionError < positionTolerance && angleError < angleTolerance;
    }
}
