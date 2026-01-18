package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * Complete autonomous routines for the auto chooser.
 *
 * Each routine is a full autonomous sequence that can be selected
 * from the SmartDashboard auto chooser before a match.
 *
 * Available routines:
 * - doNothing: Safety default, does nothing
 * - driveForwardAuto: Simple forward drive
 * - driveAndIntakeAuto: Drive while collecting FUEL
 * - intakeAndScoreAuto: Full cycle with scoring
 * - twoFuelAuto: Collect and score multiple FUEL
 * - pathPlannerAuto: Follow pre-planned paths
 */
public final class AutoRoutines {

    // Default driving parameters
    private static final double DEFAULT_SPEED = 1.5; // m/s
    private static final double DEFAULT_DISTANCE = 2.0; // meters

    private AutoRoutines() {
        // Utility class - prevent instantiation
    }

    // ================================================================
    // SIMPLE ROUTINES
    // ================================================================

    /**
     * Do nothing auto - safety default.
     *
     * Use this when:
     * - Testing other robot systems
     * - Unsure what auto to run
     * - Just need to stay still
     *
     * @return Command that does nothing
     */
    public static Command doNothing() {
        return Commands.sequence(
            AutoCommands.logMessage("Do Nothing Auto Started"),
            Commands.waitSeconds(15.0)
        ).withName("Do Nothing");
    }

    /**
     * Simple forward drive auto.
     *
     * Drives forward 2 meters, useful for:
     * - Leaving the starting zone
     * - Simple mobility points
     *
     * @param swerve The swerve drive subsystem
     * @return Command that drives forward
     */
    public static Command driveForwardAuto(SwerveDrive swerve) {
        return Commands.sequence(
            AutoCommands.logMessage("Drive Forward Auto Started"),
            AutoCommands.driveForward(swerve, DEFAULT_DISTANCE, DEFAULT_SPEED),
            AutoCommands.stopDriving(swerve),
            AutoCommands.holdPosition(swerve, 1.0),
            AutoCommands.logMessage("Drive Forward Auto Complete")
        ).withName("Drive Forward Auto");
    }

    /**
     * Drive backward out of starting position.
     *
     * @param swerve The swerve drive subsystem
     * @return Command that drives backward
     */
    public static Command driveBackwardAuto(SwerveDrive swerve) {
        return Commands.sequence(
            AutoCommands.logMessage("Drive Backward Auto Started"),
            AutoCommands.driveBackward(swerve, DEFAULT_DISTANCE, DEFAULT_SPEED),
            AutoCommands.stopDriving(swerve),
            AutoCommands.holdPosition(swerve, 1.0)
        ).withName("Drive Backward Auto");
    }

    // ================================================================
    // INTAKE ROUTINES
    // ================================================================

    /**
     * Drive forward while intaking FUEL.
     *
     * Combines movement with intake for efficiency.
     *
     * @param swerve The swerve drive subsystem
     * @param intake The intake subsystem
     * @return Command that drives and intakes
     */
    public static Command driveAndIntakeAuto(SwerveDrive swerve, Intake intake) {
        return Commands.sequence(
            AutoCommands.logMessage("Drive and Intake Auto Started"),
            // Deploy intake first
            Commands.runOnce(intake::deploy, intake),
            Commands.waitUntil(intake::isDeployed),
            // Drive while intaking
            AutoCommands.intakeWhileDriving(intake, swerve, 3.0, 1.0),
            // Retract and stop
            Commands.runOnce(intake::retract, intake),
            AutoCommands.stopDriving(swerve),
            Commands.waitUntil(intake::isStowed),
            AutoCommands.logMessage("Collected " + intake.getFuelCount() + " FUEL")
        ).withName("Drive and Intake Auto");
    }

    /**
     * Intake FUEL, then drive to scoring position.
     *
     * @param swerve The swerve drive subsystem
     * @param intake The intake subsystem
     * @return Command for intake then position
     */
    public static Command intakeThenDriveAuto(SwerveDrive swerve, Intake intake) {
        return Commands.sequence(
            AutoCommands.logMessage("Intake Then Drive Auto Started"),
            // Collect FUEL
            AutoCommands.intakeFuelWithTimeout(intake, 3.0),
            // Drive to position
            AutoCommands.driveForward(swerve, 2.0, DEFAULT_SPEED),
            AutoCommands.stopDriving(swerve)
        ).withName("Intake Then Drive Auto");
    }

    // ================================================================
    // SCORING ROUTINES
    // ================================================================

    /**
     * Full intake and score cycle.
     *
     * 1. Drive forward to FUEL
     * 2. Intake FUEL
     * 3. Drive to scoring position
     * 4. Score FUEL
     *
     * @param swerve The swerve drive subsystem
     * @param intake The intake subsystem
     * @param shooter The shooter subsystem
     * @return Command for full cycle
     */
    public static Command intakeAndScoreAuto(SwerveDrive swerve, Intake intake, Shooter shooter) {
        return Commands.sequence(
            AutoCommands.logMessage("Intake and Score Auto Started"),
            // Drive to FUEL position
            AutoCommands.driveForward(swerve, 1.5, DEFAULT_SPEED),
            // Collect FUEL
            AutoCommands.intakeFuel(intake),
            // Drive to scoring position
            AutoCommands.driveBackward(swerve, 1.5, DEFAULT_SPEED),
            // Score
            AutoCommands.shootAllFuel(shooter, intake),
            AutoCommands.stopDriving(swerve),
            AutoCommands.logMessage("Intake and Score Auto Complete")
        ).withName("Intake and Score Auto");
    }

    /**
     * Two FUEL auto - collect and score two pieces.
     *
     * More complex auto that:
     * 1. Scores preloaded FUEL
     * 2. Drives to collect another
     * 3. Returns and scores again
     *
     * @param swerve The swerve drive subsystem
     * @param intake The intake subsystem
     * @param shooter The shooter subsystem
     * @return Command for two piece auto
     */
    public static Command twoFuelAuto(SwerveDrive swerve, Intake intake, Shooter shooter) {
        return Commands.sequence(
            AutoCommands.logMessage("Two FUEL Auto Started"),

            // Score preloaded FUEL
            AutoCommands.logMessage("Scoring preloaded FUEL"),
            AutoCommands.shootOneFuel(shooter, intake),

            // Drive to first FUEL
            AutoCommands.logMessage("Driving to first FUEL"),
            AutoCommands.driveForward(swerve, 2.0, DEFAULT_SPEED),

            // Collect FUEL
            AutoCommands.intakeFuelWithTimeout(intake, 2.0),

            // Return to scoring position
            AutoCommands.logMessage("Returning to score"),
            AutoCommands.driveBackward(swerve, 2.0, DEFAULT_SPEED),

            // Score second FUEL
            AutoCommands.logMessage("Scoring second FUEL"),
            AutoCommands.shootAllFuel(shooter, intake),

            // Finish
            AutoCommands.stopDriving(swerve),
            AutoCommands.holdPosition(swerve, 1.0),
            AutoCommands.logMessage("Two FUEL Auto Complete")
        ).withName("Two FUEL Auto");
    }

    /**
     * Three FUEL auto - ambitious multi-piece auto.
     *
     * @param swerve The swerve drive subsystem
     * @param intake The intake subsystem
     * @param shooter The shooter subsystem
     * @return Command for three piece auto
     */
    public static Command threeFuelAuto(SwerveDrive swerve, Intake intake, Shooter shooter) {
        return Commands.sequence(
            AutoCommands.logMessage("Three FUEL Auto Started"),

            // Score preloaded
            AutoCommands.shootOneFuel(shooter, intake),

            // First cycle
            AutoCommands.driveForward(swerve, 2.0, 2.0),
            AutoCommands.intakeFuelWithTimeout(intake, 1.5),
            AutoCommands.driveBackward(swerve, 2.0, 2.0),
            AutoCommands.shootOneFuel(shooter, intake),

            // Second cycle
            AutoCommands.strafeLeft(swerve, 1.0, 1.5),
            AutoCommands.driveForward(swerve, 2.0, 2.0),
            AutoCommands.intakeFuelWithTimeout(intake, 1.5),
            AutoCommands.driveBackward(swerve, 2.0, 2.0),
            AutoCommands.strafeRight(swerve, 1.0, 1.5),
            AutoCommands.shootAllFuel(shooter, intake),

            AutoCommands.stopDriving(swerve),
            AutoCommands.logMessage("Three FUEL Auto Complete")
        ).withName("Three FUEL Auto");
    }

    // ================================================================
    // PATH PLANNER ROUTINES
    // ================================================================

    /**
     * PathPlanner-based auto routine.
     *
     * Follows a pre-planned path from PathPlanner.
     * Note: Requires PathPlanner to be configured.
     *
     * @param swerve The swerve drive subsystem
     * @param pathName Name of the PathPlanner path file
     * @return Command that follows the path
     */
    public static Command pathPlannerAuto(SwerveDrive swerve, String pathName) {
        return Commands.sequence(
            AutoCommands.logMessage("PathPlanner Auto: " + pathName),
            AutoCommands.followPath(pathName),
            AutoCommands.stopDriving(swerve)
        ).withName("PP: " + pathName);
    }

    /**
     * PathPlanner auto with intake during path.
     *
     * @param swerve The swerve drive subsystem
     * @param intake The intake subsystem
     * @param pathName Name of the PathPlanner path file
     * @return Command that follows path while intaking
     */
    public static Command pathPlannerWithIntakeAuto(SwerveDrive swerve, Intake intake, String pathName) {
        return Commands.sequence(
            AutoCommands.logMessage("PathPlanner + Intake Auto: " + pathName),
            Commands.parallel(
                AutoCommands.followPath(pathName),
                Commands.sequence(
                    Commands.runOnce(intake::deploy, intake),
                    Commands.waitUntil(intake::isDeployed),
                    Commands.run(intake::runIntake, intake)
                )
            ),
            Commands.runOnce(intake::stopRollers, intake),
            Commands.runOnce(intake::retract, intake),
            AutoCommands.stopDriving(swerve)
        ).withName("PP + Intake: " + pathName);
    }

    // ================================================================
    // POSITION-BASED ROUTINES
    // ================================================================

    /**
     * Drive to a specific field position.
     *
     * @param swerve The swerve drive subsystem
     * @param x Target X coordinate (meters)
     * @param y Target Y coordinate (meters)
     * @param headingDegrees Target heading (degrees)
     * @return Command that drives to position
     */
    public static Command driveToPositionAuto(SwerveDrive swerve,
                                               double x, double y, double headingDegrees) {
        Pose2d targetPose = new Pose2d(x, y, Rotation2d.fromDegrees(headingDegrees));
        return Commands.sequence(
            AutoCommands.logMessage("Driving to (" + x + ", " + y + ")"),
            AutoCommands.driveToPose(swerve, targetPose),
            AutoCommands.stopDriving(swerve),
            AutoCommands.holdPosition(swerve, 0.5)
        ).withName("Drive to Position Auto");
    }
}
