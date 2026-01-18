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

    // ================================================================
    // COMPETITION AUTO ROUTINES (DIP Switch Selectable)
    // ================================================================
    // These are the primary autonomous routines designed for competition.
    // Selected via physical DIP switch on the robot.
    //
    // Switch 0 (00): Do Nothing - Safety mode
    // Switch 1 (01): Score & Collect - Offensive, maximize FUEL
    // Switch 2 (10): Quick Climb - Defensive, guaranteed 15 pts
    // Switch 3 (11): Score Then Climb - Maximum points potential
    // ================================================================

    /**
     * STRATEGY 1: Score & Collect (Offensive)
     *
     * Goal: Maximize FUEL scored to win AUTO phase.
     *
     * Sequence:
     * 1. Score all 8 preloaded FUEL (~8 seconds)
     * 2. Drive to neutral zone (~4 seconds)
     * 3. Intake additional FUEL (~5 seconds)
     * 4. Score collected FUEL if time permits (~3 seconds)
     *
     * Expected Points: 8-12+ points from FUEL
     * Risk: Medium (depends on shooting accuracy)
     * Best When: Shooter is reliable, want to control hub shift timing
     *
     * @param swerve The swerve drive subsystem
     * @param intake The intake subsystem
     * @param shooter The shooter subsystem
     * @return Command for Score & Collect auto
     */
    public static Command scoreAndCollectAuto(SwerveDrive swerve, Intake intake, Shooter shooter) {
        return Commands.sequence(
            AutoCommands.logMessage("=== SCORE & COLLECT AUTO ==="),

            // Phase 1: Score all preloaded FUEL (~8 seconds)
            AutoCommands.logMessage("Phase 1: Scoring preloaded FUEL"),
            AutoCommands.shootAllFuel(shooter, intake),

            // Phase 2: Drive to neutral zone (~4 seconds)
            AutoCommands.logMessage("Phase 2: Driving to neutral zone"),
            AutoCommands.driveForward(swerve, 3.0, 2.0),

            // Phase 3: Collect more FUEL (~5 seconds)
            AutoCommands.logMessage("Phase 3: Collecting FUEL"),
            Commands.parallel(
                // Continue driving slowly while intaking
                AutoCommands.driveForward(swerve, 1.5, 1.0),
                Commands.sequence(
                    Commands.runOnce(intake::deploy, intake),
                    Commands.waitUntil(intake::isDeployed),
                    Commands.run(intake::runIntake, intake)
                )
            ).withTimeout(5.0),

            // Stop intake and retract
            Commands.runOnce(intake::stopRollers, intake),
            Commands.runOnce(intake::retract, intake),

            // Phase 4: Score additional FUEL if collected (remaining time)
            AutoCommands.logMessage("Phase 4: Scoring collected FUEL"),
            Commands.either(
                // If we have FUEL, score it
                Commands.sequence(
                    AutoCommands.driveBackward(swerve, 2.0, 2.5),
                    AutoCommands.shootAllFuel(shooter, intake)
                ),
                // If no FUEL, just stop
                AutoCommands.stopDriving(swerve),
                () -> intake.getFuelCount() > 0
            ),

            AutoCommands.stopDriving(swerve),
            AutoCommands.logMessage("=== SCORE & COLLECT COMPLETE ===")
        ).withName("1: Score & Collect");
    }

    /**
     * STRATEGY 2: Quick Climb (Defensive/Guaranteed)
     *
     * Goal: Secure reliable 15 points with L1 climb.
     *
     * Sequence:
     * 1. Drive directly to TOWER (~5 seconds)
     * 2. Climb to LEVEL 1 (~10 seconds)
     * 3. Hold position (remaining time)
     *
     * Expected Points: 15 points (guaranteed)
     * Risk: Low (no shooting required)
     * Best When: Shooter unreliable, climber reliable, or alliance partner scoring FUEL
     *
     * NOTE: Only 2 robots per alliance can earn L1 points in AUTO.
     * Coordinate with alliance partners!
     *
     * @param swerve The swerve drive subsystem
     * @param climber The climber subsystem
     * @return Command for Quick Climb auto
     */
    public static Command quickClimbAuto(SwerveDrive swerve, frc.robot.subsystems.Climber climber) {
        return Commands.sequence(
            AutoCommands.logMessage("=== QUICK CLIMB AUTO ==="),

            // Phase 1: Drive to tower (~5 seconds)
            AutoCommands.logMessage("Phase 1: Driving to TOWER"),
            AutoCommands.driveForward(swerve, 2.5, 2.0),
            AutoCommands.stopDriving(swerve),

            // Phase 2: Climb to L1 (~10 seconds)
            AutoCommands.logMessage("Phase 2: Climbing to LEVEL 1"),
            Commands.runOnce(climber::climbToL1, climber),
            Commands.waitUntil(climber::isAtL1).withTimeout(12.0),

            // Phase 3: Hold position
            AutoCommands.logMessage("Phase 3: Holding at L1"),
            Commands.runOnce(climber::stop, climber),

            AutoCommands.logMessage("=== QUICK CLIMB COMPLETE (15 pts) ===")
        ).withName("2: Quick Climb");
    }

    /**
     * STRATEGY 3: Score Then Climb (Maximum Points)
     *
     * Goal: Get both FUEL points AND climb bonus for maximum score.
     *
     * Sequence:
     * 1. Rapid-fire preloaded FUEL (~6 seconds)
     * 2. Drive to TOWER (~4 seconds)
     * 3. Climb to LEVEL 1 (~10 seconds)
     *
     * Expected Points: 19-23 points (4-8 FUEL + 15 climb)
     * Risk: High (time-critical, both systems must work)
     * Best When: Robot is well-tuned and practiced
     *
     * @param swerve The swerve drive subsystem
     * @param intake The intake subsystem
     * @param shooter The shooter subsystem
     * @param climber The climber subsystem
     * @return Command for Score Then Climb auto
     */
    public static Command scoreThenClimbAuto(SwerveDrive swerve, Intake intake,
                                              Shooter shooter, frc.robot.subsystems.Climber climber) {
        return Commands.sequence(
            AutoCommands.logMessage("=== SCORE THEN CLIMB AUTO ==="),

            // Phase 1: Rapid-fire FUEL (~6 seconds)
            // Only shoot for limited time to save time for climb
            AutoCommands.logMessage("Phase 1: Rapid scoring (6 sec max)"),
            Commands.race(
                AutoCommands.shootAllFuel(shooter, intake),
                Commands.waitSeconds(6.0)
            ),

            // Phase 2: Drive to tower (~4 seconds)
            AutoCommands.logMessage("Phase 2: Driving to TOWER"),
            AutoCommands.driveForward(swerve, 2.5, 2.5),  // Fast drive
            AutoCommands.stopDriving(swerve),

            // Phase 3: Climb to L1 (~10 seconds)
            AutoCommands.logMessage("Phase 3: Climbing to LEVEL 1"),
            Commands.runOnce(climber::climbToL1, climber),
            Commands.waitUntil(climber::isAtL1).withTimeout(10.0),

            // Done
            Commands.runOnce(climber::stop, climber),
            AutoCommands.logMessage("=== SCORE THEN CLIMB COMPLETE ===")
        ).withName("3: Score Then Climb");
    }

    /**
     * Get the appropriate auto command based on DIP switch selection.
     *
     * @param selection The DIP switch value (0-3)
     * @param swerve The swerve drive subsystem
     * @param intake The intake subsystem
     * @param shooter The shooter subsystem
     * @param climber The climber subsystem
     * @return The selected autonomous command
     */
    public static Command getAutoFromSelection(int selection, SwerveDrive swerve,
                                                Intake intake, Shooter shooter,
                                                frc.robot.subsystems.Climber climber) {
        switch (selection) {
            case 1:
                return scoreAndCollectAuto(swerve, intake, shooter);
            case 2:
                return quickClimbAuto(swerve, climber);
            case 3:
                return scoreThenClimbAuto(swerve, intake, shooter, climber);
            case 0:
            default:
                return doNothing();
        }
    }
}
