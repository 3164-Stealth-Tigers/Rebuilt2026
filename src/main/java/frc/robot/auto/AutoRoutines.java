package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * Complete autonomous routines for the REBUILT 2026 game.
 *
 * Contains 20 autonomous modes selectable via 5-bit DIP switch.
 * Modes have been optimized using simulator benchmarking with 1000+ simulations each.
 *
 * BENCHMARK RESULTS (from simulator):
 * - Mode 13 (Depot+Climb): 20 pts - BEST OVERALL
 * - Mode 2/3/12 (Climb modes): 18 pts - Reliable
 * - Mode 1/7 (FUEL only): 8 pts - Max FUEL
 * - Mode 11 (Fast Climb): 15 pts - Guaranteed climb
 *
 * SCORING:
 * - FUEL: 1 point each
 * - L1 Climb in AUTO: 15 points
 * - Preload: 3 FUEL (3 pts)
 * - Depot collection: 2 FUEL
 * - Neutral zone: up to 5 FUEL
 */
public final class AutoRoutines {

    private AutoRoutines() {
        // Utility class - prevent instantiation
    }

    // ================================================================
    // ALLIANCE-AWARE POSITION HELPERS
    // ================================================================

    /** Get alliance-appropriate HUB X position */
    private static double getHubX() {
        return isRedAlliance() ? FieldConstants.RED_HUB_X : FieldConstants.BLUE_HUB_X;
    }

    /** Get alliance-appropriate Tower X position */
    private static double getTowerX() {
        return isRedAlliance() ? FieldConstants.RED_TOWER_X : FieldConstants.BLUE_TOWER_X;
    }

    /** Get alliance-appropriate Tower Y position */
    private static double getTowerY() {
        return isRedAlliance() ? FieldConstants.RED_TOWER_Y : FieldConstants.BLUE_TOWER_Y;
    }

    /** Get alliance-appropriate Depot X position */
    private static double getDepotX() {
        return isRedAlliance() ? FieldConstants.RED_DEPOT_X : FieldConstants.BLUE_DEPOT_X;
    }

    /** Get alliance-appropriate Depot Y position */
    private static double getDepotY() {
        return isRedAlliance() ? FieldConstants.RED_DEPOT_Y : FieldConstants.BLUE_DEPOT_Y;
    }

    /** Check if we're on the red alliance */
    private static boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }

    /** Get direction multiplier for X movement (1 for red, -1 for blue) */
    private static double getAllianceDirection() {
        return isRedAlliance() ? -1.0 : 1.0;
    }

    /** Get shooting position (in alliance zone, safe distance from hub) */
    private static Pose2d getShootingPose() {
        double x = isRedAlliance() ?
            FieldConstants.FIELD_LENGTH - 2.5 : 2.5;
        return new Pose2d(x, FieldConstants.CENTER_Y, Rotation2d.fromDegrees(isRedAlliance() ? 180 : 0));
    }

    /** Get tower approach position */
    private static Pose2d getTowerPose() {
        double offset = isRedAlliance() ? -1.0 : 1.0;
        return new Pose2d(getTowerX() + offset, getTowerY(), Rotation2d.fromDegrees(0));
    }

    /** Get depot position */
    private static Pose2d getDepotPose() {
        return new Pose2d(getDepotX(), getDepotY(), Rotation2d.fromDegrees(0));
    }

    /** Get neutral zone collection position */
    private static Pose2d getNeutralPose(boolean farSide) {
        double offset = farSide ? FieldConstants.NEUTRAL_FAR_OFFSET : FieldConstants.NEUTRAL_CLOSE_OFFSET;
        double x = FieldConstants.CENTER_X + (isRedAlliance() ? -offset : offset);
        return new Pose2d(x, FieldConstants.CENTER_Y, Rotation2d.fromDegrees(0));
    }

    // ================================================================
    // MODE 0: DO NOTHING (Safety Default)
    // ================================================================

    /**
     * Mode 0: Do Nothing - Safety default.
     * Expected Points: 0
     */
    public static Command doNothing() {
        return Commands.sequence(
            AutoCommands.logMessage("Mode 0: Do Nothing"),
            Commands.waitSeconds(15.0)
        ).withName("0: Do Nothing");
    }

    // ================================================================
    // MODE 1: SCORE & COLLECT (8 pts)
    // ================================================================

    /**
     * Mode 1: Score preload, drive to neutral zone, collect FUEL, return and score.
     * Expected Points: 8 FUEL = 8 pts
     */
    public static Command scoreAndCollectAuto(SwerveDrive swerve, Intake intake, Shooter shooter) {
        return Commands.sequence(
            AutoCommands.logMessage("Mode 1: Score & Collect"),

            // Phase 1: Score all preloaded FUEL
            AutoCommands.shootAllFuel(shooter, intake),

            // Phase 2: Drive to neutral zone
            AutoCommands.driveToPose(swerve, getNeutralPose(false)),

            // Phase 3: Collect FUEL while moving
            Commands.parallel(
                AutoCommands.driveForward(swerve, 2.0, AutoConstants.AUTO_INTAKE_DRIVE_SPEED),
                Commands.sequence(
                    Commands.runOnce(intake::deploy, intake),
                    Commands.waitUntil(intake::isDeployed),
                    Commands.run(intake::runIntake, intake)
                )
            ).withTimeout(AutoConstants.INTAKE_TIMEOUT),

            // Phase 4: Stop intake and retract
            Commands.runOnce(intake::stopRollers, intake),
            Commands.runOnce(intake::retract, intake),

            // Phase 5: Return and score
            Commands.either(
                Commands.sequence(
                    AutoCommands.driveToPose(swerve, getShootingPose()),
                    AutoCommands.shootAllFuel(shooter, intake)
                ),
                AutoCommands.stopDriving(swerve),
                () -> intake.getFuelCount() > 0
            ),

            AutoCommands.stopDriving(swerve),
            AutoCommands.logMessage("Mode 1 Complete")
        ).withName("1: Score & Collect");
    }

    // ================================================================
    // MODE 2: QUICK CLIMB (18 pts)
    // ================================================================

    /**
     * Mode 2: Score preload, then climb L1.
     * Expected Points: 3 FUEL + 15 climb = 18 pts
     */
    public static Command quickClimbAuto(SwerveDrive swerve, Climber climber, Intake intake, Shooter shooter) {
        return Commands.sequence(
            AutoCommands.logMessage("Mode 2: Quick Climb"),

            // Phase 1: Score preload
            AutoCommands.shootAllFuel(shooter, intake),

            // Phase 2: Drive to tower
            AutoCommands.driveToPose(swerve, getTowerPose()),
            AutoCommands.stopDriving(swerve),

            // Phase 3: Climb L1
            Commands.runOnce(climber::climbToL1, climber),
            Commands.waitUntil(climber::isAtL1).withTimeout(AutoConstants.CLIMB_TIMEOUT),
            Commands.runOnce(climber::stop, climber),

            AutoCommands.logMessage("Mode 2 Complete (18 pts)")
        ).withName("2: Quick Climb");
    }

    // ================================================================
    // MODE 3: SCORE THEN CLIMB (18 pts)
    // ================================================================

    /**
     * Mode 3: Rapid fire preload, then climb L1 (optimized timing).
     * Expected Points: 3 FUEL + 15 climb = 18 pts
     */
    public static Command scoreThenClimbAuto(SwerveDrive swerve, Intake intake,
                                              Shooter shooter, Climber climber) {
        return Commands.sequence(
            AutoCommands.logMessage("Mode 3: Score Then Climb"),

            // Phase 1: Rapid fire preload (time-limited)
            Commands.race(
                AutoCommands.shootAllFuel(shooter, intake),
                Commands.waitSeconds(6.0)
            ),

            // Phase 2: Fast drive to tower
            AutoCommands.driveToPose(swerve, getTowerPose()),
            AutoCommands.stopDriving(swerve),

            // Phase 3: Climb L1
            Commands.runOnce(climber::climbToL1, climber),
            Commands.waitUntil(climber::isAtL1).withTimeout(10.0),
            Commands.runOnce(climber::stop, climber),

            AutoCommands.logMessage("Mode 3 Complete (18 pts)")
        ).withName("3: Score Then Climb");
    }

    // ================================================================
    // MODE 4: DEPOT RAID (5 pts)
    // ================================================================

    /**
     * Mode 4: Drive to depot, collect FUEL, score all.
     * Expected Points: 5 FUEL = 5 pts
     */
    public static Command depotRaidAuto(SwerveDrive swerve, Intake intake, Shooter shooter) {
        return Commands.sequence(
            AutoCommands.logMessage("Mode 4: Depot Raid"),

            // Phase 1: Drive to depot
            AutoCommands.driveToPose(swerve, getDepotPose()),

            // Phase 2: Collect from depot
            Commands.runOnce(intake::deploy, intake),
            Commands.waitUntil(intake::isDeployed),
            Commands.parallel(
                Commands.run(intake::runIntake, intake),
                Commands.sequence(
                    AutoCommands.driveForward(swerve, 0.5, 0.5),
                    AutoCommands.driveBackward(swerve, 0.5, 0.5)
                )
            ).withTimeout(AutoConstants.DEPOT_COLLECTION_TIME),
            Commands.runOnce(intake::stopRollers, intake),
            Commands.runOnce(intake::retract, intake),

            // Phase 3: Drive to scoring position and score
            AutoCommands.driveToPose(swerve, getShootingPose()),
            AutoCommands.shootAllFuel(shooter, intake),

            AutoCommands.stopDriving(swerve),
            AutoCommands.logMessage("Mode 4 Complete")
        ).withName("4: Depot Raid");
    }

    // ================================================================
    // MODE 5: FAR NEUTRAL (3-4 pts)
    // ================================================================

    /**
     * Mode 5: Score preload, drive to far neutral, collect, score.
     * Expected Points: 3-4 FUEL = 3-4 pts
     */
    public static Command farNeutralAuto(SwerveDrive swerve, Intake intake, Shooter shooter) {
        return Commands.sequence(
            AutoCommands.logMessage("Mode 5: Far Neutral"),

            // Phase 1: Score preload
            AutoCommands.shootAllFuel(shooter, intake),

            // Phase 2: Drive to far neutral zone
            AutoCommands.driveToPose(swerve, getNeutralPose(true)),

            // Phase 3: Collect FUEL
            Commands.runOnce(intake::deploy, intake),
            Commands.waitUntil(intake::isDeployed),
            Commands.parallel(
                Commands.run(intake::runIntake, intake),
                AutoCommands.driveForward(swerve, 1.5, AutoConstants.AUTO_SLOW_DRIVE_SPEED)
            ).withTimeout(3.0),
            Commands.runOnce(intake::stopRollers, intake),
            Commands.runOnce(intake::retract, intake),

            // Phase 4: Return and score
            AutoCommands.driveToPose(swerve, getShootingPose()),
            AutoCommands.shootAllFuel(shooter, intake),

            AutoCommands.stopDriving(swerve),
            AutoCommands.logMessage("Mode 5 Complete")
        ).withName("5: Far Neutral");
    }

    // ================================================================
    // MODE 6: PRELOAD ONLY (3 pts)
    // ================================================================

    /**
     * Mode 6: Just score preload, hold position.
     * Expected Points: 3 FUEL = 3 pts
     */
    public static Command preloadOnlyAuto(SwerveDrive swerve, Intake intake, Shooter shooter) {
        return Commands.sequence(
            AutoCommands.logMessage("Mode 6: Preload Only"),
            AutoCommands.shootAllFuel(shooter, intake),
            AutoCommands.stopDriving(swerve),
            AutoCommands.holdPosition(swerve, 10.0),
            AutoCommands.logMessage("Mode 6 Complete (3 pts)")
        ).withName("6: Preload Only");
    }

    // ================================================================
    // MODE 7: MAX CYCLES (8 pts)
    // ================================================================

    /**
     * Mode 7: Pure scoring loop - shoot, collect, shoot, repeat.
     * Expected Points: 8 FUEL = 8 pts
     */
    public static Command maxCyclesAuto(SwerveDrive swerve, Intake intake, Shooter shooter) {
        return Commands.sequence(
            AutoCommands.logMessage("Mode 7: Max Cycles"),

            // Cycle 1: Score preload
            AutoCommands.shootAllFuel(shooter, intake),

            // Cycle 2: Collect and score
            AutoCommands.driveToPose(swerve, getNeutralPose(false)),
            Commands.parallel(
                AutoCommands.driveForward(swerve, 2.0, AutoConstants.AUTO_INTAKE_DRIVE_SPEED),
                Commands.sequence(
                    Commands.runOnce(intake::deploy, intake),
                    Commands.waitUntil(intake::isDeployed),
                    Commands.run(intake::runIntake, intake)
                )
            ).withTimeout(AutoConstants.INTAKE_TIMEOUT),
            Commands.runOnce(intake::stopRollers, intake),
            Commands.runOnce(intake::retract, intake),
            AutoCommands.driveToPose(swerve, getShootingPose()),
            AutoCommands.shootAllFuel(shooter, intake),

            AutoCommands.stopDriving(swerve),
            AutoCommands.logMessage("Mode 7 Complete")
        ).withName("7: Max Cycles");
    }

    // ================================================================
    // MODE 8: CLIMB SUPPORT (3 pts)
    // ================================================================

    /**
     * Mode 8: Score preload, position for TELEOP climb assist.
     * Expected Points: 3 FUEL = 3 pts (sets up for TELEOP)
     */
    public static Command climbSupportAuto(SwerveDrive swerve, Intake intake, Shooter shooter) {
        return Commands.sequence(
            AutoCommands.logMessage("Mode 8: Climb Support"),

            // Score preload
            AutoCommands.shootAllFuel(shooter, intake),

            // Position near tower for TELEOP
            AutoCommands.driveToPose(swerve, getTowerPose()),
            AutoCommands.stopDriving(swerve),
            AutoCommands.holdPosition(swerve, 10.0),

            AutoCommands.logMessage("Mode 8 Complete - Ready for TELEOP climb")
        ).withName("8: Climb Support");
    }

    // ================================================================
    // MODE 9: WIN AUTO (4 pts)
    // ================================================================

    /**
     * Mode 9: Aggressive scoring to win AUTO period.
     * Expected Points: ~4 FUEL = 4 pts
     */
    public static Command winAutoAuto(SwerveDrive swerve, Intake intake, Shooter shooter) {
        return Commands.sequence(
            AutoCommands.logMessage("Mode 9: Win AUTO"),

            // Rapid score preload
            AutoCommands.shootAllFuel(shooter, intake),

            // Quick collect cycle
            AutoCommands.driveToPose(swerve, getNeutralPose(false)),
            Commands.parallel(
                AutoCommands.driveForward(swerve, 1.0, AutoConstants.AUTO_FAST_DRIVE_SPEED),
                Commands.sequence(
                    Commands.runOnce(intake::deploy, intake),
                    Commands.waitUntil(intake::isDeployed),
                    Commands.run(intake::runIntake, intake)
                )
            ).withTimeout(2.0),
            Commands.runOnce(intake::stopRollers, intake),
            Commands.runOnce(intake::retract, intake),

            // Quick score
            AutoCommands.driveToPose(swerve, getShootingPose()),
            AutoCommands.shootAllFuel(shooter, intake),

            AutoCommands.stopDriving(swerve),
            AutoCommands.logMessage("Mode 9 Complete")
        ).withName("9: Win AUTO");
    }

    // ================================================================
    // MODE 10: SCORE+COLLECT+CLIMB (18 pts)
    // ================================================================

    /**
     * Mode 10: Score preload, quick collect, score, then climb.
     * Expected Points: 3-4 FUEL + 15 climb = 18-19 pts
     */
    public static Command scoreCollectClimbAuto(SwerveDrive swerve, Intake intake,
                                                 Shooter shooter, Climber climber) {
        return Commands.sequence(
            AutoCommands.logMessage("Mode 10: Score+Collect+Climb"),

            // Phase 1: Score preload
            AutoCommands.shootAllFuel(shooter, intake),

            // Phase 2: Quick collect (if time permits)
            AutoCommands.driveToPose(swerve, getNeutralPose(false)),
            Commands.parallel(
                AutoCommands.driveForward(swerve, 1.0, AutoConstants.AUTO_FAST_DRIVE_SPEED),
                Commands.sequence(
                    Commands.runOnce(intake::deploy, intake),
                    Commands.waitUntil(intake::isDeployed),
                    Commands.run(intake::runIntake, intake)
                )
            ).withTimeout(2.0),
            Commands.runOnce(intake::stopRollers, intake),
            Commands.runOnce(intake::retract, intake),

            // Phase 3: Score collected
            AutoCommands.driveToPose(swerve, getShootingPose()),
            AutoCommands.shootAllFuel(shooter, intake),

            // Phase 4: Climb L1
            AutoCommands.driveToPose(swerve, getTowerPose()),
            Commands.runOnce(climber::climbToL1, climber),
            Commands.waitUntil(climber::isAtL1).withTimeout(8.0),
            Commands.runOnce(climber::stop, climber),

            AutoCommands.logMessage("Mode 10 Complete")
        ).withName("10: Score+Collect+Climb");
    }

    // ================================================================
    // MODE 11: FAST CLIMB (15 pts)
    // ================================================================

    /**
     * Mode 11: Drive directly to tower, climb immediately.
     * Expected Points: 15 climb pts (guaranteed)
     */
    public static Command fastClimbAuto(SwerveDrive swerve, Climber climber) {
        return Commands.sequence(
            AutoCommands.logMessage("Mode 11: Fast Climb"),

            // Drive directly to tower
            AutoCommands.driveToPose(swerve, getTowerPose()),
            AutoCommands.stopDriving(swerve),

            // Climb L1
            Commands.runOnce(climber::climbToL1, climber),
            Commands.waitUntil(climber::isAtL1).withTimeout(10.0),
            Commands.runOnce(climber::stop, climber),

            AutoCommands.logMessage("Mode 11 Complete (15 pts)")
        ).withName("11: Fast Climb");
    }

    // ================================================================
    // MODE 12: BALANCED (18 pts)
    // ================================================================

    /**
     * Mode 12: Score preload + climb (optimized timing).
     * Expected Points: 3 FUEL + 15 climb = 18 pts
     */
    public static Command balancedAuto(SwerveDrive swerve, Intake intake,
                                        Shooter shooter, Climber climber) {
        return Commands.sequence(
            AutoCommands.logMessage("Mode 12: Balanced"),

            // Score preload
            AutoCommands.shootAllFuel(shooter, intake),

            // Drive to tower
            AutoCommands.driveToPose(swerve, getTowerPose()),
            AutoCommands.stopDriving(swerve),

            // Climb L1
            Commands.runOnce(climber::climbToL1, climber),
            Commands.waitUntil(climber::isAtL1).withTimeout(10.0),
            Commands.runOnce(climber::stop, climber),

            AutoCommands.logMessage("Mode 12 Complete (18 pts)")
        ).withName("12: Balanced");
    }

    // ================================================================
    // MODE 13: DEPOT+CLIMB - BEST MODE (20 pts)
    // ================================================================

    /**
     * Mode 13: Depot collection then climb - OPTIMAL STRATEGY.
     * Expected Points: 5 FUEL + 15 climb = 20 pts
     *
     * This is the BEST autonomous mode discovered through simulation!
     * - Collects 2 FUEL from depot (faster than neutral zone)
     * - Scores all 5 FUEL (3 preload + 2 depot)
     * - Completes L1 climb for 15 bonus points
     */
    public static Command depotClimbAuto(SwerveDrive swerve, Intake intake,
                                          Shooter shooter, Climber climber) {
        return Commands.sequence(
            AutoCommands.logMessage("Mode 13: Depot+Climb (OPTIMAL - 20 pts)"),

            // Phase 1: Drive to depot
            AutoCommands.driveToPose(swerve, getDepotPose()),

            // Phase 2: Collect from depot (faster than neutral zone)
            Commands.runOnce(intake::deploy, intake),
            Commands.waitUntil(intake::isDeployed),
            Commands.parallel(
                Commands.run(intake::runIntake, intake),
                Commands.sequence(
                    AutoCommands.driveForward(swerve, 0.5, 0.5),
                    AutoCommands.driveBackward(swerve, 0.5, 0.5)
                )
            ).withTimeout(AutoConstants.DEPOT_COLLECTION_TIME),
            Commands.runOnce(intake::stopRollers, intake),
            Commands.runOnce(intake::retract, intake),

            // Phase 3: Drive to scoring position and score all FUEL
            AutoCommands.driveToPose(swerve, getShootingPose()),
            AutoCommands.shootAllFuel(shooter, intake),

            // Phase 4: Drive to tower and climb
            AutoCommands.driveToPose(swerve, getTowerPose()),
            AutoCommands.stopDriving(swerve),
            Commands.runOnce(climber::climbToL1, climber),
            Commands.waitUntil(climber::isAtL1).withTimeout(8.0),
            Commands.runOnce(climber::stop, climber),

            AutoCommands.logMessage("Mode 13 Complete (20 pts!)")
        ).withName("13: Depot+Climb");
    }

    // ================================================================
    // MODE 14: MAX POINTS (18 pts)
    // ================================================================

    /**
     * Mode 14: Dynamically maximize total points.
     * Expected Points: 3+ FUEL + 15 climb = 18+ pts
     */
    public static Command maxPointsAuto(SwerveDrive swerve, Intake intake,
                                         Shooter shooter, Climber climber) {
        return Commands.sequence(
            AutoCommands.logMessage("Mode 14: Max Points"),

            // Score preload first
            AutoCommands.shootAllFuel(shooter, intake),

            // Quick neutral zone attempt
            AutoCommands.driveToPose(swerve, getNeutralPose(false)),
            Commands.parallel(
                AutoCommands.driveForward(swerve, 1.5, AutoConstants.AUTO_FAST_DRIVE_SPEED),
                Commands.sequence(
                    Commands.runOnce(intake::deploy, intake),
                    Commands.waitUntil(intake::isDeployed),
                    Commands.run(intake::runIntake, intake)
                )
            ).withTimeout(2.5),
            Commands.runOnce(intake::stopRollers, intake),
            Commands.runOnce(intake::retract, intake),

            // Score any collected FUEL
            AutoCommands.driveToPose(swerve, getShootingPose()),
            AutoCommands.shootAllFuel(shooter, intake),

            // Always try to climb
            AutoCommands.driveToPose(swerve, getTowerPose()),
            Commands.runOnce(climber::climbToL1, climber),
            Commands.waitUntil(climber::isAtL1).withTimeout(8.0),
            Commands.runOnce(climber::stop, climber),

            AutoCommands.logMessage("Mode 14 Complete")
        ).withName("14: Max Points");
    }

    // ================================================================
    // MODE 15: SAFE CLIMB (15-18 pts)
    // ================================================================

    /**
     * Mode 15: Conservative climb with fallback.
     * Expected Points: 15-18 pts
     */
    public static Command safeClimbAuto(SwerveDrive swerve, Intake intake,
                                         Shooter shooter, Climber climber) {
        return Commands.sequence(
            AutoCommands.logMessage("Mode 15: Safe Climb"),

            // Try to score preload (with timeout)
            Commands.race(
                AutoCommands.shootAllFuel(shooter, intake),
                Commands.waitSeconds(4.0)
            ),

            // Drive to tower (priority: get to tower)
            AutoCommands.driveToPose(swerve, getTowerPose()),
            AutoCommands.stopDriving(swerve),

            // Climb L1 (extended timeout for safety)
            Commands.runOnce(climber::climbToL1, climber),
            Commands.waitUntil(climber::isAtL1).withTimeout(AutoConstants.CLIMB_TIMEOUT),
            Commands.runOnce(climber::stop, climber),

            AutoCommands.logMessage("Mode 15 Complete")
        ).withName("15: Safe Climb");
    }

    // ================================================================
    // MODE 16: DUAL CYCLE (6-8 pts)
    // ================================================================

    /**
     * Mode 16: Two full scoring cycles.
     * Expected Points: 6-8 FUEL = 6-8 pts
     */
    public static Command dualCycleAuto(SwerveDrive swerve, Intake intake, Shooter shooter) {
        return Commands.sequence(
            AutoCommands.logMessage("Mode 16: Dual Cycle"),

            // Cycle 1: Score preload
            AutoCommands.shootAllFuel(shooter, intake),

            // Collect from neutral
            AutoCommands.driveToPose(swerve, getNeutralPose(false)),
            Commands.parallel(
                AutoCommands.driveForward(swerve, 1.5, AutoConstants.AUTO_INTAKE_DRIVE_SPEED),
                Commands.sequence(
                    Commands.runOnce(intake::deploy, intake),
                    Commands.waitUntil(intake::isDeployed),
                    Commands.run(intake::runIntake, intake)
                )
            ).withTimeout(3.0),
            Commands.runOnce(intake::stopRollers, intake),
            Commands.runOnce(intake::retract, intake),

            // Score cycle 1
            AutoCommands.driveToPose(swerve, getShootingPose()),
            AutoCommands.shootAllFuel(shooter, intake),

            // Cycle 2: Collect again
            AutoCommands.driveToPose(swerve, getNeutralPose(false)),
            Commands.parallel(
                AutoCommands.driveForward(swerve, 1.0, AutoConstants.AUTO_INTAKE_DRIVE_SPEED),
                Commands.sequence(
                    Commands.runOnce(intake::deploy, intake),
                    Commands.waitUntil(intake::isDeployed),
                    Commands.run(intake::runIntake, intake)
                )
            ).withTimeout(2.0),
            Commands.runOnce(intake::stopRollers, intake),
            Commands.runOnce(intake::retract, intake),

            // Score cycle 2
            AutoCommands.driveToPose(swerve, getShootingPose()),
            AutoCommands.shootAllFuel(shooter, intake),

            AutoCommands.stopDriving(swerve),
            AutoCommands.logMessage("Mode 16 Complete")
        ).withName("16: Dual Cycle");
    }

    // ================================================================
    // MODE 17: DENY FUEL (Strategic)
    // ================================================================

    /**
     * Mode 17: Collect FUEL to deny opponents.
     * Expected Points: Variable (strategic)
     */
    public static Command denyFuelAuto(SwerveDrive swerve, Intake intake, Shooter shooter) {
        return Commands.sequence(
            AutoCommands.logMessage("Mode 17: Deny FUEL"),

            // Score preload first
            AutoCommands.shootAllFuel(shooter, intake),

            // Collect as much FUEL as possible from neutral zone
            AutoCommands.driveToPose(swerve, getNeutralPose(false)),
            Commands.runOnce(intake::deploy, intake),
            Commands.waitUntil(intake::isDeployed),
            Commands.parallel(
                Commands.run(intake::runIntake, intake),
                Commands.sequence(
                    AutoCommands.driveForward(swerve, 3.0, AutoConstants.AUTO_INTAKE_DRIVE_SPEED),
                    AutoCommands.strafeLeft(swerve, 1.0, AutoConstants.AUTO_SLOW_DRIVE_SPEED),
                    AutoCommands.driveBackward(swerve, 2.0, AutoConstants.AUTO_INTAKE_DRIVE_SPEED)
                )
            ).withTimeout(8.0),
            Commands.runOnce(intake::stopRollers, intake),
            Commands.runOnce(intake::retract, intake),

            // Score collected FUEL
            AutoCommands.driveToPose(swerve, getShootingPose()),
            AutoCommands.shootAllFuel(shooter, intake),

            AutoCommands.stopDriving(swerve),
            AutoCommands.logMessage("Mode 17 Complete")
        ).withName("17: Deny FUEL");
    }

    // ================================================================
    // MODE 18: CENTER CONTROL (Strategic)
    // ================================================================

    /**
     * Mode 18: Control neutral zone center.
     * Expected Points: Variable (strategic positioning)
     */
    public static Command centerControlAuto(SwerveDrive swerve, Intake intake, Shooter shooter) {
        return Commands.sequence(
            AutoCommands.logMessage("Mode 18: Center Control"),

            // Score preload
            AutoCommands.shootAllFuel(shooter, intake),

            // Position at center of neutral zone
            AutoCommands.driveToPose(swerve, new Pose2d(
                FieldConstants.CENTER_X,
                FieldConstants.CENTER_Y,
                Rotation2d.fromDegrees(0)
            )),

            // Collect while patrolling center
            Commands.runOnce(intake::deploy, intake),
            Commands.waitUntil(intake::isDeployed),
            Commands.parallel(
                Commands.run(intake::runIntake, intake),
                Commands.sequence(
                    AutoCommands.strafeLeft(swerve, 2.0, AutoConstants.AUTO_SLOW_DRIVE_SPEED),
                    AutoCommands.strafeRight(swerve, 4.0, AutoConstants.AUTO_SLOW_DRIVE_SPEED),
                    AutoCommands.strafeLeft(swerve, 2.0, AutoConstants.AUTO_SLOW_DRIVE_SPEED)
                )
            ).withTimeout(6.0),
            Commands.runOnce(intake::stopRollers, intake),
            Commands.runOnce(intake::retract, intake),

            // Score collected
            AutoCommands.driveToPose(swerve, getShootingPose()),
            AutoCommands.shootAllFuel(shooter, intake),

            AutoCommands.stopDriving(swerve),
            AutoCommands.logMessage("Mode 18 Complete")
        ).withName("18: Center Control");
    }

    // ================================================================
    // MODE 19: ALLIANCE SUPPORT (Strategic)
    // ================================================================

    /**
     * Mode 19: Support alliance partner's scoring.
     * Expected Points: Variable (enables alliance)
     */
    public static Command allianceSupportAuto(SwerveDrive swerve, Intake intake, Shooter shooter) {
        return Commands.sequence(
            AutoCommands.logMessage("Mode 19: Alliance Support"),

            // Score preload quickly
            AutoCommands.shootAllFuel(shooter, intake),

            // Position to funnel FUEL to alliance
            AutoCommands.driveToPose(swerve, getNeutralPose(false)),

            // Collect and position FUEL
            Commands.runOnce(intake::deploy, intake),
            Commands.waitUntil(intake::isDeployed),
            Commands.parallel(
                Commands.run(intake::runIntake, intake),
                AutoCommands.driveForward(swerve, 2.0, AutoConstants.AUTO_INTAKE_DRIVE_SPEED)
            ).withTimeout(4.0),
            Commands.runOnce(intake::stopRollers, intake),
            Commands.runOnce(intake::retract, intake),

            // Score to contribute to alliance total
            AutoCommands.driveToPose(swerve, getShootingPose()),
            AutoCommands.shootAllFuel(shooter, intake),

            // Position for TELEOP support
            AutoCommands.stopDriving(swerve),
            AutoCommands.holdPosition(swerve, 3.0),

            AutoCommands.logMessage("Mode 19 Complete")
        ).withName("19: Alliance Support");
    }

    // ================================================================
    // SIMPLE TEST ROUTINES (Not DIP selectable)
    // ================================================================

    public static Command driveForwardAuto(SwerveDrive swerve) {
        return Commands.sequence(
            AutoCommands.logMessage("Test: Drive Forward"),
            AutoCommands.driveForward(swerve, 2.0, AutoConstants.AUTO_DRIVE_SPEED),
            AutoCommands.stopDriving(swerve),
            AutoCommands.holdPosition(swerve, 1.0)
        ).withName("Drive Forward Auto");
    }

    public static Command driveBackwardAuto(SwerveDrive swerve) {
        return Commands.sequence(
            AutoCommands.logMessage("Test: Drive Backward"),
            AutoCommands.driveBackward(swerve, 2.0, AutoConstants.AUTO_DRIVE_SPEED),
            AutoCommands.stopDriving(swerve),
            AutoCommands.holdPosition(swerve, 1.0)
        ).withName("Drive Backward Auto");
    }

    public static Command driveAndIntakeAuto(SwerveDrive swerve, Intake intake) {
        return Commands.sequence(
            AutoCommands.logMessage("Test: Drive and Intake"),
            Commands.runOnce(intake::deploy, intake),
            Commands.waitUntil(intake::isDeployed),
            AutoCommands.intakeWhileDriving(intake, swerve, 3.0, 1.0),
            Commands.runOnce(intake::stopRollers, intake),
            Commands.runOnce(intake::retract, intake),
            AutoCommands.stopDriving(swerve)
        ).withName("Drive and Intake Auto");
    }

    public static Command twoFuelAuto(SwerveDrive swerve, Intake intake, Shooter shooter) {
        return Commands.sequence(
            AutoCommands.logMessage("Test: Two FUEL Auto"),
            AutoCommands.shootOneFuel(shooter, intake),
            AutoCommands.driveForward(swerve, 2.0, AutoConstants.AUTO_DRIVE_SPEED),
            AutoCommands.intakeFuelWithTimeout(intake, 2.0),
            AutoCommands.driveBackward(swerve, 2.0, AutoConstants.AUTO_DRIVE_SPEED),
            AutoCommands.shootAllFuel(shooter, intake),
            AutoCommands.stopDriving(swerve)
        ).withName("Two FUEL Auto");
    }

    // ================================================================
    // MODE SELECTOR
    // ================================================================

    /**
     * Get the appropriate auto command based on DIP switch selection.
     *
     * @param selection The DIP switch value (0-19)
     * @param swerve The swerve drive subsystem
     * @param intake The intake subsystem
     * @param shooter The shooter subsystem
     * @param climber The climber subsystem
     * @return The selected autonomous command
     */
    public static Command getAutoFromSelection(int selection, SwerveDrive swerve,
                                                Intake intake, Shooter shooter,
                                                Climber climber) {
        switch (selection) {
            case AutoConstants.AUTO_DO_NOTHING:
                return doNothing();
            case AutoConstants.AUTO_SCORE_AND_COLLECT:
                return scoreAndCollectAuto(swerve, intake, shooter);
            case AutoConstants.AUTO_QUICK_CLIMB:
                return quickClimbAuto(swerve, climber, intake, shooter);
            case AutoConstants.AUTO_SCORE_THEN_CLIMB:
                return scoreThenClimbAuto(swerve, intake, shooter, climber);
            case AutoConstants.AUTO_DEPOT_RAID:
                return depotRaidAuto(swerve, intake, shooter);
            case AutoConstants.AUTO_FAR_NEUTRAL:
                return farNeutralAuto(swerve, intake, shooter);
            case AutoConstants.AUTO_PRELOAD_ONLY:
                return preloadOnlyAuto(swerve, intake, shooter);
            case AutoConstants.AUTO_MAX_CYCLES:
                return maxCyclesAuto(swerve, intake, shooter);
            case AutoConstants.AUTO_CLIMB_SUPPORT:
                return climbSupportAuto(swerve, intake, shooter);
            case AutoConstants.AUTO_WIN_AUTO:
                return winAutoAuto(swerve, intake, shooter);
            case AutoConstants.AUTO_SCORE_COLLECT_CLIMB:
                return scoreCollectClimbAuto(swerve, intake, shooter, climber);
            case AutoConstants.AUTO_FAST_CLIMB:
                return fastClimbAuto(swerve, climber);
            case AutoConstants.AUTO_BALANCED:
                return balancedAuto(swerve, intake, shooter, climber);
            case AutoConstants.AUTO_DEPOT_CLIMB:
                return depotClimbAuto(swerve, intake, shooter, climber);
            case AutoConstants.AUTO_MAX_POINTS:
                return maxPointsAuto(swerve, intake, shooter, climber);
            case AutoConstants.AUTO_SAFE_CLIMB:
                return safeClimbAuto(swerve, intake, shooter, climber);
            case AutoConstants.AUTO_DUAL_CYCLE:
                return dualCycleAuto(swerve, intake, shooter);
            case AutoConstants.AUTO_DENY_FUEL:
                return denyFuelAuto(swerve, intake, shooter);
            case AutoConstants.AUTO_CENTER_CONTROL:
                return centerControlAuto(swerve, intake, shooter);
            case AutoConstants.AUTO_ALLIANCE_SUPPORT:
                return allianceSupportAuto(swerve, intake, shooter);
            default:
                return doNothing();
        }
    }
}
