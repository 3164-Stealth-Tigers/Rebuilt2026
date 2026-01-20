package team3164.simulator.engine;

import team3164.simulator.Constants;

/**
 * Controls the robot during autonomous period.
 *
 * Implements 10 auto modes selectable via 4-bit DIP switch:
 * - 0: Do Nothing (safety default)
 * - 1: Score & Collect (offensive - shoot all FUEL, collect more)
 * - 2: Quick Climb (defensive - drive to tower, climb L1)
 * - 3: Score Then Climb (maximum - rapid fire then climb L1)
 * - 4: Depot Raid (collect from depot, then score)
 * - 5: Far Neutral (drive to far neutral zone, collect, score)
 * - 6: Preload Only (shoot preload, hold position)
 * - 7: Max Cycles (pure scoring, shoot-collect-shoot repeating)
 * - 8: Climb Support (minimal scoring, position for TELEOP climb)
 * - 9: Win AUTO (aggressive scoring to win AUTO period)
 */
public class AutonomousController {

    // Auto mode constants (match robot code)
    public static final int AUTO_DO_NOTHING = 0;
    public static final int AUTO_SCORE_AND_COLLECT = 1;
    public static final int AUTO_QUICK_CLIMB = 2;
    public static final int AUTO_SCORE_THEN_CLIMB = 3;
    public static final int AUTO_DEPOT_RAID = 4;
    public static final int AUTO_FAR_NEUTRAL = 5;
    public static final int AUTO_PRELOAD_ONLY = 6;
    public static final int AUTO_MAX_CYCLES = 7;
    public static final int AUTO_CLIMB_SUPPORT = 8;
    public static final int AUTO_WIN_AUTO = 9;
    // NEW OPTIMIZED MODES
    public static final int AUTO_SCORE_COLLECT_CLIMB = 10;  // Score preload, collect, score, climb
    public static final int AUTO_FAST_CLIMB = 11;           // Fastest climb (no scoring)
    public static final int AUTO_BALANCED = 12;             // Balance FUEL + climb timing
    public static final int AUTO_DEPOT_CLIMB = 13;          // Depot collection then climb
    public static final int AUTO_MAX_POINTS = 14;           // Maximize total points
    // STRATEGIC MODES
    public static final int AUTO_SAFE_CLIMB = 15;           // Conservative climb approach
    public static final int AUTO_DUAL_CYCLE = 16;           // Two scoring cycles, no climb
    public static final int AUTO_DENY_FUEL = 17;            // Collect neutral FUEL to deny opponents
    public static final int AUTO_CENTER_CONTROL = 18;       // Control center field
    public static final int AUTO_ALLIANCE_SUPPORT = 19;     // Support alliance partners

    public static final int NUM_AUTO_MODES = 20;

    // Auto mode names
    public static final String[] AUTO_MODE_NAMES = {
        "0: Do Nothing",
        "1: Score & Collect",
        "2: Quick Climb",
        "3: Score Then Climb",
        "4: Depot Raid",
        "5: Far Neutral",
        "6: Preload Only",
        "7: Max Cycles",
        "8: Climb Support",
        "9: Win AUTO",
        "10: Score+Collect+Climb",
        "11: Fast Climb",
        "12: Balanced",
        "13: Depot+Climb",
        "14: Max Points",
        "15: Safe Climb",
        "16: Dual Cycle",
        "17: Deny FUEL",
        "18: Center Control",
        "19: Alliance Support"
    };

    // Selected auto mode (simulates DIP switch)
    private int selectedMode = 0;
    private boolean selectionLocked = false;

    // Debug logging
    public static boolean DEBUG_DRIVING = false;
    private static double lastDebugTime = -1;
    private int robotId = -1;  // Set during update for logging

    // State machine
    private AutoPhase currentPhase = AutoPhase.IDLE;
    private double phaseTimer = 0;
    private double totalAutoTime = 0;

    // Target positions for auto routines
    private double targetX = 0;
    private double targetY = 0;
    private double targetHeading = 0;

    // Timings (OPTIMIZED for faster FUEL collection)
    private static final double SHOOT_TIME_PER_FUEL = 0.75;
    private static final double DRIVE_TO_NEUTRAL_TIME = 8.0;  // Increased from 4.0 - need more time to reach neutral zone
    private static final double INTAKE_TIMEOUT = 4.0;         // Reduced from 5.0 - exit earlier if no FUEL found
    private static final double CLIMB_TIMEOUT = 12.0;
    private static final double DRIVE_TO_TOWER_TIME = 5.0;

    public enum AutoPhase {
        IDLE,
        // Positioning phase (used to get to shooting position first)
        POSITIONING_TO_SHOOT,
        // Score & Collect phases
        SCORING_PRELOAD,
        DRIVING_TO_NEUTRAL,
        INTAKING,
        SCORING_COLLECTED,
        // Quick Climb phases
        DRIVING_TO_TOWER,
        CLIMBING,
        HOLDING,
        // Depot Raid phases
        DRIVING_TO_DEPOT,
        COLLECTING_FROM_DEPOT,
        DRIVING_TO_SCORE,
        // Far Neutral phases
        DRIVING_TO_FAR_NEUTRAL,
        INTAKING_FAR,
        RETURNING_TO_SCORE,
        // Max Cycles phases
        CYCLING,
        // Win AUTO phases
        RAPID_SCORING,
        // Complete
        COMPLETE
    }

    /**
     * Set the selected auto mode (simulates 4-bit DIP switch).
     * Only works when selection is not locked.
     * Supports modes 0-9 (4-bit = up to 16 modes).
     */
    public void setSelectedMode(int mode) {
        if (!selectionLocked && mode >= 0 && mode < NUM_AUTO_MODES) {
            selectedMode = mode;
        }
    }

    /**
     * Get the currently selected mode.
     */
    public int getSelectedMode() {
        return selectedMode;
    }

    /**
     * Get the name of the selected mode.
     */
    public String getSelectedModeName() {
        if (selectedMode >= 0 && selectedMode < AUTO_MODE_NAMES.length) {
            return AUTO_MODE_NAMES[selectedMode];
        }
        return "Unknown";
    }

    /**
     * Lock the selection (call when auto starts).
     */
    public void lockSelection() {
        selectionLocked = true;
    }

    /**
     * Unlock selection (call when match ends or robot disabled).
     */
    public void unlockSelection() {
        selectionLocked = false;
    }

    /**
     * Check if selection is locked.
     */
    public boolean isLocked() {
        return selectionLocked;
    }

    /**
     * Start the autonomous routine.
     */
    public void startAuto(RobotState state) {
        lockSelection();
        totalAutoTime = 0;
        phaseTimer = 0;

        switch (selectedMode) {
            case AUTO_SCORE_AND_COLLECT:
                currentPhase = AutoPhase.SCORING_PRELOAD;
                break;
            case AUTO_QUICK_CLIMB:
                // FIX: Score preload first, then climb
                currentPhase = AutoPhase.SCORING_PRELOAD;
                break;
            case AUTO_SCORE_THEN_CLIMB:
                currentPhase = AutoPhase.SCORING_PRELOAD;
                break;
            case AUTO_DEPOT_RAID:
                // FIX: Score preload first, then collect from depot
                currentPhase = AutoPhase.SCORING_PRELOAD;
                break;
            case AUTO_FAR_NEUTRAL:
                // FIX: Score preload first, then collect from neutral zone
                currentPhase = AutoPhase.SCORING_PRELOAD;
                break;
            case AUTO_PRELOAD_ONLY:
                currentPhase = AutoPhase.SCORING_PRELOAD;
                break;
            case AUTO_MAX_CYCLES:
                currentPhase = AutoPhase.SCORING_PRELOAD;
                break;
            case AUTO_CLIMB_SUPPORT:
                // FIX: Score preload first, then position for climb support
                currentPhase = AutoPhase.SCORING_PRELOAD;
                break;
            case AUTO_WIN_AUTO:
                currentPhase = AutoPhase.RAPID_SCORING;
                break;
            // NEW OPTIMIZED MODES
            case AUTO_SCORE_COLLECT_CLIMB:
                currentPhase = AutoPhase.SCORING_PRELOAD;
                break;
            case AUTO_FAST_CLIMB:
                currentPhase = AutoPhase.DRIVING_TO_TOWER;
                if (state.alliance == MatchState.Alliance.RED) {
                    targetX = Constants.Field.RED_TOWER_X - 0.5;
                    targetY = Constants.Field.RED_TOWER_Y;
                } else {
                    targetX = Constants.Field.BLUE_TOWER_X + 0.5;
                    targetY = Constants.Field.BLUE_TOWER_Y;
                }
                break;
            case AUTO_BALANCED:
                currentPhase = AutoPhase.SCORING_PRELOAD;
                break;
            case AUTO_DEPOT_CLIMB:
                currentPhase = AutoPhase.SCORING_PRELOAD;
                break;
            case AUTO_MAX_POINTS:
                currentPhase = AutoPhase.SCORING_PRELOAD;
                break;
            // STRATEGIC MODES
            case AUTO_SAFE_CLIMB:
                currentPhase = AutoPhase.SCORING_PRELOAD;
                break;
            case AUTO_DUAL_CYCLE:
                currentPhase = AutoPhase.SCORING_PRELOAD;
                break;
            case AUTO_DENY_FUEL:
                currentPhase = AutoPhase.DRIVING_TO_NEUTRAL;
                targetX = Constants.Field.CENTER_X;
                targetY = Constants.Field.CENTER_Y;
                break;
            case AUTO_CENTER_CONTROL:
                currentPhase = AutoPhase.DRIVING_TO_NEUTRAL;
                targetX = Constants.Field.CENTER_X;
                targetY = Constants.Field.CENTER_Y;
                break;
            case AUTO_ALLIANCE_SUPPORT:
                currentPhase = AutoPhase.SCORING_PRELOAD;
                break;
            case AUTO_DO_NOTHING:
            default:
                currentPhase = AutoPhase.IDLE;
                break;
        }
    }

    /**
     * Update the autonomous controller.
     * Called every simulation tick during AUTO period.
     *
     * @param state Current robot state
     * @param input Input state to modify
     * @param dt Time delta in seconds
     */
    public void update(RobotState state, InputState input, double dt) {
        totalAutoTime += dt;
        phaseTimer += dt;
        this.robotId = state.robotId;  // Track for debug logging

        // Clear inputs first
        clearInputs(input);

        switch (selectedMode) {
            case AUTO_SCORE_AND_COLLECT:
                updateScoreAndCollect(state, input, dt);
                break;
            case AUTO_QUICK_CLIMB:
                updateQuickClimb(state, input, dt);
                break;
            case AUTO_SCORE_THEN_CLIMB:
                updateScoreThenClimb(state, input, dt);
                break;
            case AUTO_DEPOT_RAID:
                updateDepotRaid(state, input, dt);
                break;
            case AUTO_FAR_NEUTRAL:
                updateFarNeutral(state, input, dt);
                break;
            case AUTO_PRELOAD_ONLY:
                updatePreloadOnly(state, input, dt);
                break;
            case AUTO_MAX_CYCLES:
                updateMaxCycles(state, input, dt);
                break;
            case AUTO_CLIMB_SUPPORT:
                updateClimbSupport(state, input, dt);
                break;
            case AUTO_WIN_AUTO:
                updateWinAuto(state, input, dt);
                break;
            // NEW OPTIMIZED MODES
            case AUTO_SCORE_COLLECT_CLIMB:
                updateScoreCollectClimb(state, input, dt);
                break;
            case AUTO_FAST_CLIMB:
                updateFastClimb(state, input, dt);
                break;
            case AUTO_BALANCED:
                updateBalanced(state, input, dt);
                break;
            case AUTO_DEPOT_CLIMB:
                updateDepotClimb(state, input, dt);
                break;
            case AUTO_MAX_POINTS:
                updateMaxPoints(state, input, dt);
                break;
            // STRATEGIC MODES
            case AUTO_SAFE_CLIMB:
                updateSafeClimb(state, input, dt);
                break;
            case AUTO_DUAL_CYCLE:
                updateDualCycle(state, input, dt);
                break;
            case AUTO_DENY_FUEL:
                updateDenyFuel(state, input, dt);
                break;
            case AUTO_CENTER_CONTROL:
                updateCenterControl(state, input, dt);
                break;
            case AUTO_ALLIANCE_SUPPORT:
                updateAllianceSupport(state, input, dt);
                break;
            case AUTO_DO_NOTHING:
            default:
                // Do nothing
                break;
        }
    }

    /**
     * Reset the controller.
     */
    public void reset() {
        currentPhase = AutoPhase.IDLE;
        phaseTimer = 0;
        totalAutoTime = 0;
        unlockSelection();
    }

    /**
     * Get current phase name for display.
     */
    public String getCurrentPhaseName() {
        return currentPhase.toString().replace("_", " ");
    }

    /**
     * Get total auto time elapsed.
     */
    public double getAutoTime() {
        return totalAutoTime;
    }

    // ========================================================================
    // AUTO MODE 1: Score & Collect (OPTIMIZED - faster shooting and closer targets)
    // ========================================================================
    private void updateScoreAndCollect(RobotState state, InputState input, double dt) {
        switch (currentPhase) {
            case SCORING_PRELOAD:
                // Check if we need to reposition first
                if (!isInShootingPosition(state)) {
                    double[] shootPos = getShootingPosition(state);
                    targetX = shootPos[0];
                    targetY = shootPos[1];
                    transitionToPhase(AutoPhase.POSITIONING_TO_SHOOT);
                    return;
                }
                // Shoot all preloaded FUEL with proper aiming
                if (state.fuelCount > 0) {
                    input.spinUp = true;
                    boolean readyToShoot = aimAtHub(state, input);
                    if (readyToShoot) {
                        input.shoot = true;
                    }
                } else {
                    // All FUEL shot, move to CLOSER neutral zone target
                    transitionToPhase(AutoPhase.DRIVING_TO_NEUTRAL);
                    // OPTIMIZATION: Target closer FUEL sources (2m closer than center)
                    targetX = Constants.Field.CENTER_X - 2.0;
                    targetY = Constants.Field.CENTER_Y;
                }
                break;

            case POSITIONING_TO_SHOOT:
                // Drive to shooting position, going AROUND the hub AND bumps
                double posHubX = (state.alliance == MatchState.Alliance.RED)
                    ? Constants.Field.RED_HUB_X : Constants.Field.BLUE_HUB_X;

                // Check if robot is in the danger zone (near hub/bumps)
                double posDangerZoneHalfX = Constants.Field.BUMP_WIDTH / 2.0 + Constants.Robot.LENGTH_WITH_BUMPERS / 2.0 + 0.5;
                boolean posInDangerZone = Math.abs(state.x - posHubX) < posDangerZoneHalfX;

                // Determine safe Y position (above or below bumps)
                boolean posGoAbove = state.y >= Constants.Field.CENTER_Y;
                double posSafeY = getSafeYPosition(state, posGoAbove);
                boolean posOnSafePath = posGoAbove ? (state.y >= posSafeY - 0.5) : (state.y <= posSafeY + 0.5);

                if (posInDangerZone && !posOnSafePath && state.x > posHubX) {
                    // Coming from neutral zone side - need to move to safe Y first
                    // Team 3164 MUST avoid bumps
                    driveToTarget(state, input, state.x - 0.3, posSafeY);
                } else if (posInDangerZone && state.x > posHubX) {
                    // On safe path, drive through the gap toward alliance zone
                    driveToTarget(state, input, posHubX - posDangerZoneHalfX - 0.5, posSafeY);
                } else {
                    // Past the hub/bump area, drive to shooting position
                    driveToTarget(state, input, targetX, targetY);
                }

                if (phaseTimer > 6.0 || isAtTarget(state, targetX, targetY, 0.5)) {
                    // Transition to SCORING_COLLECTED (not SCORING_PRELOAD) if we have collected FUEL
                    transitionToPhase(AutoPhase.SCORING_COLLECTED);
                }
                break;

            case DRIVING_TO_NEUTRAL:
                // Drive towards neutral zone, going AROUND the hub AND bumps
                // Hub is at x=4.03 (BLUE) with bumps flanking it in Y direction
                double hubX = (state.alliance == MatchState.Alliance.RED)
                    ? Constants.Field.RED_HUB_X : Constants.Field.BLUE_HUB_X;

                // Check if robot needs to navigate around hub/bump area
                // The danger zone extends from hub to include bumps
                double dangerZoneHalfX = Constants.Field.BUMP_WIDTH / 2.0 + Constants.Robot.LENGTH_WITH_BUMPERS / 2.0 + 0.5;
                boolean inDangerZone = Math.abs(state.x - hubX) < dangerZoneHalfX;

                // Check if we're on a safe path (above or below the bumps)
                boolean goAbove = state.y >= Constants.Field.CENTER_Y;
                double safeY = getSafeYPosition(state, goAbove);
                boolean onSafePath = goAbove ? (state.y >= safeY - 0.5) : (state.y <= safeY + 0.5);

                if (inDangerZone && !onSafePath && state.x < hubX) {
                    // Need to move to safe Y position first before proceeding
                    // Team 3164 MUST avoid bumps - use wider path
                    driveToTarget(state, input, state.x + 0.3, safeY);
                } else if (inDangerZone && state.x < hubX) {
                    // On safe path, drive forward through the gap
                    driveToTarget(state, input, hubX + dangerZoneHalfX + 0.5, safeY);
                } else {
                    // Past the hub/bump area, drive towards the neutral zone target
                    driveToTarget(state, input, targetX, targetY);
                }

                if (phaseTimer > DRIVE_TO_NEUTRAL_TIME || isAtTarget(state, targetX, targetY, 0.5)) {
                    transitionToPhase(AutoPhase.INTAKING);
                }
                break;

            case INTAKING:
                // OPTIMIZED: More aggressive intake phase
                input.intake = true;

                // Drive faster while intaking to cover more ground
                input.forward = 0.4;  // Increased from 0.2

                // OPTIMIZATION: Collect up to 5 FUEL (max capacity is 8, preload is 3)
                boolean hasEnoughFuel = state.fuelCount >= 5;  // Increased from 4 to 5
                boolean timeoutReached = phaseTimer > INTAKE_TIMEOUT;

                if (hasEnoughFuel || timeoutReached) {
                    if (state.fuelCount > 0) {
                        transitionToPhase(AutoPhase.SCORING_COLLECTED);
                    } else {
                        transitionToPhase(AutoPhase.COMPLETE);
                    }
                }
                break;

            case SCORING_COLLECTED:
                // Check if we need to reposition first
                if (!isInShootingPosition(state)) {
                    double[] shootPos = getShootingPosition(state);
                    targetX = shootPos[0];
                    targetY = shootPos[1];
                    transitionToPhase(AutoPhase.POSITIONING_TO_SHOOT);
                    return;
                }
                // Shoot collected FUEL with proper aiming
                if (state.fuelCount > 0) {
                    input.spinUp = true;
                    boolean readyToShoot = aimAtHub(state, input);
                    if (readyToShoot) {
                        input.shoot = true;
                    }
                } else {
                    // OPTIMIZATION: Check if enough time for another cycle (~8s needed)
                    if (totalAutoTime < 12.0) {
                        // Time for another cycle - go collect more
                        transitionToPhase(AutoPhase.DRIVING_TO_NEUTRAL);
                        targetX = Constants.Field.CENTER_X - 2.0;
                        targetY = Constants.Field.CENTER_Y;
                    } else {
                        transitionToPhase(AutoPhase.COMPLETE);
                    }
                }
                break;

            case COMPLETE:
            default:
                // Done
                break;
        }
    }

    // ========================================================================
    // AUTO MODE 2: Quick Climb (FIXED - now scores preload first)
    // ========================================================================
    private void updateQuickClimb(RobotState state, InputState input, double dt) {
        switch (currentPhase) {
            case SCORING_PRELOAD:
                // FIX: Score preload first before climbing
                if (!isInShootingPosition(state)) {
                    double[] shootPos = getShootingPosition(state);
                    targetX = shootPos[0];
                    targetY = shootPos[1];
                    transitionToPhase(AutoPhase.POSITIONING_TO_SHOOT);
                    return;
                }
                if (state.fuelCount > 0) {
                    input.spinUp = true;
                    boolean readyToShoot = aimAtHub(state, input);
                    if (readyToShoot) {
                        input.shoot = true;
                    }
                } else {
                    // Done shooting, head to tower
                    transitionToPhase(AutoPhase.DRIVING_TO_TOWER);
                    if (state.alliance == MatchState.Alliance.RED) {
                        targetX = Constants.Field.RED_TOWER_X - 1.0;
                        targetY = Constants.Field.RED_TOWER_Y;
                    } else {
                        targetX = Constants.Field.BLUE_TOWER_X + 1.0;
                        targetY = Constants.Field.BLUE_TOWER_Y;
                    }
                }
                break;

            case POSITIONING_TO_SHOOT:
                // Drive to shooting position while avoiding bumps
                driveToPositionAvoidingBumps(state, input, targetX, targetY);
                if (phaseTimer > 4.0 || isAtTarget(state, targetX, targetY, 0.5)) {
                    transitionToPhase(AutoPhase.SCORING_PRELOAD);
                }
                break;

            case DRIVING_TO_TOWER:
                // Drive directly to tower
                driveToTarget(state, input, targetX, targetY);

                if (phaseTimer > DRIVE_TO_TOWER_TIME || isAtTarget(state, targetX, targetY, 0.3)) {
                    transitionToPhase(AutoPhase.CLIMBING);
                }
                break;

            case CLIMBING:
                // Climb to L1
                input.level1 = true;
                input.climberUp = true;

                if (state.climbComplete || phaseTimer > CLIMB_TIMEOUT) {
                    transitionToPhase(AutoPhase.HOLDING);
                }
                break;

            case HOLDING:
                // Hold position (do nothing, let robot stay on tower)
                break;

            case COMPLETE:
            default:
                break;
        }
    }

    // ========================================================================
    // AUTO MODE 3: Score Then Climb
    // ========================================================================
    private void updateScoreThenClimb(RobotState state, InputState input, double dt) {
        switch (currentPhase) {
            case SCORING_PRELOAD:
                // Check if we need to reposition first
                if (!isInShootingPosition(state)) {
                    double[] shootPos = getShootingPosition(state);
                    targetX = shootPos[0];
                    targetY = shootPos[1];
                    transitionToPhase(AutoPhase.POSITIONING_TO_SHOOT);
                    return;
                }
                // Rapid fire preloaded FUEL with proper aiming
                if (state.fuelCount > 0) {
                    input.spinUp = true;
                    boolean readyToShoot = aimAtHub(state, input);
                    if (readyToShoot) {
                        input.shoot = true;
                    }
                } else {
                    // Done shooting, head to tower (use correct alliance tower)
                    transitionToPhase(AutoPhase.DRIVING_TO_TOWER);
                    if (state.alliance == MatchState.Alliance.RED) {
                        targetX = Constants.Field.RED_TOWER_X - 1.0;
                        targetY = Constants.Field.RED_TOWER_Y;
                    } else {
                        targetX = Constants.Field.BLUE_TOWER_X + 1.0;
                        targetY = Constants.Field.BLUE_TOWER_Y;
                    }
                }
                break;

            case POSITIONING_TO_SHOOT:
                // Drive to shooting position while avoiding bumps (team 3164 can't traverse bumps)
                driveToPositionAvoidingBumps(state, input, targetX, targetY);
                if (phaseTimer > 4.0 || isAtTarget(state, targetX, targetY, 0.5)) {
                    transitionToPhase(AutoPhase.SCORING_PRELOAD);
                }
                break;

            case DRIVING_TO_TOWER:
                // Drive to tower
                driveToTarget(state, input, targetX, targetY);

                if (phaseTimer > 4.0 || isAtTarget(state, targetX, targetY, 0.3)) {
                    transitionToPhase(AutoPhase.CLIMBING);
                }
                break;

            case CLIMBING:
                // Climb to L1
                input.level1 = true;
                input.climberUp = true;

                if (state.climbComplete || phaseTimer > CLIMB_TIMEOUT) {
                    transitionToPhase(AutoPhase.HOLDING);
                }
                break;

            case HOLDING:
            case COMPLETE:
            default:
                // Hold position
                break;
        }
    }

    // ========================================================================
    // AUTO MODE 4: Depot Raid (REVERTED - collect from depot first, then score all)
    // Strategy: Drive to alliance depot, collect FUEL, drive back to score ALL (preload + depot)
    // Note: This mode skips initial preload scoring to maximize depot collection time
    // ========================================================================
    private void updateDepotRaid(RobotState state, InputState input, double dt) {
        switch (currentPhase) {
            case SCORING_PRELOAD:
                // REVERTED: Go to depot first (no preload scoring to save time)
                transitionToPhase(AutoPhase.DRIVING_TO_DEPOT);
                if (state.alliance == MatchState.Alliance.RED) {
                    targetX = Constants.Field.RED_DEPOT_X;
                    targetY = Constants.Field.RED_DEPOT_Y;
                } else {
                    targetX = Constants.Field.BLUE_DEPOT_X;
                    targetY = Constants.Field.BLUE_DEPOT_Y;
                }
                break;

            case DRIVING_TO_DEPOT:
                // Drive to alliance depot (no bump avoidance needed - depot is in alliance zone)
                driveToTarget(state, input, targetX, targetY);

                if (phaseTimer > 3.0 || isAtTarget(state, targetX, targetY, 0.5)) {
                    transitionToPhase(AutoPhase.COLLECTING_FROM_DEPOT);
                }
                break;

            case COLLECTING_FROM_DEPOT:
                // Collect FUEL from depot
                input.intake = true;

                // Small movements to find FUEL
                if (phaseTimer < 3.0) {
                    input.forward = 0.2 * Math.sin(phaseTimer * 2);
                }

                // Shorter collection time to leave time for scoring
                if (phaseTimer > 4.0 || state.fuelCount >= 5) {
                    transitionToPhase(AutoPhase.DRIVING_TO_SCORE);
                    // Target own alliance hub for scoring
                    if (state.alliance == MatchState.Alliance.RED) {
                        targetX = Constants.Field.RED_HUB_X - 2.5;
                    } else {
                        targetX = Constants.Field.BLUE_HUB_X + 2.5;
                    }
                    targetY = Constants.Field.CENTER_Y;
                }
                break;

            case DRIVING_TO_SCORE:
                // Drive to scoring position (no bump avoidance - staying in alliance zone)
                driveToTarget(state, input, targetX, targetY);

                if (phaseTimer > 4.0 || isAtTarget(state, targetX, targetY, 1.5)) {
                    transitionToPhase(AutoPhase.SCORING_COLLECTED);
                }
                break;

            case SCORING_COLLECTED:
                // Score all FUEL (preload + collected from depot)
                if (state.fuelCount > 0) {
                    input.spinUp = true;
                    boolean readyToShoot = aimAtHub(state, input);
                    if (readyToShoot) {
                        input.shoot = true;
                    }
                } else {
                    transitionToPhase(AutoPhase.COMPLETE);
                }
                break;

            case POSITIONING_TO_SHOOT:
                // Drive to shooting position
                driveToTarget(state, input, targetX, targetY);
                if (phaseTimer > 3.0 || isAtTarget(state, targetX, targetY, 0.5)) {
                    transitionToPhase(AutoPhase.SCORING_COLLECTED);
                }
                break;

            case COMPLETE:
            default:
                break;
        }
    }

    // ========================================================================
    // AUTO MODE 5: Far Neutral (FIXED - now scores preload first, uses closer target)
    // Strategy: Score preload, drive to neutral zone, collect, score
    // ========================================================================
    private void updateFarNeutral(RobotState state, InputState input, double dt) {
        switch (currentPhase) {
            case SCORING_PRELOAD:
                // FIX: Score preload first before going to neutral zone
                if (!isInShootingPosition(state)) {
                    double[] shootPos = getShootingPosition(state);
                    targetX = shootPos[0];
                    targetY = shootPos[1];
                    transitionToPhase(AutoPhase.POSITIONING_TO_SHOOT);
                    return;
                }
                if (state.fuelCount > 0) {
                    input.spinUp = true;
                    boolean readyToShoot = aimAtHub(state, input);
                    if (readyToShoot) {
                        input.shoot = true;
                    }
                } else {
                    // Done with preload, now go to neutral zone (CLOSER target for reliability)
                    transitionToPhase(AutoPhase.DRIVING_TO_FAR_NEUTRAL);
                    targetX = Constants.Field.CENTER_X - 1.0;  // CLOSER target instead of +3.0
                    targetY = Constants.Field.CENTER_Y;
                }
                break;

            case POSITIONING_TO_SHOOT:
                // Drive to shooting position while avoiding bumps
                driveToPositionAvoidingBumps(state, input, targetX, targetY);
                if (phaseTimer > 4.0 || isAtTarget(state, targetX, targetY, 0.5)) {
                    // Return to appropriate phase based on whether we have FUEL
                    if (state.fuelCount > 0) {
                        transitionToPhase(AutoPhase.SCORING_COLLECTED);
                    } else {
                        transitionToPhase(AutoPhase.SCORING_PRELOAD);
                    }
                }
                break;

            case DRIVING_TO_FAR_NEUTRAL:
                // Drive to neutral zone, avoiding bumps
                driveToPositionAvoidingBumps(state, input, targetX, targetY);

                if (phaseTimer > 6.0 || isAtTarget(state, targetX, targetY, 0.5)) {
                    transitionToPhase(AutoPhase.INTAKING_FAR);
                }
                break;

            case INTAKING_FAR:
                // Intake while moving through neutral zone
                input.intake = true;
                input.forward = 0.3;

                // FIX: Shorter intake time to leave time for scoring
                if (phaseTimer > 3.0 || state.fuelCount >= 4) {
                    transitionToPhase(AutoPhase.RETURNING_TO_SCORE);
                    // Target own alliance hub for scoring
                    if (state.alliance == MatchState.Alliance.RED) {
                        targetX = Constants.Field.RED_HUB_X - 2.5;
                    } else {
                        targetX = Constants.Field.BLUE_HUB_X + 2.5;
                    }
                    targetY = Constants.Field.CENTER_Y;
                }
                break;

            case RETURNING_TO_SCORE:
                // Return to hub, avoiding bumps
                driveToPositionAvoidingBumps(state, input, targetX, targetY);

                if (phaseTimer > 5.0 || isAtTarget(state, targetX, targetY, 1.5)) {
                    transitionToPhase(AutoPhase.SCORING_COLLECTED);
                }
                break;

            case SCORING_COLLECTED:
                // Check if we need to reposition first
                if (!isInShootingPosition(state)) {
                    double[] shootPos = getShootingPosition(state);
                    targetX = shootPos[0];
                    targetY = shootPos[1];
                    transitionToPhase(AutoPhase.POSITIONING_TO_SHOOT);
                    return;
                }
                // Score collected FUEL with proper aiming
                if (state.fuelCount > 0) {
                    input.spinUp = true;
                    boolean readyToShoot = aimAtHub(state, input);
                    if (readyToShoot) {
                        input.shoot = true;
                    }
                } else {
                    transitionToPhase(AutoPhase.COMPLETE);
                }
                break;

            case COMPLETE:
            default:
                break;
        }
    }

    // ========================================================================
    // AUTO MODE 6: Preload Only
    // Strategy: Shoot preload, hold position (safe, predictable)
    // ========================================================================
    private void updatePreloadOnly(RobotState state, InputState input, double dt) {
        switch (currentPhase) {
            case SCORING_PRELOAD:
                // Check if we need to reposition first
                if (!isInShootingPosition(state)) {
                    double[] shootPos = getShootingPosition(state);
                    targetX = shootPos[0];
                    targetY = shootPos[1];
                    transitionToPhase(AutoPhase.POSITIONING_TO_SHOOT);
                    return;
                }
                // Shoot all preloaded FUEL with proper aiming
                if (state.fuelCount > 0) {
                    input.spinUp = true;
                    boolean readyToShoot = aimAtHub(state, input);
                    if (readyToShoot) {
                        input.shoot = true;
                    }
                } else {
                    transitionToPhase(AutoPhase.HOLDING);
                }
                break;

            case POSITIONING_TO_SHOOT:
                // Drive to shooting position while avoiding bumps (team 3164 can't traverse bumps)
                driveToPositionAvoidingBumps(state, input, targetX, targetY);
                if (phaseTimer > 4.0 || isAtTarget(state, targetX, targetY, 0.5)) {
                    transitionToPhase(AutoPhase.SCORING_PRELOAD);
                }
                break;

            case HOLDING:
            case COMPLETE:
            default:
                // Stay in place - no movement
                break;
        }
    }

    // ========================================================================
    // AUTO MODE 7: Max Cycles (OPTIMIZED - same as Mode 1 Score & Collect)
    // Strategy: Pure scoring - shoot, collect, shoot, repeat
    // ========================================================================
    private void updateMaxCycles(RobotState state, InputState input, double dt) {
        // Use same logic as Score & Collect mode which works well
        switch (currentPhase) {
            case SCORING_PRELOAD:
                // Check if we need to reposition first
                if (!isInShootingPosition(state)) {
                    double[] shootPos = getShootingPosition(state);
                    targetX = shootPos[0];
                    targetY = shootPos[1];
                    transitionToPhase(AutoPhase.POSITIONING_TO_SHOOT);
                    return;
                }
                // Shoot all preloaded FUEL with proper aiming
                if (state.fuelCount > 0) {
                    input.spinUp = true;
                    boolean readyToShoot = aimAtHub(state, input);
                    if (readyToShoot) {
                        input.shoot = true;
                    }
                } else {
                    // All FUEL shot, move to neutral zone
                    transitionToPhase(AutoPhase.DRIVING_TO_NEUTRAL);
                    targetX = Constants.Field.CENTER_X - 2.0;  // Closer target for faster cycle
                    targetY = Constants.Field.CENTER_Y;
                }
                break;

            case POSITIONING_TO_SHOOT:
                // Use same positioning logic as Score & Collect
                double posHubX = (state.alliance == MatchState.Alliance.RED)
                    ? Constants.Field.RED_HUB_X : Constants.Field.BLUE_HUB_X;
                double posDangerZoneHalfX = Constants.Field.BUMP_WIDTH / 2.0 + Constants.Robot.LENGTH_WITH_BUMPERS / 2.0 + 0.5;
                boolean posInDangerZone = Math.abs(state.x - posHubX) < posDangerZoneHalfX;
                boolean posGoAbove = state.y >= Constants.Field.CENTER_Y;
                double posSafeY = getSafeYPosition(state, posGoAbove);
                boolean posOnSafePath = posGoAbove ? (state.y >= posSafeY - 0.5) : (state.y <= posSafeY + 0.5);

                if (posInDangerZone && !posOnSafePath && state.x > posHubX) {
                    driveToTarget(state, input, state.x - 0.3, posSafeY);
                } else if (posInDangerZone && state.x > posHubX) {
                    driveToTarget(state, input, posHubX - posDangerZoneHalfX - 0.5, posSafeY);
                } else {
                    driveToTarget(state, input, targetX, targetY);
                }

                if (phaseTimer > 6.0 || isAtTarget(state, targetX, targetY, 0.5)) {
                    transitionToPhase(AutoPhase.SCORING_COLLECTED);
                }
                break;

            case DRIVING_TO_NEUTRAL:
                // Use same driving logic as Score & Collect
                double hubX = (state.alliance == MatchState.Alliance.RED)
                    ? Constants.Field.RED_HUB_X : Constants.Field.BLUE_HUB_X;
                double dangerZoneHalfX = Constants.Field.BUMP_WIDTH / 2.0 + Constants.Robot.LENGTH_WITH_BUMPERS / 2.0 + 0.5;
                boolean inDangerZone = Math.abs(state.x - hubX) < dangerZoneHalfX;
                boolean goAbove = state.y >= Constants.Field.CENTER_Y;
                double safeY = getSafeYPosition(state, goAbove);
                boolean onSafePath = goAbove ? (state.y >= safeY - 0.5) : (state.y <= safeY + 0.5);

                if (inDangerZone && !onSafePath && state.x < hubX) {
                    driveToTarget(state, input, state.x + 0.3, safeY);
                } else if (inDangerZone && state.x < hubX) {
                    driveToTarget(state, input, hubX + dangerZoneHalfX + 0.5, safeY);
                } else {
                    driveToTarget(state, input, targetX, targetY);
                }

                if (phaseTimer > DRIVE_TO_NEUTRAL_TIME || isAtTarget(state, targetX, targetY, 0.5)) {
                    transitionToPhase(AutoPhase.INTAKING);
                }
                break;

            case INTAKING:
                input.intake = true;
                input.forward = 0.4;

                // OPTIMIZATION: Collect up to 5 FUEL for max scoring
                boolean hasEnoughFuel7 = state.fuelCount >= 5;
                boolean timeoutReached7 = phaseTimer > INTAKE_TIMEOUT;

                if (hasEnoughFuel7 || timeoutReached7) {
                    if (state.fuelCount > 0) {
                        transitionToPhase(AutoPhase.SCORING_COLLECTED);
                    } else {
                        transitionToPhase(AutoPhase.COMPLETE);
                    }
                }
                break;

            case SCORING_COLLECTED:
                // Check if we need to reposition first
                if (!isInShootingPosition(state)) {
                    double[] shootPos = getShootingPosition(state);
                    targetX = shootPos[0];
                    targetY = shootPos[1];
                    transitionToPhase(AutoPhase.POSITIONING_TO_SHOOT);
                    return;
                }
                // Shoot collected FUEL
                if (state.fuelCount > 0) {
                    input.spinUp = true;
                    boolean readyToShoot = aimAtHub(state, input);
                    if (readyToShoot) {
                        input.shoot = true;
                    }
                } else {
                    // OPTIMIZATION: Check if enough time for another cycle
                    if (totalAutoTime < 12.0) {
                        transitionToPhase(AutoPhase.DRIVING_TO_NEUTRAL);
                        targetX = Constants.Field.CENTER_X - 2.0;
                        targetY = Constants.Field.CENTER_Y;
                    } else {
                        transitionToPhase(AutoPhase.COMPLETE);
                    }
                }
                break;

            case COMPLETE:
            default:
                break;
        }
    }

    // ========================================================================
    // AUTO MODE 8: Climb Support (FIXED - scores preload first)
    // Strategy: Score preload, then position for TELEOP climb assist
    // ========================================================================
    private void updateClimbSupport(RobotState state, InputState input, double dt) {
        switch (currentPhase) {
            case SCORING_PRELOAD:
                // FIX: Score preload first before positioning
                if (!isInShootingPosition(state)) {
                    double[] shootPos = getShootingPosition(state);
                    targetX = shootPos[0];
                    targetY = shootPos[1];
                    transitionToPhase(AutoPhase.POSITIONING_TO_SHOOT);
                    return;
                }
                if (state.fuelCount > 0) {
                    input.spinUp = true;
                    boolean readyToShoot = aimAtHub(state, input);
                    if (readyToShoot) {
                        input.shoot = true;
                    }
                } else {
                    // Done shooting, position near tower for teleop
                    transitionToPhase(AutoPhase.DRIVING_TO_TOWER);
                    if (state.alliance == MatchState.Alliance.RED) {
                        targetX = Constants.Field.RED_TOWER_X - 2.0;
                        targetY = Constants.Field.RED_TOWER_Y;
                    } else {
                        targetX = Constants.Field.BLUE_TOWER_X + 2.0;
                        targetY = Constants.Field.BLUE_TOWER_Y;
                    }
                }
                break;

            case POSITIONING_TO_SHOOT:
                // Drive to shooting position while avoiding bumps
                driveToPositionAvoidingBumps(state, input, targetX, targetY);
                if (phaseTimer > 4.0 || isAtTarget(state, targetX, targetY, 0.5)) {
                    transitionToPhase(AutoPhase.SCORING_PRELOAD);
                }
                break;

            case DRIVING_TO_TOWER:
                // Drive to position near tower
                driveToTarget(state, input, targetX, targetY);

                if (phaseTimer > 6.0 || isAtTarget(state, targetX, targetY, 0.5)) {
                    transitionToPhase(AutoPhase.HOLDING);
                }
                break;

            case HOLDING:
            case COMPLETE:
            default:
                // Hold position near tower - ready for teleop climb
                break;
        }
    }

    // ========================================================================
    // AUTO MODE 9: Win AUTO (FIXED - closer targets, bump avoidance, faster returns)
    // Strategy: Aggressive rapid fire to maximize AUTO period score
    // ========================================================================
    private void updateWinAuto(RobotState state, InputState input, double dt) {
        switch (currentPhase) {
            case RAPID_SCORING:
                // Check if we need to reposition first
                if (!isInShootingPosition(state)) {
                    double[] shootPos = getShootingPosition(state);
                    targetX = shootPos[0];
                    targetY = shootPos[1];
                    transitionToPhase(AutoPhase.POSITIONING_TO_SHOOT);
                    return;
                }
                // Maximum speed shooting with proper aiming
                if (state.fuelCount > 0) {
                    input.spinUp = true;
                    // Use aimAtHub to set proper angle/power and turn toward hub
                    boolean readyToShoot = aimAtHub(state, input);
                    if (readyToShoot) {
                        input.shoot = true;
                    }
                } else {
                    // FIX: Check if enough time left for another cycle (need ~8s for drive+intake+return+score)
                    if (totalAutoTime < 11.0) {  // Only go if < 11 seconds elapsed
                        // FIX: Use CLOSER target for faster cycle
                        transitionToPhase(AutoPhase.DRIVING_TO_NEUTRAL);
                        targetX = Constants.Field.CENTER_X - 2.0;  // Closer target
                        targetY = Constants.Field.CENTER_Y;
                    } else {
                        // Not enough time, end auto
                        transitionToPhase(AutoPhase.COMPLETE);
                    }
                }
                break;

            case POSITIONING_TO_SHOOT:
                // FIX: Drive to shooting position while avoiding bumps (faster timeout)
                driveToPositionAvoidingBumps(state, input, targetX, targetY);
                if (phaseTimer > 2.5 || isAtTarget(state, targetX, targetY, 0.5)) {
                    transitionToPhase(AutoPhase.RAPID_SCORING);
                }
                break;

            case DRIVING_TO_NEUTRAL:
                // FIX: Fast drive to neutral with bump avoidance
                driveToPositionAvoidingBumps(state, input, targetX, targetY);
                // Run intake while driving
                input.intake = true;

                if (phaseTimer > 3.0 || isAtTarget(state, targetX, targetY, 0.5)) {
                    transitionToPhase(AutoPhase.INTAKING);
                }
                break;

            case INTAKING:
                // Quick intake
                input.intake = true;
                input.forward = 0.4;  // Faster movement

                // FIX: Shorter intake time to leave time for scoring
                if (phaseTimer > 2.0 || state.fuelCount >= 3) {
                    // Score quickly, don't wait for full load
                    transitionToPhase(AutoPhase.RAPID_SCORING);
                }
                break;

            case COMPLETE:
            default:
                break;
        }
    }

    // ========================================================================
    // AUTO MODE 10: Score + Collect + Climb
    // Strategy: Score preload, quick collect, score, then climb if time permits
    // Target: 5-6 FUEL + L1 climb = 20-21 points
    // ========================================================================
    private void updateScoreCollectClimb(RobotState state, InputState input, double dt) {
        switch (currentPhase) {
            case SCORING_PRELOAD:
                if (!isInShootingPosition(state)) {
                    double[] shootPos = getShootingPosition(state);
                    targetX = shootPos[0];
                    targetY = shootPos[1];
                    transitionToPhase(AutoPhase.POSITIONING_TO_SHOOT);
                    return;
                }
                if (state.fuelCount > 0) {
                    input.spinUp = true;
                    boolean readyToShoot = aimAtHub(state, input);
                    if (readyToShoot) {
                        input.shoot = true;
                    }
                } else {
                    // Quick collect phase - only if early enough
                    if (totalAutoTime < 3.0) {
                        transitionToPhase(AutoPhase.DRIVING_TO_NEUTRAL);
                        targetX = Constants.Field.CENTER_X - 2.5;  // Very close target
                        targetY = Constants.Field.CENTER_Y;
                    } else {
                        // Go straight to climb
                        transitionToPhase(AutoPhase.DRIVING_TO_TOWER);
                        setTowerTarget(state);
                    }
                }
                break;

            case POSITIONING_TO_SHOOT:
                driveToPositionAvoidingBumps(state, input, targetX, targetY);
                if (phaseTimer > 4.0 || isAtTarget(state, targetX, targetY, 0.5)) {
                    if (state.fuelCount > 0) {
                        transitionToPhase(AutoPhase.SCORING_COLLECTED);
                    } else {
                        transitionToPhase(AutoPhase.SCORING_PRELOAD);
                    }
                }
                break;

            case DRIVING_TO_NEUTRAL:
                driveToPositionAvoidingBumps(state, input, targetX, targetY);
                if (phaseTimer > 4.0 || isAtTarget(state, targetX, targetY, 0.5)) {
                    transitionToPhase(AutoPhase.INTAKING);
                }
                break;

            case INTAKING:
                input.intake = true;
                input.forward = 0.4;
                // Quick intake - get 2-3 FUEL and go
                if (phaseTimer > 2.0 || state.fuelCount >= 3) {
                    transitionToPhase(AutoPhase.SCORING_COLLECTED);
                }
                break;

            case SCORING_COLLECTED:
                if (!isInShootingPosition(state)) {
                    double[] shootPos = getShootingPosition(state);
                    targetX = shootPos[0];
                    targetY = shootPos[1];
                    transitionToPhase(AutoPhase.POSITIONING_TO_SHOOT);
                    return;
                }
                if (state.fuelCount > 0) {
                    input.spinUp = true;
                    boolean readyToShoot = aimAtHub(state, input);
                    if (readyToShoot) {
                        input.shoot = true;
                    }
                } else {
                    // Check if time for climb (need ~4s)
                    if (totalAutoTime < 16.0) {
                        transitionToPhase(AutoPhase.DRIVING_TO_TOWER);
                        setTowerTarget(state);
                    } else {
                        transitionToPhase(AutoPhase.COMPLETE);
                    }
                }
                break;

            case DRIVING_TO_TOWER:
                driveToTarget(state, input, targetX, targetY);
                if (phaseTimer > 3.0 || isAtTarget(state, targetX, targetY, 0.5)) {
                    transitionToPhase(AutoPhase.CLIMBING);
                }
                break;

            case CLIMBING:
                input.level1 = true;
                input.climberUp = true;
                if (state.climbComplete || phaseTimer > 8.0) {
                    transitionToPhase(AutoPhase.COMPLETE);
                }
                break;

            case COMPLETE:
            default:
                break;
        }
    }

    // ========================================================================
    // AUTO MODE 11: Fast Climb
    // Strategy: Drive directly to tower, climb immediately (15 points guaranteed)
    // ========================================================================
    private void updateFastClimb(RobotState state, InputState input, double dt) {
        switch (currentPhase) {
            case DRIVING_TO_TOWER:
                driveToTarget(state, input, targetX, targetY);
                if (phaseTimer > 2.5 || isAtTarget(state, targetX, targetY, 0.3)) {
                    transitionToPhase(AutoPhase.CLIMBING);
                }
                break;

            case CLIMBING:
                input.level1 = true;
                input.climberUp = true;
                if (state.climbComplete || phaseTimer > 10.0) {
                    transitionToPhase(AutoPhase.HOLDING);
                }
                break;

            case HOLDING:
            case COMPLETE:
            default:
                break;
        }
    }

    // ========================================================================
    // AUTO MODE 12: Balanced
    // Strategy: Score preload (3 pts), then climb (15 pts) = 18 points
    // Same as Mode 3 but optimized timing
    // ========================================================================
    private void updateBalanced(RobotState state, InputState input, double dt) {
        switch (currentPhase) {
            case SCORING_PRELOAD:
                if (!isInShootingPosition(state)) {
                    double[] shootPos = getShootingPosition(state);
                    targetX = shootPos[0];
                    targetY = shootPos[1];
                    transitionToPhase(AutoPhase.POSITIONING_TO_SHOOT);
                    return;
                }
                if (state.fuelCount > 0) {
                    input.spinUp = true;
                    boolean readyToShoot = aimAtHub(state, input);
                    if (readyToShoot) {
                        input.shoot = true;
                    }
                } else {
                    transitionToPhase(AutoPhase.DRIVING_TO_TOWER);
                    setTowerTarget(state);
                }
                break;

            case POSITIONING_TO_SHOOT:
                driveToPositionAvoidingBumps(state, input, targetX, targetY);
                if (phaseTimer > 3.0 || isAtTarget(state, targetX, targetY, 0.5)) {
                    transitionToPhase(AutoPhase.SCORING_PRELOAD);
                }
                break;

            case DRIVING_TO_TOWER:
                driveToTarget(state, input, targetX, targetY);
                if (phaseTimer > 3.0 || isAtTarget(state, targetX, targetY, 0.3)) {
                    transitionToPhase(AutoPhase.CLIMBING);
                }
                break;

            case CLIMBING:
                input.level1 = true;
                input.climberUp = true;
                if (state.climbComplete || phaseTimer > 10.0) {
                    transitionToPhase(AutoPhase.COMPLETE);
                }
                break;

            case COMPLETE:
            default:
                break;
        }
    }

    // ========================================================================
    // AUTO MODE 13: Depot + Climb
    // Strategy: Collect from depot (closer), score all, then climb
    // ========================================================================
    private void updateDepotClimb(RobotState state, InputState input, double dt) {
        switch (currentPhase) {
            case SCORING_PRELOAD:
                // Go to depot first
                transitionToPhase(AutoPhase.DRIVING_TO_DEPOT);
                if (state.alliance == MatchState.Alliance.RED) {
                    targetX = Constants.Field.RED_DEPOT_X;
                    targetY = Constants.Field.RED_DEPOT_Y;
                } else {
                    targetX = Constants.Field.BLUE_DEPOT_X;
                    targetY = Constants.Field.BLUE_DEPOT_Y;
                }
                break;

            case DRIVING_TO_DEPOT:
                driveToTarget(state, input, targetX, targetY);
                if (phaseTimer > 2.5 || isAtTarget(state, targetX, targetY, 0.5)) {
                    transitionToPhase(AutoPhase.COLLECTING_FROM_DEPOT);
                }
                break;

            case COLLECTING_FROM_DEPOT:
                input.intake = true;
                input.forward = 0.2 * Math.sin(phaseTimer * 2);
                if (phaseTimer > 3.0 || state.fuelCount >= 5) {
                    transitionToPhase(AutoPhase.DRIVING_TO_SCORE);
                    if (state.alliance == MatchState.Alliance.RED) {
                        targetX = Constants.Field.RED_HUB_X - 2.5;
                    } else {
                        targetX = Constants.Field.BLUE_HUB_X + 2.5;
                    }
                    targetY = Constants.Field.CENTER_Y;
                }
                break;

            case DRIVING_TO_SCORE:
                driveToTarget(state, input, targetX, targetY);
                if (phaseTimer > 3.0 || isAtTarget(state, targetX, targetY, 1.0)) {
                    transitionToPhase(AutoPhase.SCORING_COLLECTED);
                }
                break;

            case SCORING_COLLECTED:
                if (state.fuelCount > 0) {
                    input.spinUp = true;
                    boolean readyToShoot = aimAtHub(state, input);
                    if (readyToShoot) {
                        input.shoot = true;
                    }
                } else {
                    // Check if time for climb
                    if (totalAutoTime < 16.0) {
                        transitionToPhase(AutoPhase.DRIVING_TO_TOWER);
                        setTowerTarget(state);
                    } else {
                        transitionToPhase(AutoPhase.COMPLETE);
                    }
                }
                break;

            case DRIVING_TO_TOWER:
                driveToTarget(state, input, targetX, targetY);
                if (phaseTimer > 3.0 || isAtTarget(state, targetX, targetY, 0.5)) {
                    transitionToPhase(AutoPhase.CLIMBING);
                }
                break;

            case CLIMBING:
                input.level1 = true;
                input.climberUp = true;
                if (state.climbComplete || phaseTimer > 8.0) {
                    transitionToPhase(AutoPhase.COMPLETE);
                }
                break;

            case COMPLETE:
            default:
                break;
        }
    }

    // ========================================================================
    // AUTO MODE 14: Max Points
    // Strategy: Dynamically choose best path based on timing
    // Target: Maximum possible points (FUEL + climb combo)
    // ========================================================================
    private void updateMaxPoints(RobotState state, InputState input, double dt) {
        switch (currentPhase) {
            case SCORING_PRELOAD:
                if (!isInShootingPosition(state)) {
                    double[] shootPos = getShootingPosition(state);
                    targetX = shootPos[0];
                    targetY = shootPos[1];
                    transitionToPhase(AutoPhase.POSITIONING_TO_SHOOT);
                    return;
                }
                if (state.fuelCount > 0) {
                    input.spinUp = true;
                    boolean readyToShoot = aimAtHub(state, input);
                    if (readyToShoot) {
                        input.shoot = true;
                    }
                } else {
                    // Quick decision: collect or climb?
                    // If early, try to collect first
                    if (totalAutoTime < 2.0) {
                        transitionToPhase(AutoPhase.DRIVING_TO_NEUTRAL);
                        targetX = Constants.Field.CENTER_X - 2.0;
                        targetY = Constants.Field.CENTER_Y;
                    } else {
                        transitionToPhase(AutoPhase.DRIVING_TO_TOWER);
                        setTowerTarget(state);
                    }
                }
                break;

            case POSITIONING_TO_SHOOT:
                driveToPositionAvoidingBumps(state, input, targetX, targetY);
                if (phaseTimer > 3.0 || isAtTarget(state, targetX, targetY, 0.5)) {
                    if (state.fuelCount > 0) {
                        transitionToPhase(AutoPhase.SCORING_COLLECTED);
                    } else {
                        transitionToPhase(AutoPhase.SCORING_PRELOAD);
                    }
                }
                break;

            case DRIVING_TO_NEUTRAL:
                driveToPositionAvoidingBumps(state, input, targetX, targetY);
                if (phaseTimer > 5.0 || isAtTarget(state, targetX, targetY, 0.5)) {
                    transitionToPhase(AutoPhase.INTAKING);
                }
                break;

            case INTAKING:
                input.intake = true;
                input.forward = 0.4;
                // Collect 3-4 FUEL quickly
                if (phaseTimer > 2.5 || state.fuelCount >= 4) {
                    transitionToPhase(AutoPhase.SCORING_COLLECTED);
                }
                break;

            case SCORING_COLLECTED:
                if (!isInShootingPosition(state)) {
                    double[] shootPos = getShootingPosition(state);
                    targetX = shootPos[0];
                    targetY = shootPos[1];
                    transitionToPhase(AutoPhase.POSITIONING_TO_SHOOT);
                    return;
                }
                if (state.fuelCount > 0) {
                    input.spinUp = true;
                    boolean readyToShoot = aimAtHub(state, input);
                    if (readyToShoot) {
                        input.shoot = true;
                    }
                } else {
                    // After scoring, always try to climb
                    if (totalAutoTime < 16.0) {
                        transitionToPhase(AutoPhase.DRIVING_TO_TOWER);
                        setTowerTarget(state);
                    } else {
                        transitionToPhase(AutoPhase.COMPLETE);
                    }
                }
                break;

            case DRIVING_TO_TOWER:
                driveToTarget(state, input, targetX, targetY);
                if (phaseTimer > 3.0 || isAtTarget(state, targetX, targetY, 0.5)) {
                    transitionToPhase(AutoPhase.CLIMBING);
                }
                break;

            case CLIMBING:
                input.level1 = true;
                input.climberUp = true;
                if (state.climbComplete || phaseTimer > 8.0) {
                    transitionToPhase(AutoPhase.COMPLETE);
                }
                break;

            case COMPLETE:
            default:
                break;
        }
    }

    /**
     * Helper to set tower target position for current alliance.
     */
    private void setTowerTarget(RobotState state) {
        if (state.alliance == MatchState.Alliance.RED) {
            targetX = Constants.Field.RED_TOWER_X - 0.5;
            targetY = Constants.Field.RED_TOWER_Y;
        } else {
            targetX = Constants.Field.BLUE_TOWER_X + 0.5;
            targetY = Constants.Field.BLUE_TOWER_Y;
        }
    }

    // ========================================================================
    // AUTO MODE 15: Safe Climb
    // Strategy: Score preload, position safely, then climb with extra time buffer
    // Target: 15-18 points (conservative but reliable)
    // ========================================================================
    private void updateSafeClimb(RobotState state, InputState input, double dt) {
        switch (currentPhase) {
            case SCORING_PRELOAD:
                if (!isInShootingPosition(state)) {
                    double[] shootPos = getShootingPosition(state);
                    targetX = shootPos[0];
                    targetY = shootPos[1];
                    transitionToPhase(AutoPhase.POSITIONING_TO_SHOOT);
                    return;
                }
                if (state.fuelCount > 0) {
                    input.spinUp = true;
                    boolean readyToShoot = aimAtHub(state, input);
                    if (readyToShoot) {
                        input.shoot = true;
                    }
                } else {
                    // Go directly to tower with safe margin
                    transitionToPhase(AutoPhase.DRIVING_TO_TOWER);
                    setTowerTarget(state);
                }
                break;

            case POSITIONING_TO_SHOOT:
                driveToPositionAvoidingBumps(state, input, targetX, targetY);
                if (phaseTimer > 3.0 || isAtTarget(state, targetX, targetY, 0.5)) {
                    transitionToPhase(AutoPhase.SCORING_PRELOAD);
                }
                break;

            case DRIVING_TO_TOWER:
                driveToTarget(state, input, targetX, targetY);
                // Extra time buffer for safe approach
                if (phaseTimer > 4.0 || isAtTarget(state, targetX, targetY, 0.3)) {
                    transitionToPhase(AutoPhase.CLIMBING);
                }
                break;

            case CLIMBING:
                input.level1 = true;
                input.climberUp = true;
                if (state.climbComplete || phaseTimer > 12.0) {
                    transitionToPhase(AutoPhase.COMPLETE);
                }
                break;

            case COMPLETE:
            default:
                break;
        }
    }

    // ========================================================================
    // AUTO MODE 16: Dual Cycle
    // Strategy: Two complete scoring cycles (shoot-collect-shoot twice)
    // Target: 6-8 points (no climbing)
    // ========================================================================
    private void updateDualCycle(RobotState state, InputState input, double dt) {
        switch (currentPhase) {
            case SCORING_PRELOAD:
                if (!isInShootingPosition(state)) {
                    double[] shootPos = getShootingPosition(state);
                    targetX = shootPos[0];
                    targetY = shootPos[1];
                    transitionToPhase(AutoPhase.POSITIONING_TO_SHOOT);
                    return;
                }
                if (state.fuelCount > 0) {
                    input.spinUp = true;
                    boolean readyToShoot = aimAtHub(state, input);
                    if (readyToShoot) {
                        input.shoot = true;
                    }
                } else {
                    // Go collect more FUEL
                    transitionToPhase(AutoPhase.DRIVING_TO_NEUTRAL);
                    targetX = Constants.Field.CENTER_X - 1.5;
                    targetY = Constants.Field.CENTER_Y;
                }
                break;

            case POSITIONING_TO_SHOOT:
                driveToPositionAvoidingBumps(state, input, targetX, targetY);
                if (phaseTimer > 4.0 || isAtTarget(state, targetX, targetY, 0.5)) {
                    if (state.fuelCount > 0) {
                        transitionToPhase(AutoPhase.SCORING_COLLECTED);
                    } else {
                        transitionToPhase(AutoPhase.SCORING_PRELOAD);
                    }
                }
                break;

            case DRIVING_TO_NEUTRAL:
                driveToPositionAvoidingBumps(state, input, targetX, targetY);
                if (phaseTimer > 5.0 || isAtTarget(state, targetX, targetY, 0.5)) {
                    transitionToPhase(AutoPhase.INTAKING);
                }
                break;

            case INTAKING:
                input.intake = true;
                input.forward = 0.4;
                // Collect 3-4 FUEL
                if (phaseTimer > 3.0 || state.fuelCount >= 4) {
                    transitionToPhase(AutoPhase.SCORING_COLLECTED);
                }
                break;

            case SCORING_COLLECTED:
                if (!isInShootingPosition(state)) {
                    double[] shootPos = getShootingPosition(state);
                    targetX = shootPos[0];
                    targetY = shootPos[1];
                    transitionToPhase(AutoPhase.POSITIONING_TO_SHOOT);
                    return;
                }
                if (state.fuelCount > 0) {
                    input.spinUp = true;
                    boolean readyToShoot = aimAtHub(state, input);
                    if (readyToShoot) {
                        input.shoot = true;
                    }
                } else {
                    // Check if time for another cycle
                    if (totalAutoTime < 12.0) {
                        transitionToPhase(AutoPhase.DRIVING_TO_NEUTRAL);
                        targetX = Constants.Field.CENTER_X - 1.5;
                        targetY = Constants.Field.CENTER_Y;
                    } else {
                        transitionToPhase(AutoPhase.COMPLETE);
                    }
                }
                break;

            case COMPLETE:
            default:
                break;
        }
    }

    // ========================================================================
    // AUTO MODE 17: Deny FUEL
    // Strategy: Rush to neutral zone and collect as much FUEL as possible
    // to deny opponents access to game pieces
    // Target: Strategic (denies opponent points)
    // ========================================================================
    private void updateDenyFuel(RobotState state, InputState input, double dt) {
        switch (currentPhase) {
            case DRIVING_TO_NEUTRAL:
                // Rush to center field
                driveToPositionAvoidingBumps(state, input, targetX, targetY);
                if (phaseTimer > 3.0 || isAtTarget(state, targetX, targetY, 0.5)) {
                    transitionToPhase(AutoPhase.INTAKING);
                }
                break;

            case INTAKING:
                input.intake = true;
                // Sweep across neutral zone
                if (phaseTimer < 6.0) {
                    input.forward = 0.3;
                    input.strafe = 0.4 * Math.sin(phaseTimer * 1.5);  // Side-to-side sweep
                }
                // Collect as much as possible
                if (phaseTimer > 8.0 || state.fuelCount >= 8) {
                    transitionToPhase(AutoPhase.SCORING_COLLECTED);
                }
                break;

            case SCORING_COLLECTED:
                if (!isInShootingPosition(state)) {
                    double[] shootPos = getShootingPosition(state);
                    targetX = shootPos[0];
                    targetY = shootPos[1];
                    transitionToPhase(AutoPhase.POSITIONING_TO_SHOOT);
                    return;
                }
                if (state.fuelCount > 0) {
                    input.spinUp = true;
                    boolean readyToShoot = aimAtHub(state, input);
                    if (readyToShoot) {
                        input.shoot = true;
                    }
                } else {
                    transitionToPhase(AutoPhase.COMPLETE);
                }
                break;

            case POSITIONING_TO_SHOOT:
                driveToPositionAvoidingBumps(state, input, targetX, targetY);
                if (phaseTimer > 4.0 || isAtTarget(state, targetX, targetY, 0.5)) {
                    transitionToPhase(AutoPhase.SCORING_COLLECTED);
                }
                break;

            case COMPLETE:
            default:
                break;
        }
    }

    // ========================================================================
    // AUTO MODE 18: Center Control
    // Strategy: Score preload, then control center field to disrupt opponents
    // Target: Strategic (field control)
    // ========================================================================
    private void updateCenterControl(RobotState state, InputState input, double dt) {
        switch (currentPhase) {
            case DRIVING_TO_NEUTRAL:
                // First, drive to center while intaking
                driveToPositionAvoidingBumps(state, input, targetX, targetY);
                input.intake = true;
                if (phaseTimer > 4.0 || isAtTarget(state, targetX, targetY, 0.5)) {
                    transitionToPhase(AutoPhase.INTAKING);
                }
                break;

            case INTAKING:
                input.intake = true;
                // Patrol center area
                double patrolX = Constants.Field.CENTER_X + 1.5 * Math.sin(phaseTimer * 0.5);
                double patrolY = Constants.Field.CENTER_Y + 1.0 * Math.cos(phaseTimer * 0.3);
                driveToTarget(state, input, patrolX, patrolY);

                // After patrolling, score what we have
                if (phaseTimer > 8.0 || state.fuelCount >= 5) {
                    transitionToPhase(AutoPhase.SCORING_COLLECTED);
                }
                break;

            case SCORING_COLLECTED:
                if (!isInShootingPosition(state)) {
                    double[] shootPos = getShootingPosition(state);
                    targetX = shootPos[0];
                    targetY = shootPos[1];
                    transitionToPhase(AutoPhase.POSITIONING_TO_SHOOT);
                    return;
                }
                if (state.fuelCount > 0) {
                    input.spinUp = true;
                    boolean readyToShoot = aimAtHub(state, input);
                    if (readyToShoot) {
                        input.shoot = true;
                    }
                } else {
                    transitionToPhase(AutoPhase.COMPLETE);
                }
                break;

            case POSITIONING_TO_SHOOT:
                driveToPositionAvoidingBumps(state, input, targetX, targetY);
                if (phaseTimer > 4.0 || isAtTarget(state, targetX, targetY, 0.5)) {
                    transitionToPhase(AutoPhase.SCORING_COLLECTED);
                }
                break;

            case COMPLETE:
            default:
                break;
        }
    }

    // ========================================================================
    // AUTO MODE 19: Alliance Support
    // Strategy: Score preload, then position to support alliance partners
    // Target: Strategic (alliance coordination)
    // ========================================================================
    private void updateAllianceSupport(RobotState state, InputState input, double dt) {
        switch (currentPhase) {
            case SCORING_PRELOAD:
                if (!isInShootingPosition(state)) {
                    double[] shootPos = getShootingPosition(state);
                    targetX = shootPos[0];
                    targetY = shootPos[1];
                    transitionToPhase(AutoPhase.POSITIONING_TO_SHOOT);
                    return;
                }
                if (state.fuelCount > 0) {
                    input.spinUp = true;
                    boolean readyToShoot = aimAtHub(state, input);
                    if (readyToShoot) {
                        input.shoot = true;
                    }
                } else {
                    // Position to support alliance (away from tower to leave room)
                    transitionToPhase(AutoPhase.DRIVING_TO_NEUTRAL);
                    // Position at edge of neutral zone, not blocking partners
                    if (state.alliance == MatchState.Alliance.RED) {
                        targetX = Constants.Field.CENTER_X + 1.0;
                    } else {
                        targetX = Constants.Field.CENTER_X - 1.0;
                    }
                    // Offset based on robot ID to avoid collisions
                    targetY = Constants.Field.CENTER_Y + ((state.robotId % 3) - 1) * 2.5;
                }
                break;

            case POSITIONING_TO_SHOOT:
                driveToPositionAvoidingBumps(state, input, targetX, targetY);
                if (phaseTimer > 3.0 || isAtTarget(state, targetX, targetY, 0.5)) {
                    transitionToPhase(AutoPhase.SCORING_PRELOAD);
                }
                break;

            case DRIVING_TO_NEUTRAL:
                // Drive to support position
                driveToPositionAvoidingBumps(state, input, targetX, targetY);
                input.intake = true;  // Collect any nearby FUEL
                if (phaseTimer > 5.0 || isAtTarget(state, targetX, targetY, 0.5)) {
                    transitionToPhase(AutoPhase.HOLDING);
                }
                break;

            case HOLDING:
                // Hold position, ready to assist
                // Continue intaking any nearby FUEL
                input.intake = true;
                // If we have FUEL and time, score it
                if (state.fuelCount >= 3 && totalAutoTime < 15.0) {
                    transitionToPhase(AutoPhase.SCORING_COLLECTED);
                }
                break;

            case SCORING_COLLECTED:
                if (!isInShootingPosition(state)) {
                    double[] shootPos = getShootingPosition(state);
                    targetX = shootPos[0];
                    targetY = shootPos[1];
                    transitionToPhase(AutoPhase.POSITIONING_TO_SHOOT);
                    return;
                }
                if (state.fuelCount > 0) {
                    input.spinUp = true;
                    boolean readyToShoot = aimAtHub(state, input);
                    if (readyToShoot) {
                        input.shoot = true;
                    }
                } else {
                    transitionToPhase(AutoPhase.COMPLETE);
                }
                break;

            case COMPLETE:
            default:
                break;
        }
    }

    // ========================================================================
    // HELPER METHODS
    // ========================================================================

    private void transitionToPhase(AutoPhase newPhase) {
        currentPhase = newPhase;
        phaseTimer = 0;
    }

    private void clearInputs(InputState input) {
        input.forward = 0;
        input.strafe = 0;
        input.turn = 0;
        input.intake = false;
        input.shoot = false;
        input.spinUp = false;
        input.climberUp = false;
        input.climberDown = false;
        input.level1 = false;
        input.level2 = false;
        input.level3 = false;
    }

    private void driveToTarget(RobotState state, InputState input, double targetX, double targetY) {
        driveTowardTarget(state, input, targetX, targetY, 1.0);  // OPTIMIZED: Increased from 0.8 for faster travel
    }

    /**
     * Drive toward a target position at specified speed.
     * Uses field-relative control (SwervePhysics handles the transform).
     */
    private void driveTowardTarget(RobotState state, InputState input, double targetX, double targetY, double maxSpeed) {
        double dx = targetX - state.x;
        double dy = targetY - state.y;
        double distance = Math.sqrt(dx * dx + dy * dy);

        if (distance < 0.1) {
            // Close enough, stop
            if (DEBUG_DRIVING) {
                System.out.printf("[AUTO-DRV R%d] AT TARGET (dist=%.3f)%n", robotId, distance);
            }
            return;
        }

        // Calculate field-relative desired velocity
        double speed = Math.min(maxSpeed, distance * 0.5);

        // Normalize direction vector (field frame)
        double dirX = dx / distance;
        double dirY = dy / distance;

        // Field-relative desired velocities
        double fieldVx = dirX * speed;
        double fieldVy = dirY * speed;

        // Transform to robot-relative coordinates
        // This is necessary because AIRobotController sets fieldRelative = false
        double cos = Math.cos(state.heading);
        double sin = Math.sin(state.heading);

        // Robot-relative: forward is along heading, strafe is perpendicular (left positive)
        input.forward = fieldVx * cos + fieldVy * sin;
        input.strafe = -fieldVx * sin + fieldVy * cos;

        // Turn towards target direction
        double targetAngle = Math.atan2(dy, dx);
        double headingError = normalizeAngle(targetAngle - state.heading);
        if (Math.abs(headingError) > 0.1 && maxSpeed >= 0.8) {
            input.turn = Math.signum(headingError) * Math.min(0.5, Math.abs(headingError));
        }

        if (DEBUG_DRIVING && totalAutoTime - lastDebugTime > 0.3) {
            System.out.printf("[AUTO-DRV R%d] target=(%.2f,%.2f) dist=%.2f spd=%.2f fwd=%.3f str=%.3f turn=%.3f hdg=%.1f°%n",
                robotId, targetX, targetY, distance, speed, input.forward, input.strafe, input.turn, Math.toDegrees(state.heading));
        }
    }

    private boolean isAtTarget(RobotState state, double targetX, double targetY, double tolerance) {
        double dx = targetX - state.x;
        double dy = targetY - state.y;
        return Math.sqrt(dx * dx + dy * dy) < tolerance;
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    /**
     * Check if a position would put the robot on or near a bump.
     * Team 3164's robot cannot traverse bumps.
     */
    private boolean isNearBump(double x, double y, MatchState.Alliance alliance) {
        // Get bump positions for this alliance
        // Bumps flank the HUB in Y direction, centered at HUB_SIZE/2 + BUMP_LENGTH/2 from CENTER_Y
        double bumpX = (alliance == MatchState.Alliance.RED)
            ? Constants.Field.RED_HUB_X : Constants.Field.BLUE_HUB_X;
        double bump1Y = Constants.Field.CENTER_Y + Constants.Field.HUB_SIZE/2 + Constants.Field.BUMP_LENGTH/2;  // Above HUB
        double bump2Y = Constants.Field.CENTER_Y - Constants.Field.HUB_SIZE/2 - Constants.Field.BUMP_LENGTH/2;  // Below HUB

        // Bump extends ±BUMP_WIDTH/2 in X (depth toward field) and ±BUMP_LENGTH/2 in Y (extent along Y)
        double bumpHalfX = Constants.Field.BUMP_WIDTH / 2.0;
        double bumpHalfY = Constants.Field.BUMP_LENGTH / 2.0;
        double robotMargin = Constants.Robot.LENGTH_WITH_BUMPERS / 2.0 + 0.3;

        // Check bump 1 (above hub)
        if (Math.abs(x - bumpX) < bumpHalfX + robotMargin &&
            Math.abs(y - bump1Y) < bumpHalfY + robotMargin) {
            return true;
        }

        // Check bump 2 (below hub)
        if (Math.abs(x - bumpX) < bumpHalfX + robotMargin &&
            Math.abs(y - bump2Y) < bumpHalfY + robotMargin) {
            return true;
        }

        return false;
    }

    /**
     * Get a safe Y position that avoids both hub and bumps.
     * For team 3164, we need to go much wider to clear the bumps.
     */
    private double getSafeYPosition(RobotState state, boolean goAbove) {
        // Bump outer edge is at CENTER_Y ± (HUB_SIZE/2 + BUMP_LENGTH)
        // HUB_SIZE/2 = 0.597, BUMP_LENGTH = 1.854, so outer edge is ~2.45m from CENTER_Y
        // Add robot half-width and safety margin
        double bumpOuterY = Constants.Field.HUB_SIZE/2 + Constants.Field.BUMP_LENGTH;
        double robotMargin = Constants.Robot.LENGTH_WITH_BUMPERS / 2.0 + 0.5;  // Extra 0.5m safety margin
        double safeOffset = bumpOuterY + robotMargin;

        // Team 3164 can't traverse bumps - need to go wide
        if (state.teamNumber == 3164) {
            safeOffset += 0.5;  // Extra margin for team 3164
        }

        if (goAbove) {
            return Constants.Field.CENTER_Y + safeOffset;
        } else {
            return Constants.Field.CENTER_Y - safeOffset;
        }
    }

    /**
     * Drive to a position while avoiding bumps (for team 3164).
     * This method handles navigation around the hub/bump area safely.
     *
     * @param state Robot state
     * @param input Input state to modify
     * @param destX Destination X
     * @param destY Destination Y
     */
    private void driveToPositionAvoidingBumps(RobotState state, InputState input, double destX, double destY) {
        double hubX = (state.alliance == MatchState.Alliance.RED)
            ? Constants.Field.RED_HUB_X : Constants.Field.BLUE_HUB_X;

        // Check if robot is in the danger zone (near hub/bumps)
        double dangerZoneHalfX = Constants.Field.BUMP_WIDTH / 2.0 + Constants.Robot.LENGTH_WITH_BUMPERS / 2.0 + 0.5;
        boolean inDangerZone = Math.abs(state.x - hubX) < dangerZoneHalfX;

        // Determine safe Y position (above or below bumps)
        boolean goAbove = state.y >= Constants.Field.CENTER_Y;
        double safeY = getSafeYPosition(state, goAbove);
        boolean onSafePath = goAbove ? (state.y >= safeY - 0.5) : (state.y <= safeY + 0.5);

        // Determine direction of travel (toward or away from hub)
        boolean goingTowardNeutral = destX > hubX;

        if (DEBUG_DRIVING && totalAutoTime - lastDebugTime > 0.5) {
            lastDebugTime = totalAutoTime;
            System.out.printf("[AUTO-NAV R%d@%.1fs] pos=(%.2f,%.2f) dest=(%.2f,%.2f) hubX=%.2f dangerZone=%.2f%n",
                robotId, totalAutoTime, state.x, state.y, destX, destY, hubX, dangerZoneHalfX);
            System.out.printf("           inDanger=%b goAbove=%b safeY=%.2f onSafePath=%b goingToNeutral=%b%n",
                inDangerZone, goAbove, safeY, onSafePath, goingTowardNeutral);
        }

        if (inDangerZone && !onSafePath) {
            // Need to move to safe Y position first
            double stepX = goingTowardNeutral ? 0.3 : -0.3;
            if (DEBUG_DRIVING && totalAutoTime - lastDebugTime > 0.1) {
                System.out.printf("           ACTION: Move to safe Y -> (%.2f, %.2f)%n", state.x + stepX, safeY);
            }
            driveToTarget(state, input, state.x + stepX, safeY);
        } else if (inDangerZone) {
            // On safe path, drive through the gap
            double exitX = goingTowardNeutral ?
                hubX + dangerZoneHalfX + 0.5 :
                hubX - dangerZoneHalfX - 0.5;
            if (DEBUG_DRIVING && totalAutoTime - lastDebugTime > 0.1) {
                System.out.printf("           ACTION: Drive through gap -> (%.2f, %.2f)%n", exitX, safeY);
            }
            driveToTarget(state, input, exitX, safeY);
        } else {
            // Past the hub/bump area, drive directly to destination
            if (DEBUG_DRIVING && totalAutoTime - lastDebugTime > 0.1) {
                System.out.printf("           ACTION: Direct to dest -> (%.2f, %.2f)%n", destX, destY);
            }
            driveToTarget(state, input, destX, destY);
        }
    }

    /**
     * Check if robot is in a valid shooting position.
     * Must be in alliance zone and at safe distance from hub.
     */
    private boolean isInShootingPosition(RobotState state) {
        double hubX = (state.alliance == MatchState.Alliance.RED)
            ? Constants.Field.RED_HUB_X : Constants.Field.BLUE_HUB_X;
        double hubY = Constants.Field.CENTER_Y;

        // Check alliance zone (G407 rule)
        boolean inAllianceZone;
        if (state.alliance == MatchState.Alliance.RED) {
            inAllianceZone = state.x > (Constants.Field.LENGTH - Constants.Field.ALLIANCE_ZONE_DEPTH);
        } else {
            inAllianceZone = state.x < Constants.Field.ALLIANCE_ZONE_DEPTH;
        }

        // Check safe distance from hub (at least 2m to avoid collision)
        double distToHub = Math.hypot(hubX - state.x, hubY - state.y);
        boolean safeDistance = distToHub > 2.0;

        return inAllianceZone && safeDistance;
    }

    /**
     * Get a good shooting position for the robot.
     * Returns position in alliance zone, at safe distance from hub.
     */
    private double[] getShootingPosition(RobotState state) {
        double hubX = (state.alliance == MatchState.Alliance.RED)
            ? Constants.Field.RED_HUB_X : Constants.Field.BLUE_HUB_X;
        double hubY = Constants.Field.CENTER_Y;

        // Position 3m from hub, in alliance zone
        double shootX, shootY;
        if (state.alliance == MatchState.Alliance.RED) {
            shootX = Constants.Field.LENGTH - 2.5;  // Well inside red alliance zone
        } else {
            shootX = 2.5;  // Well inside blue alliance zone
        }

        // Offset Y based on robot ID to spread robots out
        double yOffset = ((state.robotId % 3) - 1) * 2.0;  // -2, 0, or +2 meters
        shootY = hubY + yOffset;
        // Clamp Y to stay on field
        shootY = Math.max(1.5, Math.min(Constants.Field.WIDTH - 1.5, shootY));

        return new double[]{shootX, shootY};
    }

    /**
     * Calculate optimal shot parameters based on distance to hub.
     * Returns array of [shooterAngle (0-1), shooterPower (0-1)].
     */
    private double[] calculateShotParams(RobotState state) {
        double hubX = (state.alliance == MatchState.Alliance.RED)
            ? Constants.Field.RED_HUB_X : Constants.Field.BLUE_HUB_X;
        double hubY = Constants.Field.CENTER_Y;

        double dist = Math.hypot(hubX - state.x, hubY - state.y);

        // Calculate angle and power based on distance
        // At close range (< 3m): low angle, low power
        // At medium range (3-6m): medium angle, medium power
        // At long range (> 6m): high angle, high power
        double angle, power;

        if (dist < 3.0) {
            // Very close - flat trajectory
            angle = 0.15;  // ~11 degrees
            power = 0.35;  // ~10.25 m/s
        } else if (dist < 6.0) {
            // Medium range - moderate arc
            angle = 0.25 + (dist - 3.0) * 0.05;  // 18-33 degrees
            power = 0.45 + (dist - 3.0) * 0.05;  // 11.75-14 m/s
        } else {
            // Long range - high arc
            angle = 0.4 + Math.min(0.3, (dist - 6.0) * 0.05);  // 30-52 degrees
            power = 0.6 + Math.min(0.3, (dist - 6.0) * 0.03);  // 14-18.5 m/s
        }

        return new double[]{Math.min(0.7, angle), Math.min(0.9, power)};
    }

    /**
     * Configure shooter to aim at own alliance's hub.
     * Sets shooter angle/power and turns robot to face hub.
     * Returns true when ready to shoot.
     */
    private boolean aimAtHub(RobotState state, InputState input) {
        double hubX = (state.alliance == MatchState.Alliance.RED)
            ? Constants.Field.RED_HUB_X : Constants.Field.BLUE_HUB_X;
        double hubY = Constants.Field.CENTER_Y;

        // Set optimal shot parameters
        double[] params = calculateShotParams(state);
        input.shooterAngle = params[0];
        input.shooterPower = params[1];

        // Calculate angle to hub
        double angleToHub = Math.atan2(hubY - state.y, hubX - state.x);
        double headingError = normalizeAngle(angleToHub - state.heading);

        // Turn to face hub if needed
        if (Math.abs(headingError) > 0.1) {
            input.turn = Math.signum(headingError) * Math.min(0.6, Math.abs(headingError));
            return false;  // Not aimed yet
        }

        // Check if shooter is ready
        return state.shooterAtAngle && state.shooterAtSpeed;
    }
}
