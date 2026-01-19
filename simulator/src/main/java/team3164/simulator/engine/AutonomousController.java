package team3164.simulator.engine;

import team3164.simulator.Constants;

/**
 * Controls the robot during autonomous period.
 *
 * Implements the 4 auto modes selectable via DIP switch:
 * - 0: Do Nothing
 * - 1: Score & Collect (offensive - shoot all FUEL, collect more)
 * - 2: Quick Climb (defensive - drive to tower, climb L1)
 * - 3: Score Then Climb (maximum - rapid fire then climb L1)
 */
public class AutonomousController {

    // Auto mode constants (match robot code)
    public static final int AUTO_DO_NOTHING = 0;
    public static final int AUTO_SCORE_AND_COLLECT = 1;
    public static final int AUTO_QUICK_CLIMB = 2;
    public static final int AUTO_SCORE_THEN_CLIMB = 3;

    // Auto mode names
    public static final String[] AUTO_MODE_NAMES = {
        "0: Do Nothing",
        "1: Score & Collect",
        "2: Quick Climb",
        "3: Score Then Climb"
    };

    // Selected auto mode (simulates DIP switch)
    private int selectedMode = 0;
    private boolean selectionLocked = false;

    // State machine
    private AutoPhase currentPhase = AutoPhase.IDLE;
    private double phaseTimer = 0;
    private double totalAutoTime = 0;

    // Target positions for auto routines
    private double targetX = 0;
    private double targetY = 0;
    private double targetHeading = 0;

    // Timings (from robot code Constants.AutoConstants)
    private static final double SHOOT_TIME_PER_FUEL = 0.75;
    private static final double DRIVE_TO_NEUTRAL_TIME = 4.0;
    private static final double INTAKE_TIMEOUT = 5.0;
    private static final double CLIMB_TIMEOUT = 12.0;
    private static final double DRIVE_TO_TOWER_TIME = 5.0;

    public enum AutoPhase {
        IDLE,
        // Score & Collect phases
        SCORING_PRELOAD,
        DRIVING_TO_NEUTRAL,
        INTAKING,
        SCORING_COLLECTED,
        // Quick Climb phases
        DRIVING_TO_TOWER,
        CLIMBING,
        HOLDING,
        // Complete
        COMPLETE
    }

    /**
     * Set the selected auto mode (simulates DIP switch).
     * Only works when selection is not locked.
     */
    public void setSelectedMode(int mode) {
        if (!selectionLocked && mode >= 0 && mode <= 3) {
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
                currentPhase = AutoPhase.DRIVING_TO_TOWER;
                // Target tower position (blue alliance)
                targetX = Constants.Field.BLUE_TOWER_X + 1.0;
                targetY = Constants.Field.BLUE_TOWER_Y;
                break;
            case AUTO_SCORE_THEN_CLIMB:
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
    // AUTO MODE 1: Score & Collect
    // ========================================================================
    private void updateScoreAndCollect(RobotState state, InputState input, double dt) {
        switch (currentPhase) {
            case SCORING_PRELOAD:
                // Shoot all preloaded FUEL
                if (state.fuelCount > 0) {
                    input.spinUp = true;
                    input.shoot = true;
                    // Aim at hub
                    input.shooterAngle = 0.6; // 45 degrees
                    input.shooterPower = 0.7;
                } else {
                    // All FUEL shot, move to neutral zone
                    transitionToPhase(AutoPhase.DRIVING_TO_NEUTRAL);
                    targetX = Constants.Field.CENTER_X;
                    targetY = Constants.Field.CENTER_Y;
                }
                break;

            case DRIVING_TO_NEUTRAL:
                // Drive towards neutral zone
                driveToTarget(state, input, targetX, targetY);

                if (phaseTimer > DRIVE_TO_NEUTRAL_TIME || isAtTarget(state, targetX, targetY, 0.5)) {
                    transitionToPhase(AutoPhase.INTAKING);
                }
                break;

            case INTAKING:
                // Intake FUEL in neutral zone
                input.intake = true;

                // Slowly drive forward while intaking
                input.forward = 0.2;

                if (phaseTimer > INTAKE_TIMEOUT || state.fuelCount >= Constants.Intake.MAX_CAPACITY) {
                    if (state.fuelCount > 0) {
                        transitionToPhase(AutoPhase.SCORING_COLLECTED);
                    } else {
                        transitionToPhase(AutoPhase.COMPLETE);
                    }
                }
                break;

            case SCORING_COLLECTED:
                // Shoot collected FUEL
                if (state.fuelCount > 0) {
                    input.spinUp = true;
                    input.shoot = true;
                    input.shooterAngle = 0.6;
                    input.shooterPower = 0.7;
                } else {
                    transitionToPhase(AutoPhase.COMPLETE);
                }
                break;

            case COMPLETE:
            default:
                // Done
                break;
        }
    }

    // ========================================================================
    // AUTO MODE 2: Quick Climb
    // ========================================================================
    private void updateQuickClimb(RobotState state, InputState input, double dt) {
        switch (currentPhase) {
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
                // Rapid fire preloaded FUEL (faster than Score & Collect)
                if (state.fuelCount > 0) {
                    input.spinUp = true;
                    input.shoot = true;
                    input.shooterAngle = 0.5; // Slightly lower angle for speed
                    input.shooterPower = 0.8; // Higher power for faster shots
                } else {
                    // Done shooting, head to tower
                    transitionToPhase(AutoPhase.DRIVING_TO_TOWER);
                    targetX = Constants.Field.BLUE_TOWER_X + 1.0;
                    targetY = Constants.Field.BLUE_TOWER_Y;
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
        double dx = targetX - state.x;
        double dy = targetY - state.y;
        double distance = Math.sqrt(dx * dx + dy * dy);

        if (distance < 0.1) {
            // Close enough, stop
            return;
        }

        // Calculate direction to target
        double targetAngle = Math.atan2(dy, dx);
        double headingError = normalizeAngle(targetAngle - state.heading);

        // Field-relative driving
        double speed = Math.min(0.8, distance * 0.5);

        // Convert to robot-relative if needed
        double cos = Math.cos(state.heading);
        double sin = Math.sin(state.heading);

        // Simplified field-relative control
        input.forward = speed * Math.cos(targetAngle - state.heading);
        input.strafe = -speed * Math.sin(targetAngle - state.heading);

        // Turn towards target
        if (Math.abs(headingError) > 0.1) {
            input.turn = Math.signum(headingError) * Math.min(0.5, Math.abs(headingError));
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
}
