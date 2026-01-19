package team3164.simulator.engine;

import team3164.simulator.Constants;
import team3164.simulator.physics.CollisionPhysics;
import team3164.simulator.physics.TrenchPhysics;

import java.util.Random;

/**
 * AI controller for non-player robots in the simulation.
 * Each AI robot randomly selects an autonomous mode and executes it.
 * During teleop, AI robots perform simple behaviors based on their role.
 */
public class AIRobotController {

    private static final Random random = new Random();

    // AI behavior modes
    public enum AIBehavior {
        AGGRESSIVE,    // Seeks FUEL and scores
        DEFENSIVE,     // Blocks opponents, plays defense
        CLIMBER,       // Prioritizes climbing
        COLLECTOR      // Focuses on collecting FUEL
    }

    private final int robotId;
    private final MatchState.Alliance alliance;
    private final int teamNumber;

    private int selectedAutoMode;
    private AIBehavior teleopBehavior;
    private AutonomousController autoController;

    // AI state
    private double targetX;
    private double targetY;
    private double actionTimer;
    private boolean hasTarget;
    private AIPhase currentPhase;

    public enum AIPhase {
        IDLE,
        MOVING_TO_TARGET,
        SHOOTING,
        INTAKING,
        CLIMBING,
        DEFENDING
    }

    /**
     * Create a new AI robot controller.
     *
     * @param robotId Unique robot ID (0-5)
     * @param alliance Robot's alliance
     * @param teamNumber Team number for display
     */
    public AIRobotController(int robotId, MatchState.Alliance alliance, int teamNumber) {
        this.robotId = robotId;
        this.alliance = alliance;
        this.teamNumber = teamNumber;
        this.autoController = new AutonomousController();

        // Randomly select auto mode
        this.selectedAutoMode = random.nextInt(4);
        autoController.setSelectedMode(selectedAutoMode);

        // Randomly select teleop behavior
        AIBehavior[] behaviors = AIBehavior.values();
        this.teleopBehavior = behaviors[random.nextInt(behaviors.length)];

        this.currentPhase = AIPhase.IDLE;
        this.hasTarget = false;
    }

    /**
     * Get the selected auto mode.
     */
    public int getSelectedAutoMode() {
        return selectedAutoMode;
    }

    /**
     * Get the auto mode name.
     */
    public String getAutoModeName() {
        return autoController.getSelectedModeName();
    }

    /**
     * Get the teleop behavior.
     */
    public AIBehavior getTeleopBehavior() {
        return teleopBehavior;
    }

    /**
     * Get the team number.
     */
    public int getTeamNumber() {
        return teamNumber;
    }

    /**
     * Start autonomous period.
     */
    public void startAuto(RobotState state) {
        autoController.startAuto(state);
    }

    /**
     * Reset the controller.
     */
    public void reset() {
        autoController.reset();
        currentPhase = AIPhase.IDLE;
        hasTarget = false;
        actionTimer = 0;

        // Re-randomize for next match
        selectedAutoMode = random.nextInt(4);
        autoController.setSelectedMode(selectedAutoMode);

        AIBehavior[] behaviors = AIBehavior.values();
        teleopBehavior = behaviors[random.nextInt(behaviors.length)];
    }

    /**
     * Update the AI controller.
     *
     * @param state This robot's state
     * @param input Input to modify
     * @param matchState Current match state
     * @param allRobots All robots for collision avoidance
     * @param dt Time delta
     */
    public void update(RobotState state, InputState input, MatchState matchState,
                       RobotState[] allRobots, double dt) {
        actionTimer += dt;

        // Clear inputs
        clearInputs(input);

        // Manage trench mode - enable when approaching trenches, disable when clear
        manageTrenchMode(state);

        if (!matchState.matchStarted || matchState.matchEnded) {
            return;
        }

        switch (matchState.currentPhase) {
            case AUTO:
                updateAuto(state, input, dt);
                break;
            case TRANSITION:
                // Wait during transition
                break;
            default:
                updateTeleop(state, input, matchState, allRobots, dt);
                break;
        }
    }

    /**
     * Update during autonomous period.
     */
    private void updateAuto(RobotState state, InputState input, double dt) {
        autoController.update(state, input, dt);
    }

    /**
     * Update during teleop period.
     */
    private void updateTeleop(RobotState state, InputState input, MatchState matchState,
                              RobotState[] allRobots, double dt) {
        switch (teleopBehavior) {
            case AGGRESSIVE:
                updateAggressive(state, input, matchState, dt);
                break;
            case DEFENSIVE:
                updateDefensive(state, input, matchState, allRobots, dt);
                break;
            case CLIMBER:
                updateClimber(state, input, matchState, dt);
                break;
            case COLLECTOR:
                updateCollector(state, input, matchState, dt);
                break;
        }
    }

    /**
     * Aggressive behavior - seek FUEL and score.
     */
    private void updateAggressive(RobotState state, InputState input, MatchState matchState, double dt) {
        // If we have FUEL, shoot it
        if (state.fuelCount > 0) {
            currentPhase = AIPhase.SHOOTING;
            // Drive toward active hub
            double hubX = alliance == MatchState.Alliance.RED ?
                    Constants.Field.RED_HUB_X - 2.0 : Constants.Field.BLUE_HUB_X + 2.0;
            double hubY = Constants.Field.CENTER_Y;

            if (driveToTarget(state, input, hubX, hubY, 1.5)) {
                // At hub, shoot
                input.spinUp = true;
                input.shoot = true;
                input.shooterAngle = 0.6;
                input.shooterPower = 0.7;
            }
        } else {
            // Go collect FUEL
            currentPhase = AIPhase.INTAKING;
            if (!hasTarget || actionTimer > 3.0) {
                pickRandomNeutralTarget();
                actionTimer = 0;
            }

            input.intake = true;
            driveToTarget(state, input, targetX, targetY, 0.5);
        }
    }

    /**
     * Defensive behavior - block opponents.
     */
    private void updateDefensive(RobotState state, InputState input, MatchState matchState,
                                 RobotState[] allRobots, double dt) {
        currentPhase = AIPhase.DEFENDING;

        // Find nearest opponent
        RobotState nearestOpponent = null;
        double nearestDist = Double.MAX_VALUE;

        for (RobotState other : allRobots) {
            if (other == state || other.alliance == alliance) continue;

            double dist = Math.hypot(other.x - state.x, other.y - state.y);
            if (dist < nearestDist) {
                nearestDist = dist;
                nearestOpponent = other;
            }
        }

        if (nearestOpponent != null) {
            // Position between opponent and their hub
            double hubX = nearestOpponent.alliance == MatchState.Alliance.RED ?
                    Constants.Field.RED_HUB_X : Constants.Field.BLUE_HUB_X;
            double hubY = Constants.Field.CENTER_Y;

            // Intercept position
            double interceptX = (nearestOpponent.x + hubX) / 2;
            double interceptY = (nearestOpponent.y + hubY) / 2;

            driveToTarget(state, input, interceptX, interceptY, 1.0);
        } else {
            // Patrol neutral zone
            if (!hasTarget || actionTimer > 4.0) {
                pickRandomNeutralTarget();
                actionTimer = 0;
            }
            driveToTarget(state, input, targetX, targetY, 0.5);
        }
    }

    /**
     * Climber behavior - prioritize climbing in end game.
     */
    private void updateClimber(RobotState state, InputState input, MatchState matchState, double dt) {
        double timeRemaining = matchState.getRemainingTime();

        // Start climbing when 30 seconds left
        if (timeRemaining < 30 && !state.climbComplete) {
            currentPhase = AIPhase.CLIMBING;

            // Drive to tower
            double towerX = alliance == MatchState.Alliance.RED ?
                    Constants.Field.RED_TOWER_X - 1.0 : Constants.Field.BLUE_TOWER_X + 1.0;
            double towerY = alliance == MatchState.Alliance.RED ?
                    Constants.Field.RED_TOWER_Y : Constants.Field.BLUE_TOWER_Y;

            if (driveToTarget(state, input, towerX, towerY, 0.5)) {
                // At tower, climb
                input.level2 = true;  // Aim for L2
                input.climberUp = true;
            }
        } else {
            // Before end game, collect and score
            updateAggressive(state, input, matchState, dt);
        }
    }

    /**
     * Collector behavior - focus on gathering FUEL.
     */
    private void updateCollector(RobotState state, InputState input, MatchState matchState, double dt) {
        if (state.fuelCount >= Constants.Intake.MAX_CAPACITY - 1) {
            // Full, go score
            currentPhase = AIPhase.SHOOTING;
            double hubX = alliance == MatchState.Alliance.RED ?
                    Constants.Field.RED_HUB_X - 2.0 : Constants.Field.BLUE_HUB_X + 2.0;
            double hubY = Constants.Field.CENTER_Y;

            if (driveToTarget(state, input, hubX, hubY, 1.5)) {
                input.spinUp = true;
                input.shoot = true;
                input.shooterAngle = 0.6;
                input.shooterPower = 0.7;
            }
        } else {
            // Collect FUEL
            currentPhase = AIPhase.INTAKING;
            if (!hasTarget || actionTimer > 5.0) {
                pickRandomFieldTarget();
                actionTimer = 0;
            }

            input.intake = true;
            driveToTarget(state, input, targetX, targetY, 0.5);
        }
    }

    /**
     * Pick a random target in the neutral zone.
     */
    private void pickRandomNeutralTarget() {
        targetX = Constants.Field.CENTER_X + (random.nextDouble() - 0.5) * 6.0;
        targetY = Constants.Field.CENTER_Y + (random.nextDouble() - 0.5) * 4.0;
        hasTarget = true;
    }

    /**
     * Pick a random target anywhere on field (alliance side preferred).
     */
    private void pickRandomFieldTarget() {
        if (alliance == MatchState.Alliance.RED) {
            targetX = Constants.Field.CENTER_X + random.nextDouble() * 4.0;
        } else {
            targetX = Constants.Field.CENTER_X - random.nextDouble() * 4.0;
        }
        targetY = 1.0 + random.nextDouble() * (Constants.Field.WIDTH - 2.0);
        hasTarget = true;
    }

    /**
     * Drive toward a target position.
     *
     * @return true if at target
     */
    private boolean driveToTarget(RobotState state, InputState input,
                                  double tgtX, double tgtY, double tolerance) {
        double dx = tgtX - state.x;
        double dy = tgtY - state.y;
        double distance = Math.hypot(dx, dy);

        if (distance < tolerance) {
            return true;
        }

        // Check if path is blocked and adjust
        if (CollisionPhysics.wouldCollide(tgtX, tgtY)) {
            // Target is inside obstacle, adjust
            tgtX += (random.nextDouble() - 0.5) * 2.0;
            tgtY += (random.nextDouble() - 0.5) * 2.0;
            dx = tgtX - state.x;
            dy = tgtY - state.y;
            distance = Math.hypot(dx, dy);
        }

        // Calculate direction
        double targetAngle = Math.atan2(dy, dx);
        double speed = Math.min(0.7, distance * 0.3);

        // Field-relative control
        input.forward = speed * Math.cos(targetAngle - state.heading);
        input.strafe = -speed * Math.sin(targetAngle - state.heading);

        // Turn towards movement direction
        double headingError = normalizeAngle(targetAngle - state.heading);
        if (Math.abs(headingError) > 0.2) {
            input.turn = Math.signum(headingError) * Math.min(0.4, Math.abs(headingError) * 0.5);
        }

        return false;
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

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    /**
     * Manage trench mode for AI robots.
     * Automatically enables trench mode when approaching a trench,
     * and disables it when clear of trenches.
     */
    private void manageTrenchMode(RobotState state) {
        // Get nearest trench info
        double[] nearestTrench = TrenchPhysics.getNearestTrench(state.x, state.y);
        double trenchDist = nearestTrench[2];

        // Check if we're in or near a trench
        TrenchPhysics.TrenchInfo trenchInfo = TrenchPhysics.getTrenchInfo(state.x, state.y);

        if (trenchInfo.inTrench) {
            // We're inside a trench - must be in trench mode to pass
            if (!state.trenchMode) {
                enableTrenchMode(state);
            }
        } else if (trenchDist < 2.0) {
            // Approaching a trench - check if we're heading toward it
            if (TrenchPhysics.isApproachingTrench(state.x, state.y, state.vx, state.vy)) {
                if (!state.trenchMode) {
                    enableTrenchMode(state);
                }
            }
        } else if (trenchDist > 3.0 && state.trenchMode) {
            // Far from any trench, safe to exit trench mode
            disableTrenchMode(state);
        }
    }

    /**
     * Enable trench mode - lower robot height to fit under trenches.
     */
    private void enableTrenchMode(RobotState state) {
        state.trenchMode = true;
        state.robotHeight = Constants.Robot.TRENCH_CONFIG_HEIGHT;
    }

    /**
     * Disable trench mode - return to normal height.
     */
    private void disableTrenchMode(RobotState state) {
        state.trenchMode = false;
        state.robotHeight = Constants.Robot.MAX_HEIGHT;
    }
}
