package team3164.simulator.physics;

import team3164.simulator.Constants;
import team3164.simulator.engine.InputState;
import team3164.simulator.engine.MatchState;
import team3164.simulator.engine.RobotState;

/**
 * Physics for TOWER climbing in REBUILT 2026.
 * Handles rung validation and climbing mechanics.
 */
public class TowerPhysics {

    // Tower zone radius for engagement
    private static final double TOWER_ENGAGEMENT_RADIUS = 2.0;  // meters

    // Rung heights
    private static final double RUNG_LOW = Constants.Field.RUNG_LOW;
    private static final double RUNG_MID = Constants.Field.RUNG_MID;
    private static final double RUNG_HIGH = Constants.Field.RUNG_HIGH;

    // Height tolerance for rung engagement
    private static final double RUNG_TOLERANCE = 0.1;  // meters

    /**
     * Update climbing physics for a robot.
     *
     * @param state Robot state
     * @param input Input state
     * @param matchState Match state
     * @param dt Time step
     */
    public static void update(RobotState state, InputState input, MatchState matchState, double dt) {
        // Check if near alliance's tower
        if (!isNearTower(state)) {
            // Not near tower - reset climbing if was climbing
            if (state.isClimbing && !state.climbComplete) {
                state.isClimbing = false;
                state.climbLevel = 0;
            }
            return;
        }

        // Handle climb level selection
        if (input.level1 && !state.climbComplete) {
            state.climbLevel = 1;
        } else if (input.level2 && !state.climbComplete) {
            state.climbLevel = 2;
        } else if (input.level3 && !state.climbComplete) {
            state.climbLevel = 3;
        }

        // Handle climber movement
        if (state.climbLevel > 0) {
            updateClimberMotion(state, input, dt);
            checkRungEngagement(state, matchState);
        }
    }

    /**
     * Check if robot is near its alliance's tower.
     */
    public static boolean isNearTower(RobotState state) {
        double towerX, towerY;

        if (state.alliance == MatchState.Alliance.RED) {
            towerX = Constants.Field.RED_TOWER_X;
            towerY = Constants.Field.RED_TOWER_Y;
        } else {
            towerX = Constants.Field.BLUE_TOWER_X;
            towerY = Constants.Field.BLUE_TOWER_Y;
        }

        double dist = Math.hypot(state.x - towerX, state.y - towerY);
        return dist <= TOWER_ENGAGEMENT_RADIUS;
    }

    /**
     * Update climber arm/hook motion.
     */
    private static void updateClimberMotion(RobotState state, InputState input, double dt) {
        double targetVelocity = 0;

        if (input.climberUp && !state.climbComplete) {
            targetVelocity = Constants.Climber.MAX_VELOCITY;
        } else if (input.climberDown) {
            targetVelocity = -Constants.Climber.MAX_VELOCITY;
        }

        // Smooth velocity change
        double velocityDiff = targetVelocity - state.climberVelocity;
        double maxChange = Constants.Climber.MAX_ACCELERATION * dt;

        if (Math.abs(velocityDiff) <= maxChange) {
            state.climberVelocity = targetVelocity;
        } else {
            state.climberVelocity += Math.signum(velocityDiff) * maxChange;
        }

        // Update position
        state.climberPosition += state.climberVelocity * dt;

        // Clamp to limits
        if (state.climberPosition < Constants.Climber.MIN_POSITION) {
            state.climberPosition = Constants.Climber.MIN_POSITION;
            state.climberVelocity = 0;
        } else if (state.climberPosition > Constants.Climber.MAX_POSITION) {
            state.climberPosition = Constants.Climber.MAX_POSITION;
            state.climberVelocity = 0;
        }
    }

    /**
     * Check if climber has engaged with a rung.
     */
    private static void checkRungEngagement(RobotState state, MatchState matchState) {
        if (state.climbComplete) return;

        double targetHeight = state.getTargetClimbHeight();
        if (targetHeight == 0) return;

        // Check if climber position matches rung height
        double heightDiff = Math.abs(state.climberPosition - targetHeight);

        if (heightDiff <= RUNG_TOLERANCE) {
            // Engaged with rung
            state.isClimbing = true;

            // If climber is at rest and engaged, climb is complete
            if (Math.abs(state.climberVelocity) < 0.01) {
                completeClimb(state, matchState);
            }
        }
    }

    /**
     * Complete a climb and award points.
     */
    private static void completeClimb(RobotState state, MatchState matchState) {
        state.climbComplete = true;
        state.climberVelocity = 0;

        // Determine points based on level and match phase
        int points;
        if (matchState.isAuto()) {
            // Only L1 in AUTO
            points = Constants.Scoring.TOWER_L1_AUTO;
        } else {
            switch (state.climbLevel) {
                case 1: points = Constants.Scoring.TOWER_L1_TELEOP; break;
                case 2: points = Constants.Scoring.TOWER_L2_TELEOP; break;
                case 3: points = Constants.Scoring.TOWER_L3_TELEOP; break;
                default: points = 0;
            }
        }

        state.towerPoints += points;

        // Record in match state (assuming robot index 0 for single robot sim)
        matchState.scoreTowerClimb(state.alliance, 0, state.climbLevel);
    }

    /**
     * Get the rung height for a given level.
     *
     * @param level Climb level (1-3)
     * @return Height in meters
     */
    public static double getRungHeight(int level) {
        switch (level) {
            case 1: return RUNG_LOW;
            case 2: return RUNG_MID;
            case 3: return RUNG_HIGH;
            default: return 0;
        }
    }

    /**
     * Check if a climb level is valid for current match phase.
     */
    public static boolean isLevelValidForPhase(int level, MatchState matchState) {
        if (matchState.isAuto()) {
            return level == 1;  // Only L1 in AUTO
        }
        return level >= 1 && level <= 3;
    }

    /**
     * Get points for a climb level in current phase.
     */
    public static int getPointsForLevel(int level, MatchState matchState) {
        if (matchState.isAuto()) {
            return Constants.Scoring.TOWER_L1_AUTO;
        }
        switch (level) {
            case 1: return Constants.Scoring.TOWER_L1_TELEOP;
            case 2: return Constants.Scoring.TOWER_L2_TELEOP;
            case 3: return Constants.Scoring.TOWER_L3_TELEOP;
            default: return 0;
        }
    }

    /**
     * Get distance from robot to its alliance's tower.
     */
    public static double getDistanceToTower(RobotState state) {
        double towerX, towerY;

        if (state.alliance == MatchState.Alliance.RED) {
            towerX = Constants.Field.RED_TOWER_X;
            towerY = Constants.Field.RED_TOWER_Y;
        } else {
            towerX = Constants.Field.BLUE_TOWER_X;
            towerY = Constants.Field.BLUE_TOWER_Y;
        }

        return Math.hypot(state.x - towerX, state.y - towerY);
    }

    /**
     * Reset climbing state (for robot reset).
     */
    public static void resetClimb(RobotState state) {
        state.climberPosition = 0;
        state.climberVelocity = 0;
        state.climbLevel = 0;
        state.isClimbing = false;
        state.climbComplete = false;
        state.towerPoints = 0;
    }
}
