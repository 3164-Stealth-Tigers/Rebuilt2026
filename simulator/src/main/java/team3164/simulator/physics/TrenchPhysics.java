package team3164.simulator.physics;

import team3164.simulator.Constants;
import team3164.simulator.engine.RobotState;

/**
 * Physics for TRENCH traversal in REBUILT 2026.
 * Handles clearance checking and collision for robots going under trenches.
 */
public class TrenchPhysics {

    private static final double TRENCH_LENGTH = Constants.Field.TRENCH_LENGTH;
    private static final double TRENCH_WIDTH = Constants.Field.TRENCH_WIDTH;
    private static final double TRENCH_CLEARANCE = Constants.Field.TRENCH_CLEARANCE;

    // Trench positions
    private static final double[][] TRENCH_POSITIONS = {
        {Constants.Field.RED_TRENCH_1_X, Constants.Field.RED_TRENCH_1_Y},
        {Constants.Field.RED_TRENCH_2_X, Constants.Field.RED_TRENCH_2_Y},
        {Constants.Field.BLUE_TRENCH_1_X, Constants.Field.BLUE_TRENCH_1_Y},
        {Constants.Field.BLUE_TRENCH_2_X, Constants.Field.BLUE_TRENCH_2_Y}
    };

    /**
     * Check if robot can pass through a trench at its current height.
     *
     * @param state Robot state
     * @return true if robot can pass, false if blocked
     */
    public static boolean canPassTrench(RobotState state) {
        return state.robotHeight <= TRENCH_CLEARANCE || state.trenchMode;
    }

    /**
     * Check if position is inside a trench zone.
     */
    public static TrenchInfo getTrenchInfo(double x, double y) {
        TrenchInfo info = new TrenchInfo();

        for (double[] trenchPos : TRENCH_POSITIONS) {
            double trenchX = trenchPos[0];
            double trenchY = trenchPos[1];

            double halfLength = TRENCH_LENGTH / 2.0;
            double halfWidth = TRENCH_WIDTH / 2.0;

            if (Math.abs(x - trenchX) <= halfLength && Math.abs(y - trenchY) <= halfWidth) {
                info.inTrench = true;
                info.trenchX = trenchX;
                info.trenchY = trenchY;
                return info;
            }
        }

        return info;  // Not in any trench
    }

    /**
     * Update robot state for trench interaction.
     *
     * @param state Robot state
     * @param dt Time step
     * @return true if robot passed through, false if blocked
     */
    public static boolean update(RobotState state, double dt) {
        TrenchInfo info = getTrenchInfo(state.x, state.y);

        if (!info.inTrench) {
            return true;  // Not in trench, no problem
        }

        // Check if robot can fit
        if (canPassTrench(state)) {
            return true;  // Robot fits under trench
        }

        // Robot is too tall - block movement
        return false;
    }

    /**
     * Check if movement from one position to another is blocked by trench.
     *
     * @param fromX Starting X
     * @param fromY Starting Y
     * @param toX Ending X
     * @param toY Ending Y
     * @param robotHeight Robot's current height
     * @return true if movement is blocked
     */
    public static boolean isMovementBlocked(double fromX, double fromY,
                                           double toX, double toY,
                                           double robotHeight) {
        if (robotHeight <= TRENCH_CLEARANCE) {
            return false;  // Robot is short enough to pass
        }

        // Check if path crosses any trench
        TrenchInfo startInfo = getTrenchInfo(fromX, fromY);
        TrenchInfo endInfo = getTrenchInfo(toX, toY);

        // If starting outside trench and ending inside, check collision
        if (!startInfo.inTrench && endInfo.inTrench) {
            return true;  // Would collide with trench entrance
        }

        // If both in different trenches (shouldn't happen normally)
        if (startInfo.inTrench && endInfo.inTrench &&
            (startInfo.trenchX != endInfo.trenchX || startInfo.trenchY != endInfo.trenchY)) {
            return true;
        }

        return false;
    }

    /**
     * Get the push-out vector if robot is stuck in trench.
     *
     * @param x Robot X position
     * @param y Robot Y position
     * @return Array of [pushX, pushY] or null if not stuck
     */
    public static double[] getPushOutVector(double x, double y) {
        TrenchInfo info = getTrenchInfo(x, y);

        if (!info.inTrench) {
            return null;
        }

        // Calculate direction to nearest edge
        double halfLength = TRENCH_LENGTH / 2.0;
        double halfWidth = TRENCH_WIDTH / 2.0;

        double dx = x - info.trenchX;
        double dy = y - info.trenchY;

        // Find nearest edge
        double distToLeftRight = halfLength - Math.abs(dx);
        double distToTopBottom = halfWidth - Math.abs(dy);

        if (distToLeftRight < distToTopBottom) {
            // Push left/right
            return new double[]{Math.signum(dx) * (halfLength + 0.1 - Math.abs(dx)), 0};
        } else {
            // Push top/bottom
            return new double[]{0, Math.signum(dy) * (halfWidth + 0.1 - Math.abs(dy))};
        }
    }

    /**
     * Toggle trench mode on robot.
     */
    public static void toggleTrenchMode(RobotState state) {
        state.trenchMode = !state.trenchMode;
        if (state.trenchMode) {
            state.robotHeight = Constants.Robot.TRENCH_CONFIG_HEIGHT;
        } else {
            state.robotHeight = Constants.Robot.MAX_HEIGHT;
        }
    }

    /**
     * Information about a trench at a position.
     */
    public static class TrenchInfo {
        public boolean inTrench = false;
        public double trenchX = 0;
        public double trenchY = 0;
    }

    /**
     * Get the nearest trench to a position.
     *
     * @return Array of [trenchX, trenchY, distance]
     */
    public static double[] getNearestTrench(double x, double y) {
        double nearestDist = Double.MAX_VALUE;
        double nearestX = 0;
        double nearestY = 0;

        for (double[] trenchPos : TRENCH_POSITIONS) {
            double dist = Math.hypot(x - trenchPos[0], y - trenchPos[1]);
            if (dist < nearestDist) {
                nearestDist = dist;
                nearestX = trenchPos[0];
                nearestY = trenchPos[1];
            }
        }

        return new double[]{nearestX, nearestY, nearestDist};
    }

    /**
     * Check if a position is approaching a trench entrance.
     */
    public static boolean isApproachingTrench(double x, double y, double vx, double vy) {
        double[] nearest = getNearestTrench(x, y);
        double dist = nearest[2];

        if (dist > 3.0) return false;  // Too far

        // Check if moving toward trench
        double dx = nearest[0] - x;
        double dy = nearest[1] - y;
        double dot = dx * vx + dy * vy;

        return dot > 0;  // Moving toward trench
    }
}
