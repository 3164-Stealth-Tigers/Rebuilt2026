package team3164.simulator.physics;

import team3164.simulator.Constants;
import team3164.simulator.engine.RobotState;

/**
 * Physics for BUMP traversal in REBUILT 2026.
 * Handles robot height offset and speed changes when on bumps.
 */
public class BumpPhysics {

    private static final double BUMP_LENGTH = Constants.Field.BUMP_LENGTH;
    private static final double BUMP_WIDTH = Constants.Field.BUMP_WIDTH;
    private static final double BUMP_HEIGHT = Constants.Field.BUMP_HEIGHT;
    private static final double BUMP_ANGLE = Math.toRadians(Constants.Field.BUMP_RAMP_ANGLE);

    // Bump positions
    private static final double[][] BUMP_POSITIONS = {
        {Constants.Field.RED_BUMP_1_X, Constants.Field.RED_BUMP_1_Y},
        {Constants.Field.RED_BUMP_2_X, Constants.Field.RED_BUMP_2_Y},
        {Constants.Field.BLUE_BUMP_1_X, Constants.Field.BLUE_BUMP_1_Y},
        {Constants.Field.BLUE_BUMP_2_X, Constants.Field.BLUE_BUMP_2_Y}
    };

    // Speed reduction when climbing bump
    private static final double BUMP_SPEED_FACTOR = 0.7;

    // Ramp length (horizontal distance of the angled portion)
    private static final double RAMP_LENGTH = BUMP_HEIGHT / Math.tan(BUMP_ANGLE);

    /**
     * Update robot state for bump interaction.
     *
     * @param state Robot state
     * @param dt Time step
     */
    public static void update(RobotState state, double dt) {
        BumpInfo bumpInfo = getBumpInfo(state.x, state.y);

        state.onBump = bumpInfo.onBump;

        if (bumpInfo.onBump) {
            // Adjust robot based on bump position
            if (bumpInfo.onRamp) {
                // Robot is on the ramp portion
                // Apply speed reduction and calculate height
                double heightOffset = bumpInfo.rampProgress * BUMP_HEIGHT;
                state.robotHeight = Constants.Robot.MAX_HEIGHT + heightOffset;

                // Reduce speed when going uphill
                double rampFactor = 1.0 - (1.0 - BUMP_SPEED_FACTOR) * bumpInfo.rampProgress;
                // This is informational - actual speed limiting done in SwervePhysics
            } else {
                // Robot is on top of bump
                state.robotHeight = Constants.Robot.MAX_HEIGHT + BUMP_HEIGHT;
            }
        } else {
            // Not on bump - normal height (unless in trench mode)
            if (!state.trenchMode) {
                state.robotHeight = Constants.Robot.MAX_HEIGHT;
            }
        }
    }

    /**
     * Get information about bump at a position.
     */
    public static BumpInfo getBumpInfo(double x, double y) {
        BumpInfo info = new BumpInfo();

        for (double[] bumpPos : BUMP_POSITIONS) {
            double bumpX = bumpPos[0];
            double bumpY = bumpPos[1];

            // Check if within bump bounds (including ramps)
            double halfLength = BUMP_LENGTH / 2.0 + RAMP_LENGTH;
            double halfWidth = BUMP_WIDTH / 2.0;

            if (Math.abs(x - bumpX) <= halfLength && Math.abs(y - bumpY) <= halfWidth) {
                info.onBump = true;
                info.bumpX = bumpX;
                info.bumpY = bumpY;

                // Determine if on ramp or top
                double distFromCenter = Math.abs(x - bumpX);
                double topHalfLength = BUMP_LENGTH / 2.0;

                if (distFromCenter <= topHalfLength) {
                    // On top of bump
                    info.onRamp = false;
                    info.rampProgress = 1.0;
                } else {
                    // On ramp
                    info.onRamp = true;
                    info.rampProgress = 1.0 - (distFromCenter - topHalfLength) / RAMP_LENGTH;
                    info.rampProgress = Math.max(0, Math.min(1, info.rampProgress));
                }

                return info;
            }
        }

        return info;  // Not on any bump
    }

    /**
     * Check if a position is blocked by a bump (for collision).
     */
    public static boolean isBlockedByBump(double x, double y, double robotHeight) {
        for (double[] bumpPos : BUMP_POSITIONS) {
            double bumpX = bumpPos[0];
            double bumpY = bumpPos[1];

            // Check if in bump collision zone
            double halfLength = BUMP_LENGTH / 2.0;
            double halfWidth = BUMP_WIDTH / 2.0;

            if (Math.abs(x - bumpX) <= halfLength && Math.abs(y - bumpY) <= halfWidth) {
                // Robot is trying to drive through bump wall
                // Only blocked if robot can't climb (height too low or coming too fast)
                return false;  // For now, allow all robots to traverse bumps
            }
        }

        return false;
    }

    /**
     * Get speed multiplier for bump traversal.
     */
    public static double getSpeedMultiplier(double x, double y) {
        BumpInfo info = getBumpInfo(x, y);

        if (!info.onBump) {
            return 1.0;
        }

        if (info.onRamp) {
            // Speed reduced based on how steep the climb is
            return BUMP_SPEED_FACTOR + (1.0 - BUMP_SPEED_FACTOR) * (1.0 - info.rampProgress);
        }

        // On top - slight reduction
        return 0.9;
    }

    /**
     * Information about a bump at a position.
     */
    public static class BumpInfo {
        public boolean onBump = false;
        public boolean onRamp = false;
        public double rampProgress = 0;  // 0 = bottom of ramp, 1 = top
        public double bumpX = 0;
        public double bumpY = 0;

        public double getHeightOffset() {
            if (!onBump) return 0;
            return rampProgress * BUMP_HEIGHT;
        }
    }

    /**
     * Get the nearest bump to a position.
     *
     * @return Array of [bumpX, bumpY, distance] or null if no bumps
     */
    public static double[] getNearestBump(double x, double y) {
        double nearestDist = Double.MAX_VALUE;
        double nearestX = 0;
        double nearestY = 0;

        for (double[] bumpPos : BUMP_POSITIONS) {
            double dist = Math.hypot(x - bumpPos[0], y - bumpPos[1]);
            if (dist < nearestDist) {
                nearestDist = dist;
                nearestX = bumpPos[0];
                nearestY = bumpPos[1];
            }
        }

        return new double[]{nearestX, nearestY, nearestDist};
    }
}
