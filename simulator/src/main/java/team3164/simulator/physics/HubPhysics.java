package team3164.simulator.physics;

import team3164.simulator.Constants;
import team3164.simulator.engine.FuelState;
import team3164.simulator.engine.FuelState.Fuel;
import team3164.simulator.engine.MatchState;
import team3164.simulator.engine.RobotState;

/**
 * Physics and scoring for HUB structures in REBUILT 2026.
 * Handles FUEL entry detection and scoring validation.
 */
public class HubPhysics {

    private static final double HUB_HALF_SIZE = Constants.Field.HUB_SIZE / 2.0;
    private static final double HUB_HEIGHT = Constants.Field.HUB_HEIGHT;
    private static final double FUEL_RADIUS = Constants.Fuel.RADIUS;

    // Minimum entry velocity for a valid shot
    private static final double MIN_ENTRY_VELOCITY = 2.0;  // m/s

    // HUB opening dimensions (top opening)
    private static final double HUB_OPENING_RADIUS = 0.4;  // meters

    /**
     * Check if a FUEL successfully enters a HUB.
     *
     * @param fuel The FUEL to check
     * @param matchState Current match state
     * @param alliance Which alliance's HUB to check
     * @return Points scored (0 if miss or HUB inactive)
     */
    public static int checkScoring(Fuel fuel, MatchState matchState, MatchState.Alliance alliance) {
        double hubX = (alliance == MatchState.Alliance.RED)
            ? Constants.Field.RED_HUB_X
            : Constants.Field.BLUE_HUB_X;
        double hubY = (alliance == MatchState.Alliance.RED)
            ? Constants.Field.RED_HUB_Y
            : Constants.Field.BLUE_HUB_Y;

        // Check if FUEL is within HUB horizontal bounds
        if (!isWithinHubBounds(fuel.x, fuel.y, hubX, hubY)) {
            return 0;
        }

        // Check if FUEL is at correct height (entering from top)
        if (fuel.z > HUB_HEIGHT + FUEL_RADIUS * 2 || fuel.z < 0) {
            return 0;
        }

        // Check if HUB is active
        boolean isActive = matchState.isHubActive(alliance);

        // Calculate points
        if (isActive) {
            return Constants.Scoring.FUEL_ACTIVE_HUB;
        }

        return 0;  // FUEL goes in but doesn't count
    }

    /**
     * Check if position is within HUB horizontal bounds.
     */
    private static boolean isWithinHubBounds(double x, double y, double hubX, double hubY) {
        return Math.abs(x - hubX) <= HUB_HALF_SIZE &&
               Math.abs(y - hubY) <= HUB_HALF_SIZE;
    }

    /**
     * Check if a shot trajectory will hit a HUB.
     * Used for aim assist and trajectory visualization.
     *
     * @param startX Shot starting X
     * @param startY Shot starting Y
     * @param startZ Shot starting height
     * @param vx X velocity
     * @param vy Y velocity
     * @param vz Z velocity
     * @param alliance Which HUB to check
     * @return true if trajectory intersects HUB
     */
    public static boolean willHitHub(double startX, double startY, double startZ,
                                    double vx, double vy, double vz,
                                    MatchState.Alliance alliance) {
        double hubX = (alliance == MatchState.Alliance.RED)
            ? Constants.Field.RED_HUB_X
            : Constants.Field.BLUE_HUB_X;
        double hubY = (alliance == MatchState.Alliance.RED)
            ? Constants.Field.RED_HUB_Y
            : Constants.Field.BLUE_HUB_Y;

        // Predict trajectory (simplified, no drag)
        double gravity = Constants.Fuel.GRAVITY;

        // Find time when x reaches hub
        double t;
        if (Math.abs(vx) > 0.1) {
            t = (hubX - startX) / vx;
        } else {
            return false;  // Not moving toward HUB
        }

        if (t < 0) return false;  // Moving away from HUB

        // Calculate y and z at that time
        double y = startY + vy * t;
        double z = startZ + vz * t - 0.5 * gravity * t * t;

        // Check if within HUB bounds
        return Math.abs(y - hubY) <= HUB_HALF_SIZE &&
               z >= 0 && z <= HUB_HEIGHT + 0.5;
    }

    /**
     * Calculate optimal shot parameters to hit a HUB.
     *
     * @param robotState Robot's current state
     * @param alliance Target HUB alliance
     * @return Array of [angle, velocity] for the shot, or null if no valid shot
     */
    public static double[] calculateOptimalShot(RobotState robotState, MatchState.Alliance alliance) {
        double hubX = (alliance == MatchState.Alliance.RED)
            ? Constants.Field.RED_HUB_X
            : Constants.Field.BLUE_HUB_X;
        double hubY = (alliance == MatchState.Alliance.RED)
            ? Constants.Field.RED_HUB_Y
            : Constants.Field.BLUE_HUB_Y;

        // Distance to HUB
        double dx = hubX - robotState.x;
        double dy = hubY - robotState.y;
        double horizontalDist = Math.hypot(dx, dy);

        // Target height (aim for center of HUB opening)
        double targetZ = HUB_HEIGHT - 0.3;
        double startZ = 1.0;  // Shooter height on robot
        double dz = targetZ - startZ;

        // Calculate angle for given velocity
        // Using simplified projectile motion
        double gravity = Constants.Fuel.GRAVITY;

        // Try different velocities to find a valid shot
        for (double vel = Constants.Shooter.MIN_VELOCITY; vel <= Constants.Shooter.MAX_VELOCITY; vel += 0.5) {
            double v2 = vel * vel;
            double v4 = v2 * v2;
            double discriminant = v4 - gravity * (gravity * horizontalDist * horizontalDist + 2 * dz * v2);

            if (discriminant >= 0) {
                // Two possible angles, choose the lower one (direct shot)
                double tanTheta = (v2 - Math.sqrt(discriminant)) / (gravity * horizontalDist);
                double angle = Math.atan(tanTheta);

                if (angle >= 0 && angle <= Math.toRadians(Constants.Shooter.MAX_ANGLE)) {
                    return new double[]{Math.toDegrees(angle), vel};
                }
            }
        }

        return null;  // No valid shot found
    }

    /**
     * Get distance from robot to HUB.
     */
    public static double getDistanceToHub(RobotState robotState, MatchState.Alliance alliance) {
        double hubX = (alliance == MatchState.Alliance.RED)
            ? Constants.Field.RED_HUB_X
            : Constants.Field.BLUE_HUB_X;
        double hubY = (alliance == MatchState.Alliance.RED)
            ? Constants.Field.RED_HUB_Y
            : Constants.Field.BLUE_HUB_Y;

        return Math.hypot(hubX - robotState.x, hubY - robotState.y);
    }

    /**
     * Get angle from robot to HUB.
     */
    public static double getAngleToHub(RobotState robotState, MatchState.Alliance alliance) {
        double hubX = (alliance == MatchState.Alliance.RED)
            ? Constants.Field.RED_HUB_X
            : Constants.Field.BLUE_HUB_X;
        double hubY = (alliance == MatchState.Alliance.RED)
            ? Constants.Field.RED_HUB_Y
            : Constants.Field.BLUE_HUB_Y;

        return Math.atan2(hubY - robotState.y, hubX - robotState.x);
    }

    /**
     * Check if robot is in good shooting position.
     */
    public static boolean isInShootingRange(RobotState robotState, MatchState.Alliance alliance) {
        double dist = getDistanceToHub(robotState, alliance);
        // Shooting range: 3-10 meters
        return dist >= 3.0 && dist <= 10.0;
    }
}
