package team3164.simulator.physics;

import team3164.simulator.Constants;
import team3164.simulator.engine.RobotState;

/**
 * Comprehensive collision detection for REBUILT 2026 field elements.
 * Handles collisions with HUBs, TOWERs, DEPOTs, and BUMPs.
 */
public class CollisionPhysics {

    // Robot half-size with bumpers for collision detection
    private static final double ROBOT_HALF_SIZE = Constants.Robot.LENGTH_WITH_BUMPERS / 2.0;

    // Collision response parameters
    private static final double PUSH_OUT_MARGIN = 0.02;  // Extra margin when pushing out
    private static final double VELOCITY_DAMPING = 0.1;  // Velocity retained after collision

    /**
     * Represents a rectangular obstacle on the field.
     */
    public static class Obstacle {
        public final double x, y;           // Center position
        public final double halfWidth;      // Half-width (X direction)
        public final double halfHeight;     // Half-height (Y direction)
        public final String name;
        public final boolean isDriveable;   // Can robot drive over it (like bumps)?

        public Obstacle(double x, double y, double width, double height, String name, boolean isDriveable) {
            this.x = x;
            this.y = y;
            this.halfWidth = width / 2.0;
            this.halfHeight = height / 2.0;
            this.name = name;
            this.isDriveable = isDriveable;
        }
    }

    // All obstacles on the field
    private static final Obstacle[] OBSTACLES = {
        // HUBs (solid, cannot drive through)
        new Obstacle(Constants.Field.RED_HUB_X, Constants.Field.RED_HUB_Y,
                     Constants.Field.HUB_SIZE, Constants.Field.HUB_SIZE, "Red HUB", false),
        new Obstacle(Constants.Field.BLUE_HUB_X, Constants.Field.BLUE_HUB_Y,
                     Constants.Field.HUB_SIZE, Constants.Field.HUB_SIZE, "Blue HUB", false),

        // TOWERs (solid, cannot drive through)
        new Obstacle(Constants.Field.RED_TOWER_X, Constants.Field.RED_TOWER_Y,
                     Constants.Field.TOWER_LENGTH, Constants.Field.TOWER_WIDTH, "Red TOWER", false),
        new Obstacle(Constants.Field.BLUE_TOWER_X, Constants.Field.BLUE_TOWER_Y,
                     Constants.Field.TOWER_LENGTH, Constants.Field.TOWER_WIDTH, "Blue TOWER", false),

        // DEPOTs (solid, cannot drive through)
        new Obstacle(Constants.Field.RED_DEPOT_X, Constants.Field.RED_DEPOT_Y,
                     Constants.Field.DEPOT_LENGTH, Constants.Field.DEPOT_WIDTH, "Red DEPOT", false),
        new Obstacle(Constants.Field.BLUE_DEPOT_X, Constants.Field.BLUE_DEPOT_Y,
                     Constants.Field.DEPOT_LENGTH, Constants.Field.DEPOT_WIDTH, "Blue DEPOT", false),
    };

    // Bump obstacles (driveable but affect physics)
    private static final Obstacle[] BUMP_OBSTACLES = {
        new Obstacle(Constants.Field.RED_BUMP_1_X, Constants.Field.RED_BUMP_1_Y,
                     Constants.Field.BUMP_LENGTH, Constants.Field.BUMP_WIDTH, "Red BUMP 1", true),
        new Obstacle(Constants.Field.RED_BUMP_2_X, Constants.Field.RED_BUMP_2_Y,
                     Constants.Field.BUMP_LENGTH, Constants.Field.BUMP_WIDTH, "Red BUMP 2", true),
        new Obstacle(Constants.Field.BLUE_BUMP_1_X, Constants.Field.BLUE_BUMP_1_Y,
                     Constants.Field.BUMP_LENGTH, Constants.Field.BUMP_WIDTH, "Blue BUMP 1", true),
        new Obstacle(Constants.Field.BLUE_BUMP_2_X, Constants.Field.BLUE_BUMP_2_Y,
                     Constants.Field.BUMP_LENGTH, Constants.Field.BUMP_WIDTH, "Blue BUMP 2", true),
    };

    /**
     * Result of a collision check.
     */
    public static class CollisionResult {
        public boolean collided = false;
        public double pushX = 0;
        public double pushY = 0;
        public String obstacleName = null;
        public boolean onBump = false;

        public void apply(RobotState state) {
            if (collided) {
                state.x += pushX;
                state.y += pushY;

                // Dampen velocity in collision direction
                if (Math.abs(pushX) > 0.001) {
                    state.vx *= VELOCITY_DAMPING;
                }
                if (Math.abs(pushY) > 0.001) {
                    state.vy *= VELOCITY_DAMPING;
                }
            }
        }
    }

    /**
     * Check collision between robot and an AABB obstacle.
     *
     * @param robotX Robot center X
     * @param robotY Robot center Y
     * @param obstacle The obstacle to check
     * @return CollisionResult with push-out vector if colliding
     */
    private static CollisionResult checkAABBCollision(double robotX, double robotY, Obstacle obstacle) {
        CollisionResult result = new CollisionResult();

        // Calculate overlap on each axis (Minkowski sum approach)
        double combinedHalfWidth = obstacle.halfWidth + ROBOT_HALF_SIZE;
        double combinedHalfHeight = obstacle.halfHeight + ROBOT_HALF_SIZE;

        double dx = robotX - obstacle.x;
        double dy = robotY - obstacle.y;

        double overlapX = combinedHalfWidth - Math.abs(dx);
        double overlapY = combinedHalfHeight - Math.abs(dy);

        // Check if overlapping on both axes
        if (overlapX > 0 && overlapY > 0) {
            result.collided = true;
            result.obstacleName = obstacle.name;
            result.onBump = obstacle.isDriveable;

            // For driveable obstacles (bumps), don't push out - just note we're on it
            if (obstacle.isDriveable) {
                result.collided = false;  // Not a "collision" in the blocking sense
                result.onBump = true;
                return result;
            }

            // Push out along the axis with minimum overlap
            if (overlapX < overlapY) {
                // Push along X axis
                result.pushX = (Math.signum(dx) * overlapX) + (Math.signum(dx) * PUSH_OUT_MARGIN);
            } else {
                // Push along Y axis
                result.pushY = (Math.signum(dy) * overlapY) + (Math.signum(dy) * PUSH_OUT_MARGIN);
            }
        }

        return result;
    }

    /**
     * Check for collisions with all field obstacles and resolve them.
     *
     * @param state Robot state (will be modified if collision occurs)
     * @return CollisionResult describing what happened
     */
    public static CollisionResult checkAndResolveCollisions(RobotState state) {
        CollisionResult finalResult = new CollisionResult();

        // Check solid obstacles
        for (Obstacle obstacle : OBSTACLES) {
            CollisionResult result = checkAABBCollision(state.x, state.y, obstacle);
            if (result.collided) {
                // Apply this collision
                result.apply(state);
                finalResult = result;
            }
        }

        // Check bumps (they don't block, but we track if robot is on one)
        for (Obstacle bump : BUMP_OBSTACLES) {
            CollisionResult result = checkAABBCollision(state.x, state.y, bump);
            if (result.onBump) {
                finalResult.onBump = true;
            }
        }

        return finalResult;
    }

    /**
     * Check if a position would collide with any obstacle.
     * Used for pathfinding and prediction.
     *
     * @param x X position to check
     * @param y Y position to check
     * @return true if position would cause a collision
     */
    public static boolean wouldCollide(double x, double y) {
        for (Obstacle obstacle : OBSTACLES) {
            double combinedHalfWidth = obstacle.halfWidth + ROBOT_HALF_SIZE;
            double combinedHalfHeight = obstacle.halfHeight + ROBOT_HALF_SIZE;

            if (Math.abs(x - obstacle.x) < combinedHalfWidth &&
                Math.abs(y - obstacle.y) < combinedHalfHeight) {
                return true;
            }
        }
        return false;
    }

    /**
     * Check if movement from one point to another would cross an obstacle.
     * Uses simple line-box intersection.
     *
     * @param fromX Starting X
     * @param fromY Starting Y
     * @param toX Ending X
     * @param toY Ending Y
     * @return true if path is blocked
     */
    public static boolean isPathBlocked(double fromX, double fromY, double toX, double toY) {
        // If either endpoint collides, path is blocked
        if (wouldCollide(toX, toY)) {
            return true;
        }

        // Sample along the path to check for intersections
        double dist = Math.hypot(toX - fromX, toY - fromY);
        int samples = (int) Math.ceil(dist / 0.1);  // Sample every 10cm

        for (int i = 1; i < samples; i++) {
            double t = i / (double) samples;
            double x = fromX + t * (toX - fromX);
            double y = fromY + t * (toY - fromY);

            if (wouldCollide(x, y)) {
                return true;
            }
        }

        return false;
    }

    /**
     * Get the nearest obstacle to a position.
     *
     * @param x X position
     * @param y Y position
     * @return Nearest obstacle or null if none nearby
     */
    public static Obstacle getNearestObstacle(double x, double y) {
        Obstacle nearest = null;
        double nearestDist = Double.MAX_VALUE;

        for (Obstacle obstacle : OBSTACLES) {
            double dist = Math.hypot(x - obstacle.x, y - obstacle.y);
            if (dist < nearestDist) {
                nearestDist = dist;
                nearest = obstacle;
            }
        }

        return nearest;
    }

    /**
     * Calculate a safe path around an obstacle.
     * Returns waypoints to navigate around the obstacle.
     *
     * @param fromX Starting X
     * @param fromY Starting Y
     * @param toX Target X
     * @param toY Target Y
     * @return Array of waypoints [[x1,y1], [x2,y2], ...] or null if direct path is clear
     */
    public static double[][] calculateSafePath(double fromX, double fromY, double toX, double toY) {
        if (!isPathBlocked(fromX, fromY, toX, toY)) {
            return null;  // Direct path is clear
        }

        // Find which obstacle is blocking
        for (Obstacle obstacle : OBSTACLES) {
            // Check if this obstacle is between start and end
            double minX = Math.min(fromX, toX) - ROBOT_HALF_SIZE;
            double maxX = Math.max(fromX, toX) + ROBOT_HALF_SIZE;
            double minY = Math.min(fromY, toY) - ROBOT_HALF_SIZE;
            double maxY = Math.max(fromY, toY) + ROBOT_HALF_SIZE;

            if (obstacle.x >= minX && obstacle.x <= maxX &&
                obstacle.y >= minY && obstacle.y <= maxY) {

                // Generate waypoints to go around this obstacle
                double clearance = ROBOT_HALF_SIZE + 0.3;  // Extra clearance

                // Determine which side to go around
                double leftWaypoint = obstacle.x - obstacle.halfWidth - clearance;
                double rightWaypoint = obstacle.x + obstacle.halfWidth + clearance;
                double topWaypoint = obstacle.y + obstacle.halfHeight + clearance;
                double bottomWaypoint = obstacle.y - obstacle.halfHeight - clearance;

                // Choose the shortest path around
                // For simplicity, try going around in Y direction first
                if (fromY < obstacle.y) {
                    // Coming from below, check if going bottom around is shorter
                    return new double[][] {
                        {fromX, bottomWaypoint},
                        {toX, bottomWaypoint}
                    };
                } else {
                    return new double[][] {
                        {fromX, topWaypoint},
                        {toX, topWaypoint}
                    };
                }
            }
        }

        return null;
    }
}
