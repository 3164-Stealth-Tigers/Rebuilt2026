package team3164.simulator.physics;

import team3164.simulator.Constants;
import team3164.simulator.engine.FuelState;
import team3164.simulator.engine.FuelState.Fuel;
import team3164.simulator.engine.FuelState.FuelLocation;
import team3164.simulator.engine.MatchState;
import team3164.simulator.engine.RobotState;

import java.util.List;

/**
 * Physics simulation for FUEL balls in REBUILT 2026.
 * Handles projectile motion, collisions, and ground rolling.
 */
public class FuelPhysics {

    // Physics constants
    private static final double GRAVITY = Constants.Fuel.GRAVITY;
    private static final double DRAG_COEFF = Constants.Fuel.DRAG_COEFFICIENT;
    private static final double AIR_DENSITY = Constants.Fuel.AIR_DENSITY;
    private static final double RESTITUTION = Constants.Fuel.RESTITUTION;
    private static final double FUEL_RADIUS = Constants.Fuel.RADIUS;
    private static final double FUEL_MASS = Constants.Fuel.MASS;

    // Cross-sectional area of FUEL ball
    private static final double CROSS_SECTION = Math.PI * FUEL_RADIUS * FUEL_RADIUS;

    // Ground friction
    private static final double GROUND_FRICTION = 0.3;
    private static final double ROLLING_FRICTION = 0.05;
    private static final double VELOCITY_THRESHOLD = 0.1;  // Below this, consider stopped

    /**
     * Update all FUEL physics for one tick.
     *
     * @param fuelState The FUEL tracking state
     * @param matchState The match state
     * @param dt Time step in seconds
     */
    public static void update(FuelState fuelState, MatchState matchState, double dt) {
        // Create a copy of the list to avoid ConcurrentModificationException
        List<Fuel> flightFuelCopy = new java.util.ArrayList<>(fuelState.getFlightFuel());
        List<Fuel> fieldFuelCopy = new java.util.ArrayList<>(fuelState.getFieldFuel());

        // Process in-flight FUEL
        for (Fuel fuel : flightFuelCopy) {
            // First check for hub collision (scoring) before updating physics
            checkHubCollision(fuel, fuelState, matchState);

            // If still in flight after hub check, update physics
            if (fuel.location == FuelLocation.IN_FLIGHT) {
                boolean landed = updateFlightPhysics(fuel, dt);
                if (landed) {
                    // Transition to ON_FIELD
                    fuelState.landOnField(fuel);
                }
            }
        }

        // Process on-field FUEL
        for (Fuel fuel : fieldFuelCopy) {
            if (fuel.location == FuelLocation.ON_FIELD) {
                updateGroundPhysics(fuel, dt);
                checkFieldBoundaries(fuel);
                checkCorralEntry(fuel, fuelState);
            }
        }
    }

    /**
     * Update physics for FUEL in flight.
     * Returns true if FUEL has landed and should transition to ON_FIELD.
     */
    private static boolean updateFlightPhysics(Fuel fuel, double dt) {
        // Calculate drag force
        double speed = fuel.getSpeed();
        if (speed > 0) {
            double dragMag = 0.5 * AIR_DENSITY * DRAG_COEFF * CROSS_SECTION * speed * speed;
            double dragX = -dragMag * fuel.vx / speed;
            double dragY = -dragMag * fuel.vy / speed;
            double dragZ = -dragMag * fuel.vz / speed;

            // Apply drag acceleration
            fuel.vx += (dragX / FUEL_MASS) * dt;
            fuel.vy += (dragY / FUEL_MASS) * dt;
            fuel.vz += (dragZ / FUEL_MASS) * dt;
        }

        // Apply gravity
        fuel.vz -= GRAVITY * dt;

        // Update position
        fuel.x += fuel.vx * dt;
        fuel.y += fuel.vy * dt;
        fuel.z += fuel.vz * dt;

        // Check for ground collision
        if (fuel.z <= FUEL_RADIUS) {
            fuel.z = FUEL_RADIUS;
            fuel.vz = -fuel.vz * RESTITUTION;

            // Apply some horizontal damping on bounce
            fuel.vx *= 0.8;
            fuel.vy *= 0.8;

            // If very low bounce, consider landed and transition to ON_FIELD
            if (Math.abs(fuel.vz) < 0.5) {
                fuel.vz = 0;
                return true;  // Signal to transition to ON_FIELD
            }
        }
        return false;
    }

    /**
     * Update physics for FUEL on the ground.
     */
    private static void updateGroundPhysics(Fuel fuel, double dt) {
        double speed = Math.hypot(fuel.vx, fuel.vy);

        if (speed < VELOCITY_THRESHOLD) {
            // FUEL is at rest
            fuel.vx = 0;
            fuel.vy = 0;
            fuel.isMoving = false;
            fuel.restTimer += dt;
            return;
        }

        fuel.isMoving = true;
        fuel.restTimer = 0;

        // Apply rolling friction
        double frictionDecel = ROLLING_FRICTION * GRAVITY;
        double newSpeed = Math.max(0, speed - frictionDecel * dt);

        if (newSpeed > 0) {
            double scale = newSpeed / speed;
            fuel.vx *= scale;
            fuel.vy *= scale;
        } else {
            fuel.vx = 0;
            fuel.vy = 0;
        }

        // Update position
        fuel.x += fuel.vx * dt;
        fuel.y += fuel.vy * dt;

        // Keep on ground
        fuel.z = FUEL_RADIUS;
    }

    /**
     * Check if FUEL enters a HUB.
     */
    private static void checkHubCollision(Fuel fuel, FuelState fuelState, MatchState matchState) {
        // Check red HUB
        if (isInHubZone(fuel, Constants.Field.RED_HUB_X, Constants.Field.RED_HUB_Y)) {
            if (fuel.z <= Constants.Field.HUB_HEIGHT && fuel.z >= FUEL_RADIUS) {
                fuelState.enterHub(fuel, MatchState.Alliance.RED);
                matchState.scoreFuel(MatchState.Alliance.RED, 1);
                fuelState.markScored(fuel);
                return;
            }
        }

        // Check blue HUB
        if (isInHubZone(fuel, Constants.Field.BLUE_HUB_X, Constants.Field.BLUE_HUB_Y)) {
            if (fuel.z <= Constants.Field.HUB_HEIGHT && fuel.z >= FUEL_RADIUS) {
                fuelState.enterHub(fuel, MatchState.Alliance.BLUE);
                matchState.scoreFuel(MatchState.Alliance.BLUE, 1);
                fuelState.markScored(fuel);
            }
        }
    }

    /**
     * Check if FUEL position is within a HUB's scoring zone.
     */
    private static boolean isInHubZone(Fuel fuel, double hubX, double hubY) {
        double halfSize = Constants.Field.HUB_SIZE / 2.0;
        return fuel.x >= hubX - halfSize && fuel.x <= hubX + halfSize &&
               fuel.y >= hubY - halfSize && fuel.y <= hubY + halfSize;
    }

    /**
     * Keep FUEL within field boundaries.
     */
    private static void checkFieldBoundaries(Fuel fuel) {
        double fieldLength = Constants.Field.LENGTH;
        double fieldWidth = Constants.Field.WIDTH;

        // Bounce off walls
        if (fuel.x < FUEL_RADIUS) {
            fuel.x = FUEL_RADIUS;
            fuel.vx = Math.abs(fuel.vx) * RESTITUTION;
        } else if (fuel.x > fieldLength - FUEL_RADIUS) {
            fuel.x = fieldLength - FUEL_RADIUS;
            fuel.vx = -Math.abs(fuel.vx) * RESTITUTION;
        }

        if (fuel.y < FUEL_RADIUS) {
            fuel.y = FUEL_RADIUS;
            fuel.vy = Math.abs(fuel.vy) * RESTITUTION;
        } else if (fuel.y > fieldWidth - FUEL_RADIUS) {
            fuel.y = fieldWidth - FUEL_RADIUS;
            fuel.vy = -Math.abs(fuel.vy) * RESTITUTION;
        }
    }

    /**
     * Check if FUEL enters a CORRAL.
     */
    private static void checkCorralEntry(Fuel fuel, FuelState fuelState) {
        // Check red CORRAL
        if (isInCorralZone(fuel, Constants.Field.RED_CORRAL_X, Constants.Field.RED_CORRAL_Y)) {
            fuelState.addToCorral(fuel, MatchState.Alliance.RED);
            return;
        }

        // Check blue CORRAL
        if (isInCorralZone(fuel, Constants.Field.BLUE_CORRAL_X, Constants.Field.BLUE_CORRAL_Y)) {
            fuelState.addToCorral(fuel, MatchState.Alliance.BLUE);
        }
    }

    /**
     * Check if FUEL position is within a CORRAL zone.
     */
    private static boolean isInCorralZone(Fuel fuel, double corralX, double corralY) {
        double halfL = Constants.Field.CORRAL_LENGTH / 2.0;
        double halfW = Constants.Field.CORRAL_WIDTH / 2.0;
        return fuel.x >= corralX - halfL && fuel.x <= corralX + halfL &&
               fuel.y >= corralY - halfW && fuel.y <= corralY + halfW;
    }

    /**
     * Calculate trajectory for a shot.
     *
     * @param startX Starting X position
     * @param startY Starting Y position
     * @param startZ Starting height
     * @param angle Launch angle in radians (from horizontal)
     * @param velocity Exit velocity in m/s
     * @param heading Direction in radians (0 = toward red alliance)
     * @return Array of [vx, vy, vz]
     */
    public static double[] calculateLaunchVelocity(double startX, double startY, double startZ,
                                                   double angle, double velocity, double heading) {
        double horizontalVel = velocity * Math.cos(angle);
        double verticalVel = velocity * Math.sin(angle);

        double vx = horizontalVel * Math.cos(heading);
        double vy = horizontalVel * Math.sin(heading);
        double vz = verticalVel;

        return new double[]{vx, vy, vz};
    }

    /**
     * Predict where a shot will land (simplified, no drag).
     *
     * @param startX Starting X position
     * @param startY Starting Y position
     * @param startZ Starting height
     * @param vx X velocity
     * @param vy Y velocity
     * @param vz Z velocity
     * @return Array of [landX, landY, flightTime]
     */
    public static double[] predictLanding(double startX, double startY, double startZ,
                                         double vx, double vy, double vz) {
        // Solve for time when z = FUEL_RADIUS
        // z = startZ + vz*t - 0.5*g*t^2 = FUEL_RADIUS
        // -0.5*g*t^2 + vz*t + (startZ - FUEL_RADIUS) = 0
        double a = -0.5 * GRAVITY;
        double b = vz;
        double c = startZ - FUEL_RADIUS;

        double discriminant = b * b - 4 * a * c;
        if (discriminant < 0) {
            return new double[]{startX, startY, 0};
        }

        double t = (-b - Math.sqrt(discriminant)) / (2 * a);
        if (t < 0) {
            t = (-b + Math.sqrt(discriminant)) / (2 * a);
        }

        double landX = startX + vx * t;
        double landY = startY + vy * t;

        return new double[]{landX, landY, t};
    }

    /**
     * Apply kick to a FUEL ball (from robot contact).
     */
    public static void applyKick(Fuel fuel, double kickVx, double kickVy) {
        fuel.vx += kickVx;
        fuel.vy += kickVy;
        fuel.isMoving = true;
        fuel.restTimer = 0;
    }

    /**
     * Check collision between FUEL and robot, apply physics.
     *
     * @return true if collision occurred
     */
    public static boolean checkRobotCollision(Fuel fuel, RobotState robot) {
        // Simple circular collision check
        double dx = fuel.x - robot.x;
        double dy = fuel.y - robot.y;
        double dist = Math.hypot(dx, dy);

        double robotRadius = Constants.Robot.WIDTH_WITH_BUMPERS / 2.0;
        double minDist = robotRadius + FUEL_RADIUS;

        if (dist < minDist && dist > 0) {
            // Push FUEL away
            double overlap = minDist - dist;
            double nx = dx / dist;
            double ny = dy / dist;

            fuel.x += nx * overlap;
            fuel.y += ny * overlap;

            // Transfer some robot velocity
            double robotSpeed = robot.getSpeed();
            if (robotSpeed > 0.5) {
                double kickSpeed = robotSpeed * 0.5;
                fuel.vx = nx * kickSpeed + robot.vx * 0.3;
                fuel.vy = ny * kickSpeed + robot.vy * 0.3;
                fuel.isMoving = true;
            }

            return true;
        }

        return false;
    }
}
