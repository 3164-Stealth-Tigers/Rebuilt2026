package team3164.simulator.physics;

import team3164.simulator.Constants;
import team3164.simulator.engine.FuelState;
import team3164.simulator.engine.FuelState.Fuel;
import team3164.simulator.engine.InputState;
import team3164.simulator.engine.RobotState;
import team3164.simulator.engine.RobotState.IntakeState;

/**
 * Physics for intake mechanism in REBUILT 2026.
 * Handles FUEL pickup from the field.
 */
public class IntakePhysics {

    // Intake physical parameters - made generous for easier pickup
    private static final double INTAKE_REACH = 0.8;  // How far in front of robot intake reaches
    private static final double INTAKE_WIDTH = 0.8;  // Wider intake zone

    /**
     * Update intake physics - check for FUEL pickup.
     *
     * @param state Robot state
     * @param input Input state
     * @param fuelState FUEL tracking state
     * @param dt Time step
     * @return true if FUEL was picked up
     */
    public static boolean update(RobotState state, InputState input, FuelState fuelState, double dt) {
        // Only check for pickup if actively intaking and have capacity
        if (!input.intake || !state.canIntakeFuel()) {
            return false;
        }

        // Check for FUEL near the robot (simple distance check)
        Fuel fuel = findNearestFuelInRange(state.x, state.y, INTAKE_REACH + 0.5, fuelState);

        if (fuel != null) {
            // Pick up the FUEL
            return pickupFuel(state, fuel, fuelState);
        }

        // Check for FUEL in depot (only own alliance's depot)
        if (fuelState.isNearDepot(state.x, state.y, state.alliance)) {
            Fuel depotFuel = fuelState.pickupFromDepot(state.x, state.y, state.alliance, INTAKE_REACH + 0.5);
            if (depotFuel != null) {
                state.addFuel();
                depotFuel.owningRobotIndex = state.robotId;
                state.intakeState = IntakeState.TRANSFERRING;
                state.intakeTimer = Constants.Intake.TRANSFER_TIME;
                return true;
            }
        }

        return false;
    }

    /**
     * Find the nearest FUEL within pickup range.
     */
    private static Fuel findNearestFuelInRange(double robotX, double robotY, double range, FuelState fuelState) {
        Fuel nearest = null;
        double nearestDist = Double.MAX_VALUE;

        for (Fuel fuel : fuelState.getFieldFuel()) {
            double dist = Math.hypot(fuel.x - robotX, fuel.y - robotY);
            if (dist <= range && dist < nearestDist) {
                nearestDist = dist;
                nearest = fuel;
            }
        }

        return nearest;
    }

    /**
     * Find FUEL in the intake zone.
     * Uses a circular zone around the robot for more forgiving pickup.
     */
    private static Fuel findFuelInIntakeZone(double intakeX, double intakeY, double heading,
                                            FuelState fuelState) {
        // Use a simpler circular pickup zone centered on the robot
        // This is more realistic for a floor-level intake that can grab balls nearby
        double pickupRadius = INTAKE_REACH + INTAKE_WIDTH / 2.0;  // ~1.2m radius

        Fuel nearest = null;
        double nearestDist = Double.MAX_VALUE;

        for (Fuel fuel : fuelState.getFieldFuel()) {
            // Check distance from robot center (not intake position)
            double dx = fuel.x - intakeX + Math.cos(heading) * INTAKE_REACH / 2;
            double dy = fuel.y - intakeY + Math.sin(heading) * INTAKE_REACH / 2;
            double dist = Math.hypot(dx, dy);

            // Prefer FUEL in front of robot but allow any nearby FUEL
            // Give bonus to FUEL in front direction
            double dotProduct = dx * Math.cos(heading) + dy * Math.sin(heading);
            double effectiveDist = dist - (dotProduct > 0 ? 0.3 : 0);  // Slight preference for forward FUEL

            if (dist <= pickupRadius && effectiveDist < nearestDist) {
                nearestDist = effectiveDist;
                nearest = fuel;
            }
        }

        return nearest;
    }

    /**
     * Pick up a FUEL ball.
     */
    private static boolean pickupFuel(RobotState state, Fuel fuel, FuelState fuelState) {
        // Remove from field
        fuelState.getFieldFuel().remove(fuel);

        // Add to robot
        state.addFuel();
        fuel.location = FuelState.FuelLocation.IN_ROBOT;
        fuel.owningRobotIndex = 0;  // Assume single robot simulation

        // Start transfer animation
        state.intakeState = IntakeState.TRANSFERRING;
        state.intakeTimer = Constants.Intake.TRANSFER_TIME;

        return true;
    }

    /**
     * Get the intake position in front of the robot.
     */
    public static double[] getIntakePosition(RobotState state) {
        double x = state.x + Math.cos(state.heading) * INTAKE_REACH;
        double y = state.y + Math.sin(state.heading) * INTAKE_REACH;
        return new double[]{x, y};
    }

    /**
     * Check if there is FUEL available to intake.
     */
    public static boolean isFuelAvailable(RobotState state, FuelState fuelState) {
        double intakeX = state.x + Math.cos(state.heading) * INTAKE_REACH;
        double intakeY = state.y + Math.sin(state.heading) * INTAKE_REACH;

        return findFuelInIntakeZone(intakeX, intakeY, state.heading, fuelState) != null;
    }

    /**
     * Get count of FUEL near the intake.
     */
    public static int countNearbyFuel(RobotState state, FuelState fuelState, double radius) {
        int count = 0;
        for (Fuel fuel : fuelState.getFieldFuel()) {
            double dist = Math.hypot(fuel.x - state.x, fuel.y - state.y);
            if (dist <= radius) {
                count++;
            }
        }
        return count;
    }

    /**
     * Get distance to nearest FUEL on field.
     */
    public static double getDistanceToNearestFuel(RobotState state, FuelState fuelState) {
        double minDist = Double.MAX_VALUE;
        for (Fuel fuel : fuelState.getFieldFuel()) {
            double dist = Math.hypot(fuel.x - state.x, fuel.y - state.y);
            if (dist < minDist) {
                minDist = dist;
            }
        }
        return minDist;
    }

    /**
     * Get the nearest FUEL position.
     */
    public static double[] getNearestFuelPosition(RobotState state, FuelState fuelState) {
        double minDist = Double.MAX_VALUE;
        Fuel nearest = null;

        for (Fuel fuel : fuelState.getFieldFuel()) {
            double dist = Math.hypot(fuel.x - state.x, fuel.y - state.y);
            if (dist < minDist) {
                minDist = dist;
                nearest = fuel;
            }
        }

        if (nearest != null) {
            return new double[]{nearest.x, nearest.y};
        }
        return null;
    }
}
