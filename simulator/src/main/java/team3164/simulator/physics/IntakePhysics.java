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

    // Intake physical parameters
    private static final double INTAKE_REACH = 0.4;  // How far in front of robot intake reaches
    private static final double INTAKE_WIDTH = Constants.Intake.WIDTH;

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

        // Calculate intake zone (in front of robot)
        double intakeX = state.x + Math.cos(state.heading) * INTAKE_REACH;
        double intakeY = state.y + Math.sin(state.heading) * INTAKE_REACH;

        // Check for FUEL in intake zone (field fuel)
        Fuel fuel = findFuelInIntakeZone(intakeX, intakeY, state.heading, fuelState);

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
     * Find FUEL in the intake zone.
     */
    private static Fuel findFuelInIntakeZone(double intakeX, double intakeY, double heading,
                                            FuelState fuelState) {
        double halfWidth = INTAKE_WIDTH / 2.0;

        // Calculate intake zone corners
        double perpX = -Math.sin(heading);
        double perpY = Math.cos(heading);

        for (Fuel fuel : fuelState.getFieldFuel()) {
            // Check if FUEL is within intake zone
            double dx = fuel.x - intakeX;
            double dy = fuel.y - intakeY;

            // Project onto intake direction
            double forward = dx * Math.cos(heading) + dy * Math.sin(heading);
            double lateral = dx * perpX + dy * perpY;

            // Check bounds
            if (Math.abs(forward) <= INTAKE_REACH / 2 &&
                Math.abs(lateral) <= halfWidth) {
                return fuel;
            }
        }

        return null;
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
