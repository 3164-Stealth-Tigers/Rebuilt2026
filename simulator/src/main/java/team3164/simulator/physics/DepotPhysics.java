package team3164.simulator.physics;

import team3164.simulator.Constants;
import team3164.simulator.engine.FuelState;
import team3164.simulator.engine.FuelState.Fuel;
import team3164.simulator.engine.MatchState;
import team3164.simulator.engine.RobotState;

/**
 * Physics for DEPOT FUEL pickup in REBUILT 2026.
 * Handles robot interaction with DEPOT areas for FUEL collection.
 */
public class DepotPhysics {

    private static final double DEPOT_LENGTH = Constants.Field.DEPOT_LENGTH;
    private static final double DEPOT_WIDTH = Constants.Field.DEPOT_WIDTH;

    // Pickup range from depot edge
    private static final double PICKUP_RANGE = 0.5;

    /**
     * Check if robot is near a DEPOT.
     *
     * @param state Robot state
     * @return true if near any depot
     */
    public static boolean isNearDepot(RobotState state) {
        return isNearRedDepot(state) || isNearBlueDepot(state);
    }

    /**
     * Check if robot is near red DEPOT.
     */
    public static boolean isNearRedDepot(RobotState state) {
        double dx = Math.abs(state.x - Constants.Field.RED_DEPOT_X);
        double dy = Math.abs(state.y - Constants.Field.RED_DEPOT_Y);
        return dx <= DEPOT_LENGTH / 2 + PICKUP_RANGE &&
               dy <= DEPOT_WIDTH / 2 + PICKUP_RANGE;
    }

    /**
     * Check if robot is near blue DEPOT.
     */
    public static boolean isNearBlueDepot(RobotState state) {
        double dx = Math.abs(state.x - Constants.Field.BLUE_DEPOT_X);
        double dy = Math.abs(state.y - Constants.Field.BLUE_DEPOT_Y);
        return dx <= DEPOT_LENGTH / 2 + PICKUP_RANGE &&
               dy <= DEPOT_WIDTH / 2 + PICKUP_RANGE;
    }

    /**
     * Get the DEPOT position for an alliance.
     */
    public static double[] getDepotPosition(MatchState.Alliance alliance) {
        if (alliance == MatchState.Alliance.RED) {
            return new double[]{Constants.Field.RED_DEPOT_X, Constants.Field.RED_DEPOT_Y};
        }
        return new double[]{Constants.Field.BLUE_DEPOT_X, Constants.Field.BLUE_DEPOT_Y};
    }

    /**
     * Check if a position is inside a DEPOT zone.
     */
    public static DepotInfo getDepotInfo(double x, double y) {
        DepotInfo info = new DepotInfo();

        // Check red depot
        double redDx = Math.abs(x - Constants.Field.RED_DEPOT_X);
        double redDy = Math.abs(y - Constants.Field.RED_DEPOT_Y);
        if (redDx <= DEPOT_LENGTH / 2 && redDy <= DEPOT_WIDTH / 2) {
            info.inDepot = true;
            info.alliance = MatchState.Alliance.RED;
            info.depotX = Constants.Field.RED_DEPOT_X;
            info.depotY = Constants.Field.RED_DEPOT_Y;
            return info;
        }

        // Check blue depot
        double blueDx = Math.abs(x - Constants.Field.BLUE_DEPOT_X);
        double blueDy = Math.abs(y - Constants.Field.BLUE_DEPOT_Y);
        if (blueDx <= DEPOT_LENGTH / 2 && blueDy <= DEPOT_WIDTH / 2) {
            info.inDepot = true;
            info.alliance = MatchState.Alliance.BLUE;
            info.depotX = Constants.Field.BLUE_DEPOT_X;
            info.depotY = Constants.Field.BLUE_DEPOT_Y;
            return info;
        }

        return info;  // Not in any depot
    }

    /**
     * Attempt to pick up FUEL from DEPOT area.
     *
     * @param state Robot state
     * @param fuelState FUEL tracking state
     * @return true if FUEL was picked up
     */
    public static boolean attemptPickup(RobotState state, FuelState fuelState) {
        if (!state.canIntakeFuel()) {
            return false;  // Robot at capacity
        }

        // Try to find FUEL near robot in depot area
        double pickupRadius = Constants.Intake.WIDTH + PICKUP_RANGE;
        Fuel fuel = fuelState.pickupFromField(state.x, state.y, pickupRadius);

        if (fuel != null) {
            state.addFuel();
            fuel.owningRobotIndex = 0;  // Mark as owned
            return true;
        }

        return false;
    }

    /**
     * Get distance from robot to nearest depot.
     */
    public static double getDistanceToNearestDepot(RobotState state) {
        double redDist = Math.hypot(
            state.x - Constants.Field.RED_DEPOT_X,
            state.y - Constants.Field.RED_DEPOT_Y
        );
        double blueDist = Math.hypot(
            state.x - Constants.Field.BLUE_DEPOT_X,
            state.y - Constants.Field.BLUE_DEPOT_Y
        );
        return Math.min(redDist, blueDist);
    }

    /**
     * Get distance from robot to its alliance's depot.
     */
    public static double getDistanceToAllianceDepot(RobotState state) {
        double depotX, depotY;
        if (state.alliance == MatchState.Alliance.RED) {
            depotX = Constants.Field.RED_DEPOT_X;
            depotY = Constants.Field.RED_DEPOT_Y;
        } else {
            depotX = Constants.Field.BLUE_DEPOT_X;
            depotY = Constants.Field.BLUE_DEPOT_Y;
        }
        return Math.hypot(state.x - depotX, state.y - depotY);
    }

    /**
     * Information about a DEPOT at a position.
     */
    public static class DepotInfo {
        public boolean inDepot = false;
        public MatchState.Alliance alliance = null;
        public double depotX = 0;
        public double depotY = 0;
    }

    /**
     * Count FUEL available near a DEPOT.
     */
    public static int countFuelNearDepot(FuelState fuelState, MatchState.Alliance alliance) {
        double depotX, depotY;
        if (alliance == MatchState.Alliance.RED) {
            depotX = Constants.Field.RED_DEPOT_X;
            depotY = Constants.Field.RED_DEPOT_Y;
        } else {
            depotX = Constants.Field.BLUE_DEPOT_X;
            depotY = Constants.Field.BLUE_DEPOT_Y;
        }

        int count = 0;
        double searchRadius = DEPOT_LENGTH / 2 + PICKUP_RANGE;

        for (Fuel fuel : fuelState.getFieldFuel()) {
            double dist = Math.hypot(fuel.x - depotX, fuel.y - depotY);
            if (dist <= searchRadius) {
                count++;
            }
        }

        return count;
    }
}
