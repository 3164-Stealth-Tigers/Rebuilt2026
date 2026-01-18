package team3164.simulator.physics;

import team3164.simulator.Constants;
import team3164.simulator.engine.FuelState;
import team3164.simulator.engine.FuelState.Fuel;
import team3164.simulator.engine.InputState;
import team3164.simulator.engine.MatchState;
import team3164.simulator.engine.RobotState;

/**
 * Physics for OUTPOST (CHUTE and CORRAL) mechanics in REBUILT 2026.
 * Handles human player FUEL management and robot interaction.
 */
public class OutpostPhysics {

    // CHUTE constants
    private static final double CHUTE_RELEASE_INTERVAL = 0.5;  // Minimum time between releases
    private static final double CHUTE_SLOPE_ANGLE = Math.toRadians(Constants.Field.CHUTE_SLOPE_ANGLE);

    // CORRAL constants
    private static final double CORRAL_PICKUP_RANGE = 0.5;  // Robot can pick up from this distance

    // Timers for release rate limiting
    private static double redChuteTimer = 0;
    private static double blueChuteTimer = 0;

    /**
     * Update OUTPOST physics.
     *
     * @param fuelState FUEL tracking state
     * @param matchState Match state
     * @param input Input state (for HP controls)
     * @param dt Time step
     */
    public static void update(FuelState fuelState, MatchState matchState, InputState input, double dt) {
        // Update timers
        if (redChuteTimer > 0) redChuteTimer -= dt;
        if (blueChuteTimer > 0) blueChuteTimer -= dt;

        // Handle HP chute releases
        if (input.redChuteRelease && redChuteTimer <= 0) {
            releaseFuelFromChute(fuelState, matchState, MatchState.Alliance.RED);
            redChuteTimer = CHUTE_RELEASE_INTERVAL;
        }

        if (input.blueChuteRelease && blueChuteTimer <= 0) {
            releaseFuelFromChute(fuelState, matchState, MatchState.Alliance.BLUE);
            blueChuteTimer = CHUTE_RELEASE_INTERVAL;
        }

        // Handle HP corral-to-chute transfers
        if (input.redCorralTransfer) {
            transferCorralToChute(fuelState, matchState, MatchState.Alliance.RED);
        }

        if (input.blueCorralTransfer) {
            transferCorralToChute(fuelState, matchState, MatchState.Alliance.BLUE);
        }

        // Update chute open status in match state
        matchState.redChuteOpen = input.redChuteRelease;
        matchState.blueChuteOpen = input.blueChuteRelease;
    }

    /**
     * Release FUEL from CHUTE onto field.
     *
     * @return The released FUEL, or null if chute empty
     */
    public static Fuel releaseFuelFromChute(FuelState fuelState, MatchState matchState,
                                           MatchState.Alliance alliance) {
        Fuel fuel = fuelState.releaseFuelFromChute(alliance);

        if (fuel != null) {
            // Update match state count
            if (alliance == MatchState.Alliance.RED) {
                matchState.redChuteCount = fuelState.getRedChuteFuel().size();
            } else {
                matchState.blueChuteCount = fuelState.getBlueChuteFuel().size();
            }
        }

        return fuel;
    }

    /**
     * Transfer FUEL from CORRAL to CHUTE.
     *
     * @return true if transfer successful
     */
    public static boolean transferCorralToChute(FuelState fuelState, MatchState matchState,
                                               MatchState.Alliance alliance) {
        boolean success = fuelState.transferCorralToChute(alliance);

        if (success) {
            // Update match state counts
            if (alliance == MatchState.Alliance.RED) {
                matchState.redChuteCount = fuelState.getRedChuteFuel().size();
                matchState.redCorralCount = fuelState.getRedCorralFuel().size();
            } else {
                matchState.blueChuteCount = fuelState.getBlueChuteFuel().size();
                matchState.blueCorralCount = fuelState.getBlueCorralFuel().size();
            }
        }

        return success;
    }

    /**
     * Check if robot is near an OUTPOST for pickup.
     */
    public static boolean isNearOutpost(RobotState state) {
        double outpostX, outpostY;

        if (state.alliance == MatchState.Alliance.RED) {
            outpostX = Constants.Field.RED_OUTPOST_X;
            outpostY = Constants.Field.RED_OUTPOST_Y;
        } else {
            outpostX = Constants.Field.BLUE_OUTPOST_X;
            outpostY = Constants.Field.BLUE_OUTPOST_Y;
        }

        double dist = Math.hypot(state.x - outpostX, state.y - outpostY);
        return dist <= 2.0;  // Within 2 meters
    }

    /**
     * Check if robot is near a CORRAL for deposit.
     */
    public static boolean isNearCorral(RobotState state) {
        double corralX, corralY;

        if (state.alliance == MatchState.Alliance.RED) {
            corralX = Constants.Field.RED_CORRAL_X;
            corralY = Constants.Field.RED_CORRAL_Y;
        } else {
            corralX = Constants.Field.BLUE_CORRAL_X;
            corralY = Constants.Field.BLUE_CORRAL_Y;
        }

        double dist = Math.hypot(state.x - corralX, state.y - corralY);
        return dist <= CORRAL_PICKUP_RANGE + Constants.Robot.WIDTH_WITH_BUMPERS / 2;
    }

    /**
     * Get the CHUTE position for an alliance.
     */
    public static double[] getChutePosition(MatchState.Alliance alliance) {
        if (alliance == MatchState.Alliance.RED) {
            return new double[]{Constants.Field.RED_OUTPOST_X, Constants.Field.RED_OUTPOST_Y};
        }
        return new double[]{Constants.Field.BLUE_OUTPOST_X, Constants.Field.BLUE_OUTPOST_Y};
    }

    /**
     * Get the CORRAL position for an alliance.
     */
    public static double[] getCorralPosition(MatchState.Alliance alliance) {
        if (alliance == MatchState.Alliance.RED) {
            return new double[]{Constants.Field.RED_CORRAL_X, Constants.Field.RED_CORRAL_Y};
        }
        return new double[]{Constants.Field.BLUE_CORRAL_X, Constants.Field.BLUE_CORRAL_Y};
    }

    /**
     * Calculate release position and velocity for FUEL from CHUTE.
     *
     * @param alliance Which alliance's CHUTE
     * @return Array of [x, y, z, vx, vy, vz]
     */
    public static double[] calculateChuteRelease(MatchState.Alliance alliance) {
        double chuteX, chuteY;
        double dirX;

        if (alliance == MatchState.Alliance.RED) {
            chuteX = Constants.Field.RED_OUTPOST_X - 0.5;
            chuteY = Constants.Field.RED_OUTPOST_Y;
            dirX = -1;  // Release toward field
        } else {
            chuteX = Constants.Field.BLUE_OUTPOST_X + 0.5;
            chuteY = Constants.Field.BLUE_OUTPOST_Y;
            dirX = 1;   // Release toward field
        }

        double chuteZ = Constants.Field.CHUTE_HEIGHT;

        // Initial velocity down the chute
        double speed = 2.0;
        double vx = dirX * speed * Math.cos(CHUTE_SLOPE_ANGLE);
        double vy = 0;
        double vz = -speed * Math.sin(CHUTE_SLOPE_ANGLE);

        return new double[]{chuteX, chuteY, chuteZ, vx, vy, vz};
    }

    /**
     * Get CHUTE count for an alliance.
     */
    public static int getChuteCount(FuelState fuelState, MatchState.Alliance alliance) {
        if (alliance == MatchState.Alliance.RED) {
            return fuelState.getRedChuteFuel().size();
        }
        return fuelState.getBlueChuteFuel().size();
    }

    /**
     * Get CORRAL count for an alliance.
     */
    public static int getCorralCount(FuelState fuelState, MatchState.Alliance alliance) {
        if (alliance == MatchState.Alliance.RED) {
            return fuelState.getRedCorralFuel().size();
        }
        return fuelState.getBlueCorralFuel().size();
    }

    /**
     * Check if CHUTE can release (has FUEL and not rate limited).
     */
    public static boolean canRelease(FuelState fuelState, MatchState.Alliance alliance) {
        int count = getChuteCount(fuelState, alliance);
        double timer = (alliance == MatchState.Alliance.RED) ? redChuteTimer : blueChuteTimer;
        return count > 0 && timer <= 0;
    }

    /**
     * Check if transfer from CORRAL to CHUTE is possible.
     */
    public static boolean canTransfer(FuelState fuelState, MatchState.Alliance alliance) {
        int corralCount = getCorralCount(fuelState, alliance);
        int chuteCount = getChuteCount(fuelState, alliance);
        return corralCount > 0 && chuteCount < Constants.Field.CHUTE_CAPACITY;
    }
}
