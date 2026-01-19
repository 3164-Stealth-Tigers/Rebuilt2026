package team3164.simulator.engine;

import team3164.simulator.Constants;
import java.util.ArrayList;
import java.util.List;

/**
 * Tracks all FUEL balls on the field for REBUILT 2026.
 * Handles positions, velocities, and states for up to 504 FUEL.
 */
public class FuelState {

    // ========================================================================
    // FUEL STATES
    // ========================================================================

    public enum FuelLocation {
        ON_FIELD,       // Rolling/sitting on the field
        IN_ROBOT,       // Inside a robot's storage
        IN_FLIGHT,      // Flying through the air (shot)
        IN_HUB,         // Inside a HUB (scored or falling out)
        IN_CHUTE,       // In an alliance's CHUTE (HP controlled)
        IN_CORRAL,      // In an alliance's CORRAL
        IN_DEPOT,       // In a DEPOT
        SCORED          // Successfully scored and counted
    }

    // ========================================================================
    // FUEL DATA STRUCTURE
    // ========================================================================

    public static class Fuel {
        public int id;
        public FuelLocation location;
        public MatchState.Alliance alliance;  // Which alliance's area (null if neutral)

        // Position (meters)
        public double x;
        public double y;
        public double z;  // Height above field

        // Velocity (m/s)
        public double vx;
        public double vy;
        public double vz;

        // Physics state
        public boolean isMoving;
        public double restTimer;  // Time at rest

        // Ownership
        public int owningRobotIndex = -1;  // -1 = no robot

        public Fuel(int id) {
            this.id = id;
            this.location = FuelLocation.ON_FIELD;
            this.alliance = null;
            this.x = 0;
            this.y = 0;
            this.z = Constants.Fuel.RADIUS;
            this.vx = 0;
            this.vy = 0;
            this.vz = 0;
            this.isMoving = false;
            this.restTimer = 0;
        }

        /**
         * Get total speed.
         */
        public double getSpeed() {
            return Math.sqrt(vx * vx + vy * vy + vz * vz);
        }

        /**
         * Check if FUEL is at rest.
         */
        public boolean isAtRest() {
            return !isMoving && restTimer > 0.5;  // Rest for 0.5s to be considered stopped
        }
    }

    // ========================================================================
    // FUEL LISTS
    // ========================================================================

    private final List<Fuel> allFuel = new ArrayList<>();
    private final List<Fuel> fieldFuel = new ArrayList<>();     // ON_FIELD
    private final List<Fuel> flightFuel = new ArrayList<>();    // IN_FLIGHT
    private final List<Fuel> redChuteFuel = new ArrayList<>();
    private final List<Fuel> blueChuteFuel = new ArrayList<>();
    private final List<Fuel> redCorralFuel = new ArrayList<>();
    private final List<Fuel> blueCorralFuel = new ArrayList<>();
    private final List<Fuel> redDepotFuel = new ArrayList<>();
    private final List<Fuel> blueDepotFuel = new ArrayList<>();

    private int nextFuelId = 0;

    // ========================================================================
    // INITIALIZATION
    // ========================================================================

    /**
     * Initialize FUEL for a new match.
     */
    public void reset() {
        allFuel.clear();
        fieldFuel.clear();
        flightFuel.clear();
        redChuteFuel.clear();
        blueChuteFuel.clear();
        redCorralFuel.clear();
        blueCorralFuel.clear();
        redDepotFuel.clear();
        blueDepotFuel.clear();
        nextFuelId = 0;

        // Add FUEL to CHUTES
        for (int i = 0; i < Constants.Fuel.CHUTE_START_COUNT; i++) {
            Fuel redFuel = createFuel();
            redFuel.location = FuelLocation.IN_CHUTE;
            redFuel.alliance = MatchState.Alliance.RED;
            redFuel.x = Constants.Field.RED_OUTPOST_X;
            redFuel.y = Constants.Field.RED_OUTPOST_Y;
            redChuteFuel.add(redFuel);

            Fuel blueFuel = createFuel();
            blueFuel.location = FuelLocation.IN_CHUTE;
            blueFuel.alliance = MatchState.Alliance.BLUE;
            blueFuel.x = Constants.Field.BLUE_OUTPOST_X;
            blueFuel.y = Constants.Field.BLUE_OUTPOST_Y;
            blueChuteFuel.add(blueFuel);
        }

        // Add some initial FUEL to field positions
        initializeFieldFuel();
    }

    /**
     * Create initial FUEL distribution on field.
     * Based on REBUILT 2026 game manual section 6.3.4.1:
     * - Neutral zone FUEL in dense rectangular grid (206in x 72in bounding box)
     * - Alliance depot FUEL (near each alliance wall)
     */
    private void initializeFieldFuel() {
        // ====================================================================
        // NEUTRAL ZONE FUEL (Game Manual Section 6.3.4.1)
        // Bounding box: 206.0in (5.23m) wide x 72.0in (1.83m) deep
        // Solid divider in middle: 2.0in (0.0508m) wide
        // FUEL arranged in dense grid pattern
        // ====================================================================
        double centerX = Constants.Field.CENTER_X;
        double centerY = Constants.Field.CENTER_Y;

        // Neutral zone dimensions from game manual
        final double NEUTRAL_BOX_WIDTH = 5.23;   // 206.0 inches in meters (Y direction)
        final double NEUTRAL_BOX_DEPTH = 1.83;   // 72.0 inches in meters (X direction)
        final double DIVIDER_WIDTH = 0.0508;     // 2.0 inches in meters

        // FUEL spacing (touching, with slight gap for realism)
        final double FUEL_SPACING = Constants.Fuel.DIAMETER * 1.05;  // ~0.1575m

        // Calculate grid dimensions
        // Y direction (width): split by divider
        double halfWidth = (NEUTRAL_BOX_WIDTH - DIVIDER_WIDTH) / 2.0;
        int fuelPerSideY = (int) Math.floor(halfWidth / FUEL_SPACING);

        // X direction (depth): full depth
        int fuelCountX = (int) Math.floor(NEUTRAL_BOX_DEPTH / FUEL_SPACING);

        // Starting positions (centered on field center)
        double startX = centerX - (NEUTRAL_BOX_DEPTH / 2.0) + (FUEL_SPACING / 2.0);

        // Create FUEL grid - LEFT side of divider (negative Y offset)
        double leftStartY = centerY - (DIVIDER_WIDTH / 2.0) - (FUEL_SPACING / 2.0);
        for (int row = 0; row < fuelCountX; row++) {
            double x = startX + row * FUEL_SPACING;
            for (int col = 0; col < fuelPerSideY; col++) {
                double y = leftStartY - col * FUEL_SPACING;
                // Keep within field bounds
                if (y > 0.5 && y < Constants.Field.WIDTH - 0.5) {
                    Fuel fuel = createFuel();
                    fuel.location = FuelLocation.ON_FIELD;
                    fuel.x = x;
                    fuel.y = y;
                    fuel.z = Constants.Fuel.RADIUS;
                    fieldFuel.add(fuel);
                }
            }
        }

        // Create FUEL grid - RIGHT side of divider (positive Y offset)
        double rightStartY = centerY + (DIVIDER_WIDTH / 2.0) + (FUEL_SPACING / 2.0);
        for (int row = 0; row < fuelCountX; row++) {
            double x = startX + row * FUEL_SPACING;
            for (int col = 0; col < fuelPerSideY; col++) {
                double y = rightStartY + col * FUEL_SPACING;
                // Keep within field bounds
                if (y > 0.5 && y < Constants.Field.WIDTH - 0.5) {
                    Fuel fuel = createFuel();
                    fuel.location = FuelLocation.ON_FIELD;
                    fuel.x = x;
                    fuel.y = y;
                    fuel.z = Constants.Fuel.RADIUS;
                    fieldFuel.add(fuel);
                }
            }
        }

        // ====================================================================
        // DEPOT FUEL (alliance-specific, easy autonomous access)
        // Each depot has 5 FUEL for autonomous collection
        // ====================================================================

        // RED depot FUEL
        for (int i = 0; i < 5; i++) {
            Fuel fuel = createFuel();
            fuel.location = FuelLocation.IN_DEPOT;
            fuel.alliance = MatchState.Alliance.RED;
            fuel.x = Constants.Field.RED_DEPOT_X + (Math.random() - 0.5) * Constants.Field.DEPOT_LENGTH * 0.8;
            fuel.y = Constants.Field.RED_DEPOT_Y + (Math.random() - 0.5) * Constants.Field.DEPOT_WIDTH * 0.8;
            fuel.z = Constants.Fuel.RADIUS;
            redDepotFuel.add(fuel);
        }

        // BLUE depot FUEL
        for (int i = 0; i < 5; i++) {
            Fuel fuel = createFuel();
            fuel.location = FuelLocation.IN_DEPOT;
            fuel.alliance = MatchState.Alliance.BLUE;
            fuel.x = Constants.Field.BLUE_DEPOT_X + (Math.random() - 0.5) * Constants.Field.DEPOT_LENGTH * 0.8;
            fuel.y = Constants.Field.BLUE_DEPOT_Y + (Math.random() - 0.5) * Constants.Field.DEPOT_WIDTH * 0.8;
            fuel.z = Constants.Fuel.RADIUS;
            blueDepotFuel.add(fuel);
        }
    }

    /**
     * Create a new FUEL ball.
     */
    private Fuel createFuel() {
        Fuel fuel = new Fuel(nextFuelId++);
        allFuel.add(fuel);
        return fuel;
    }

    // ========================================================================
    // GETTERS
    // ========================================================================

    public List<Fuel> getAllFuel() {
        return allFuel;
    }

    public List<Fuel> getFieldFuel() {
        return fieldFuel;
    }

    public List<Fuel> getFlightFuel() {
        return flightFuel;
    }

    public List<Fuel> getRedChuteFuel() {
        return redChuteFuel;
    }

    public List<Fuel> getBlueChuteFuel() {
        return blueChuteFuel;
    }

    public List<Fuel> getRedCorralFuel() {
        return redCorralFuel;
    }

    public List<Fuel> getBlueCorralFuel() {
        return blueCorralFuel;
    }

    public List<Fuel> getRedDepotFuel() {
        return redDepotFuel;
    }

    public List<Fuel> getBlueDepotFuel() {
        return blueDepotFuel;
    }

    public int getTotalFuelCount() {
        return allFuel.size();
    }

    /**
     * Pick up FUEL from depot.
     *
     * @param x Robot X position
     * @param y Robot Y position
     * @param alliance Which alliance's depot to pick from
     * @param pickupRadius How far the robot can reach
     * @return The picked up FUEL, or null if none in range
     */
    public Fuel pickupFromDepot(double x, double y, MatchState.Alliance alliance, double pickupRadius) {
        List<Fuel> depotFuel = (alliance == MatchState.Alliance.RED) ? redDepotFuel : blueDepotFuel;

        Fuel closest = null;
        double closestDist = Double.MAX_VALUE;

        for (Fuel fuel : depotFuel) {
            double dist = Math.hypot(fuel.x - x, fuel.y - y);
            if (dist < pickupRadius && dist < closestDist) {
                closest = fuel;
                closestDist = dist;
            }
        }

        if (closest != null) {
            depotFuel.remove(closest);
            closest.location = FuelLocation.IN_ROBOT;
            closest.isMoving = false;
        }

        return closest;
    }

    /**
     * Check if robot is near its alliance depot.
     */
    public boolean isNearDepot(double x, double y, MatchState.Alliance alliance) {
        double depotX = (alliance == MatchState.Alliance.RED) ?
                Constants.Field.RED_DEPOT_X : Constants.Field.BLUE_DEPOT_X;
        double depotY = (alliance == MatchState.Alliance.RED) ?
                Constants.Field.RED_DEPOT_Y : Constants.Field.BLUE_DEPOT_Y;

        double dist = Math.hypot(x - depotX, y - depotY);
        return dist < Constants.Field.DEPOT_LENGTH + 0.5;  // Within depot + some margin
    }

    // ========================================================================
    // FUEL TRANSITIONS
    // ========================================================================

    /**
     * Move FUEL from CHUTE to field (HP releases).
     *
     * @param alliance Which alliance's CHUTE
     * @return The released FUEL, or null if chute is empty
     */
    public Fuel releaseFuelFromChute(MatchState.Alliance alliance) {
        List<Fuel> chute = (alliance == MatchState.Alliance.RED) ? redChuteFuel : blueChuteFuel;
        if (chute.isEmpty()) return null;

        Fuel fuel = chute.remove(chute.size() - 1);
        fuel.location = FuelLocation.ON_FIELD;

        // Set release position (at CHUTE opening)
        if (alliance == MatchState.Alliance.RED) {
            fuel.x = Constants.Field.RED_OUTPOST_X - 0.5;
            fuel.y = Constants.Field.RED_OUTPOST_Y;
        } else {
            fuel.x = Constants.Field.BLUE_OUTPOST_X + 0.5;
            fuel.y = Constants.Field.BLUE_OUTPOST_Y;
        }
        fuel.z = Constants.Field.CHUTE_HEIGHT;

        // Give it some initial velocity down the chute
        double angle = Math.toRadians(Constants.Field.CHUTE_SLOPE_ANGLE);
        double speed = 2.0;  // Initial roll speed
        if (alliance == MatchState.Alliance.RED) {
            fuel.vx = -speed * Math.cos(angle);
        } else {
            fuel.vx = speed * Math.cos(angle);
        }
        fuel.vy = 0;
        fuel.vz = -speed * Math.sin(angle);
        fuel.isMoving = true;

        fieldFuel.add(fuel);
        return fuel;
    }

    /**
     * Move FUEL from CORRAL to CHUTE (HP picks up and places).
     *
     * @param alliance Which alliance
     * @return true if transfer successful
     */
    public boolean transferCorralToChute(MatchState.Alliance alliance) {
        List<Fuel> corral = (alliance == MatchState.Alliance.RED) ? redCorralFuel : blueCorralFuel;
        List<Fuel> chute = (alliance == MatchState.Alliance.RED) ? redChuteFuel : blueChuteFuel;

        if (corral.isEmpty() || chute.size() >= Constants.Field.CHUTE_CAPACITY) return false;

        Fuel fuel = corral.remove(0);
        fuel.location = FuelLocation.IN_CHUTE;
        fuel.vx = 0;
        fuel.vy = 0;
        fuel.vz = 0;
        fuel.isMoving = false;
        chute.add(fuel);

        return true;
    }

    /**
     * Add FUEL to CORRAL (from robot or field).
     */
    public void addToCorral(Fuel fuel, MatchState.Alliance alliance) {
        // Remove from current location
        fieldFuel.remove(fuel);
        flightFuel.remove(fuel);

        fuel.location = FuelLocation.IN_CORRAL;
        fuel.alliance = alliance;
        fuel.isMoving = false;
        fuel.vx = 0;
        fuel.vy = 0;
        fuel.vz = 0;

        if (alliance == MatchState.Alliance.RED) {
            fuel.x = Constants.Field.RED_CORRAL_X;
            fuel.y = Constants.Field.RED_CORRAL_Y;
            redCorralFuel.add(fuel);
        } else {
            fuel.x = Constants.Field.BLUE_CORRAL_X;
            fuel.y = Constants.Field.BLUE_CORRAL_Y;
            blueCorralFuel.add(fuel);
        }
    }

    /**
     * Launch FUEL into the air (robot shoots).
     */
    public Fuel launchFuel(double startX, double startY, double startZ,
                          double velX, double velY, double velZ) {
        Fuel fuel = createFuel();
        fuel.location = FuelLocation.IN_FLIGHT;
        fuel.x = startX;
        fuel.y = startY;
        fuel.z = startZ;
        fuel.vx = velX;
        fuel.vy = velY;
        fuel.vz = velZ;
        fuel.isMoving = true;

        flightFuel.add(fuel);
        return fuel;
    }

    /**
     * FUEL lands on field after flight.
     */
    public void landOnField(Fuel fuel) {
        flightFuel.remove(fuel);
        fuel.location = FuelLocation.ON_FIELD;
        fuel.z = Constants.Fuel.RADIUS;
        fuel.vz = 0;
        fuel.isMoving = fuel.vx != 0 || fuel.vy != 0;
        fieldFuel.add(fuel);
    }

    /**
     * FUEL enters HUB.
     */
    public void enterHub(Fuel fuel, MatchState.Alliance hubAlliance) {
        flightFuel.remove(fuel);
        fieldFuel.remove(fuel);
        fuel.location = FuelLocation.IN_HUB;
        fuel.alliance = hubAlliance;
        fuel.isMoving = false;
    }

    /**
     * Mark FUEL as scored.
     */
    public void markScored(Fuel fuel) {
        fuel.location = FuelLocation.SCORED;
    }

    /**
     * Pick up FUEL from field into a robot.
     *
     * @param x Robot X position
     * @param y Robot Y position
     * @param intakeRadius How far the intake can reach
     * @return The picked up FUEL, or null if none in range
     */
    public Fuel pickupFromField(double x, double y, double intakeRadius) {
        Fuel closest = null;
        double closestDist = Double.MAX_VALUE;

        for (Fuel fuel : fieldFuel) {
            if (fuel.location != FuelLocation.ON_FIELD) continue;
            if (fuel.owningRobotIndex >= 0) continue;  // Already owned

            double dist = Math.hypot(fuel.x - x, fuel.y - y);
            if (dist < intakeRadius && dist < closestDist) {
                closest = fuel;
                closestDist = dist;
            }
        }

        if (closest != null) {
            fieldFuel.remove(closest);
            closest.location = FuelLocation.IN_ROBOT;
            closest.isMoving = false;
        }

        return closest;
    }

    /**
     * Get count of FUEL in a specific location.
     */
    public int getCountByLocation(FuelLocation location) {
        int count = 0;
        for (Fuel fuel : allFuel) {
            if (fuel.location == location) count++;
        }
        return count;
    }

    /**
     * Get all FUEL that need physics updates (on field or in flight).
     */
    public List<Fuel> getActiveFuel() {
        List<Fuel> active = new ArrayList<>();
        active.addAll(fieldFuel);
        active.addAll(flightFuel);
        return active;
    }
}
