package team3164.simulator;

/**
 * Simulation constants for REBUILT 2026.
 * Field dimensions and game elements based on official FRC game manual.
 */
public final class Constants {

    private Constants() {}

    // ========================================================================
    // FIELD DIMENSIONS (meters)
    // ========================================================================
    public static final class Field {
        // Field size: 651.2in x 317.7in (swapped from typical orientation)
        public static final double LENGTH = 16.5405;  // 651.2 inches
        public static final double WIDTH = 8.0696;    // 317.7 inches

        // Field center
        public static final double CENTER_X = LENGTH / 2.0;
        public static final double CENTER_Y = WIDTH / 2.0;

        // Alliance Zone boundary (158.6 inches from each alliance wall)
        public static final double ALLIANCE_ZONE_DEPTH = 4.03;

        // ====================================================================
        // HUB (2 per field, one per alliance)
        // ====================================================================
        // HUB dimensions: 47in x 47in (1.194m x 1.194m)
        public static final double HUB_SIZE = 1.194;
        public static final double HUB_HEIGHT = 1.83;          // 72in opening height
        public static final double HUB_DISTANCE_FROM_WALL = 4.03; // Distance from alliance wall

        // Red HUB position (near red alliance wall)
        public static final double RED_HUB_X = LENGTH - HUB_DISTANCE_FROM_WALL;
        public static final double RED_HUB_Y = CENTER_Y;

        // Blue HUB position (near blue alliance wall)
        public static final double BLUE_HUB_X = HUB_DISTANCE_FROM_WALL;
        public static final double BLUE_HUB_Y = CENTER_Y;

        // ====================================================================
        // BUMP (4 per field)
        // ====================================================================
        // BUMP dimensions: 1.854m x 1.128m x 0.165m, 15° ramps
        public static final double BUMP_LENGTH = 1.854;
        public static final double BUMP_WIDTH = 1.128;
        public static final double BUMP_HEIGHT = 0.165;
        public static final double BUMP_RAMP_ANGLE = 15.0;    // degrees

        // BUMP positions (flanking HUBs on either side in Y direction)
        // Red side bumps (flanking red HUB)
        public static final double RED_BUMP_1_X = RED_HUB_X;
        public static final double RED_BUMP_1_Y = CENTER_Y + HUB_SIZE/2 + BUMP_WIDTH/2 + 0.3;
        public static final double RED_BUMP_2_X = RED_HUB_X;
        public static final double RED_BUMP_2_Y = CENTER_Y - HUB_SIZE/2 - BUMP_WIDTH/2 - 0.3;

        // Blue side bumps (flanking blue HUB)
        public static final double BLUE_BUMP_1_X = BLUE_HUB_X;
        public static final double BLUE_BUMP_1_Y = CENTER_Y + HUB_SIZE/2 + BUMP_WIDTH/2 + 0.3;
        public static final double BLUE_BUMP_2_X = BLUE_HUB_X;
        public static final double BLUE_BUMP_2_Y = CENTER_Y - HUB_SIZE/2 - BUMP_WIDTH/2 - 0.3;

        // ====================================================================
        // TRENCH (4 per field)
        // ====================================================================
        // TRENCH dimensions: 1.668m x 1.194m x 1.022m, clearance 0.565m
        public static final double TRENCH_LENGTH = 1.668;
        public static final double TRENCH_WIDTH = 1.194;
        public static final double TRENCH_HEIGHT = 1.022;
        public static final double TRENCH_CLEARANCE = 0.565;  // Robot must be under this

        // TRENCH positions (at field edges near guardrails)
        // Red side trenches
        public static final double RED_TRENCH_1_X = RED_HUB_X - 1.5;
        public static final double RED_TRENCH_1_Y = WIDTH - TRENCH_WIDTH/2 - 0.1;
        public static final double RED_TRENCH_2_X = RED_HUB_X - 1.5;
        public static final double RED_TRENCH_2_Y = TRENCH_WIDTH/2 + 0.1;

        // Blue side trenches
        public static final double BLUE_TRENCH_1_X = BLUE_HUB_X + 1.5;
        public static final double BLUE_TRENCH_1_Y = WIDTH - TRENCH_WIDTH/2 - 0.1;
        public static final double BLUE_TRENCH_2_X = BLUE_HUB_X + 1.5;
        public static final double BLUE_TRENCH_2_Y = TRENCH_WIDTH/2 + 0.1;

        // ====================================================================
        // TOWER (2 per field, one per alliance)
        // ====================================================================
        // TOWER dimensions: 1.251m x 1.143m x 1.988m
        public static final double TOWER_LENGTH = 1.251;
        public static final double TOWER_WIDTH = 1.143;
        public static final double TOWER_HEIGHT = 1.988;

        // RUNG heights
        public static final double RUNG_LOW = 0.686;          // 27 inches
        public static final double RUNG_MID = 1.143;          // 45 inches
        public static final double RUNG_HIGH = 1.60;          // 63 inches

        // TOWER positions (at alliance walls, integrated into wall structure)
        public static final double TOWER_DISTANCE_FROM_WALL = 0.625;  // Half tower length

        public static final double RED_TOWER_X = LENGTH - TOWER_DISTANCE_FROM_WALL;
        public static final double RED_TOWER_Y = CENTER_Y + 2.0;

        public static final double BLUE_TOWER_X = TOWER_DISTANCE_FROM_WALL;
        public static final double BLUE_TOWER_Y = CENTER_Y + 2.0;

        // ====================================================================
        // DEPOT (2 per field, one per alliance)
        // ====================================================================
        // DEPOT dimensions: 1.07m x 0.686m
        public static final double DEPOT_LENGTH = 1.07;
        public static final double DEPOT_WIDTH = 0.686;

        // DEPOT positions (near alliance walls)
        public static final double RED_DEPOT_X = LENGTH - 1.0;
        public static final double RED_DEPOT_Y = 1.5;

        public static final double BLUE_DEPOT_X = 1.0;
        public static final double BLUE_DEPOT_Y = 1.5;

        // ====================================================================
        // OUTPOST / CHUTE / CORRAL (2 per field, one per alliance)
        // ====================================================================
        // OUTPOST CHUTE opening: 0.808m x 0.178m at 0.714m height
        public static final double CHUTE_OPENING_WIDTH = 0.808;
        public static final double CHUTE_OPENING_HEIGHT = 0.178;
        public static final double CHUTE_HEIGHT = 0.714;
        public static final double CHUTE_CAPACITY = 25;       // Max FUEL in chute
        public static final double CHUTE_SLOPE_ANGLE = 15.0;  // degrees

        // CORRAL dimensions: 35.8in x 37.6in (0.909m x 0.955m)
        public static final double CORRAL_LENGTH = 0.909;
        public static final double CORRAL_WIDTH = 0.955;
        public static final double CORRAL_OPENING_WIDTH = 0.813;  // 32.0in
        public static final double CORRAL_OPENING_HEIGHT = 0.048; // 1.88in

        // OUTPOST positions (at field corners)
        public static final double RED_OUTPOST_X = LENGTH - 0.5;
        public static final double RED_OUTPOST_Y = WIDTH - 1.0;

        public static final double BLUE_OUTPOST_X = 0.5;
        public static final double BLUE_OUTPOST_Y = WIDTH - 1.0;

        // CORRAL positions (near alliance walls)
        public static final double RED_CORRAL_X = LENGTH - 1.5;
        public static final double RED_CORRAL_Y = 0.5;

        public static final double BLUE_CORRAL_X = 1.5;
        public static final double BLUE_CORRAL_Y = 0.5;

        // ====================================================================
        // APRILTAGS (for vision-based localization)
        // ====================================================================
        // AprilTag dimensions (6.5" = 16.5cm standard FRC size)
        public static final double APRILTAG_SIZE = 0.165;  // meters

        // AprilTag positions: {id, x, y, z, rotation_degrees}
        // Rotation: 0 = facing +X (red wall), 90 = facing +Y, 180 = facing -X (blue wall), 270 = facing -Y
        public static final double[][] APRILTAG_POSITIONS = {
            // BLUE HUB AprilTags (Tags 1-3) - facing toward field center (+X direction)
            {1, BLUE_HUB_X + HUB_SIZE/2 + 0.01, CENTER_Y, 1.2, 0},           // Blue HUB front center
            {2, BLUE_HUB_X + HUB_SIZE/2 + 0.01, CENTER_Y + 0.4, 1.0, 0},     // Blue HUB front left
            {3, BLUE_HUB_X + HUB_SIZE/2 + 0.01, CENTER_Y - 0.4, 1.0, 0},     // Blue HUB front right

            // RED HUB AprilTags (Tags 4-6) - facing toward field center (-X direction)
            {4, RED_HUB_X - HUB_SIZE/2 - 0.01, CENTER_Y, 1.2, 180},          // Red HUB front center
            {5, RED_HUB_X - HUB_SIZE/2 - 0.01, CENTER_Y + 0.4, 1.0, 180},    // Red HUB front left
            {6, RED_HUB_X - HUB_SIZE/2 - 0.01, CENTER_Y - 0.4, 1.0, 180},    // Red HUB front right

            // BLUE TOWER AprilTags (Tags 7-8) - on tower structure
            {7, BLUE_TOWER_X + 0.3, BLUE_TOWER_Y, 1.5, 0},                   // Blue Tower front
            {8, BLUE_TOWER_X, BLUE_TOWER_Y - 0.5, 1.5, 270},                 // Blue Tower side

            // RED TOWER AprilTags (Tags 9-10) - on tower structure
            {9, RED_TOWER_X - 0.3, RED_TOWER_Y, 1.5, 180},                   // Red Tower front
            {10, RED_TOWER_X, RED_TOWER_Y - 0.5, 1.5, 270},                  // Red Tower side

            // FIELD WALL AprilTags (Tags 11-16) - for general field localization
            {11, 0.01, WIDTH/4, 0.6, 0},                                      // Blue wall lower
            {12, 0.01, 3*WIDTH/4, 0.6, 0},                                    // Blue wall upper
            {13, LENGTH - 0.01, WIDTH/4, 0.6, 180},                           // Red wall lower
            {14, LENGTH - 0.01, 3*WIDTH/4, 0.6, 180},                         // Red wall upper
            {15, CENTER_X, 0.01, 0.6, 90},                                    // Bottom wall center
            {16, CENTER_X, WIDTH - 0.01, 0.6, 270}                            // Top wall center
        };
    }

    // ========================================================================
    // FUEL (Scoring Element)
    // ========================================================================
    public static final class Fuel {
        public static final double DIAMETER = 0.150;          // 5.91 inches
        public static final double RADIUS = DIAMETER / 2.0;
        public static final double MASS = 0.215;              // ~0.475 lbs in kg
        public static final int TOTAL_PER_MATCH = 504;        // Total FUEL in a match

        // Physics constants
        public static final double DRAG_COEFFICIENT = 0.47;   // Sphere
        public static final double RESTITUTION = 0.6;         // Bounce coefficient
        public static final double AIR_DENSITY = 1.225;       // kg/m³
        public static final double GRAVITY = 9.81;            // m/s²

        // Initial distribution
        public static final int PRELOAD_PER_ROBOT = 3;
        public static final int CHUTE_START_COUNT = 25;
    }

    // ========================================================================
    // ROBOT DIMENSIONS (meters)
    // ========================================================================
    public static final class Robot {
        public static final double LENGTH = 0.7556;           // 29.75 inches
        public static final double WIDTH = 0.7556;            // 29.75 inches
        public static final double BUMPER_THICKNESS = 0.0889; // 3.5 inches

        public static final double LENGTH_WITH_BUMPERS = LENGTH + 2 * BUMPER_THICKNESS;
        public static final double WIDTH_WITH_BUMPERS = WIDTH + 2 * BUMPER_THICKNESS;

        // Height constraint for TRENCH
        public static final double MAX_HEIGHT = 1.1;          // Typical max robot height
        public static final double TRENCH_CONFIG_HEIGHT = 0.5; // Height when configured for trench
    }

    // ========================================================================
    // SWERVE DRIVE
    // ========================================================================
    public static final class Swerve {
        public static final double TRACK_WIDTH = 0.4509;      // 17.75 inches
        public static final double WHEEL_BASE = 0.7556;       // 29.75 inches

        public static final double MAX_SPEED = 4.2;           // m/s
        public static final double MAX_ANGULAR_VELOCITY = 9.547; // rad/s

        // Acceleration limits for simulation
        public static final double MAX_ACCELERATION = 8.0;          // m/s^2
        public static final double MAX_ANGULAR_ACCELERATION = 20.0; // rad/s^2
    }

    // ========================================================================
    // SHOOTER (replaces Elevator/Arm for CORAL)
    // ========================================================================
    public static final class Shooter {
        public static final double MIN_ANGLE = 0.0;           // degrees (horizontal)
        public static final double MAX_ANGLE = 75.0;          // degrees (steep shot)

        public static final double MAX_VELOCITY = 20.0;       // m/s ball exit velocity
        public static final double MIN_VELOCITY = 5.0;        // m/s minimum shot

        public static final double ANGLE_RATE = 90.0;         // degrees/second
        public static final double SPINUP_TIME = 0.5;         // seconds to reach target velocity
    }

    // ========================================================================
    // INTAKE
    // ========================================================================
    public static final class Intake {
        public static final double WIDTH = 0.30;              // meters
        public static final int MAX_CAPACITY = 8;             // Maximum FUEL robot can hold
        public static final double INTAKE_TIME = 0.3;         // seconds per FUEL
        public static final double TRANSFER_TIME = 0.2;       // seconds to transfer to shooter
    }

    // ========================================================================
    // CLIMBER
    // ========================================================================
    public static final class Climber {
        public static final double MIN_POSITION = 0.0;
        public static final double MAX_POSITION = 1.70;       // m (to reach HIGH RUNG)

        public static final double MAX_VELOCITY = 0.5;        // m/s
        public static final double MAX_ACCELERATION = 2.0;    // m/s^2
    }

    // ========================================================================
    // SCORING
    // ========================================================================
    public static final class Scoring {
        // FUEL points
        public static final int FUEL_ACTIVE_HUB = 1;          // 1 point per FUEL in active HUB

        // TOWER climbing points (AUTO)
        public static final int TOWER_L1_AUTO = 15;
        // TOWER climbing points (TELEOP)
        public static final int TOWER_L1_TELEOP = 10;
        public static final int TOWER_L2_TELEOP = 20;
        public static final int TOWER_L3_TELEOP = 30;

        // Ranking point thresholds
        public static final int ENERGIZED_THRESHOLD = 100;    // 100 FUEL for RP
        public static final int SUPERCHARGED_THRESHOLD = 360; // 360 FUEL bonus
        public static final int TRAVERSAL_THRESHOLD = 50;     // 50 TOWER points for RP
    }

    // ========================================================================
    // MATCH TIMING
    // ========================================================================
    public static final class Match {
        public static final double TOTAL_DURATION = 160.0;    // 2:40 total

        public static final double AUTO_END = 20.0;           // 0-20s
        public static final double TRANSITION_END = 30.0;     // 20-30s (transition)
        public static final double SHIFT_1_END = 55.0;        // 30-55s
        public static final double SHIFT_2_END = 80.0;        // 55-80s
        public static final double SHIFT_3_END = 105.0;       // 80-105s
        public static final double SHIFT_4_END = 130.0;       // 105-130s
        public static final double END_GAME_START = 130.0;    // 130-160s end game

        public static final double SHIFT_DURATION = 25.0;     // Duration of each shift period
    }

    // ========================================================================
    // SIMULATION
    // ========================================================================
    public static final class Simulation {
        public static final double TICK_RATE = 50.0;          // Hz (20ms per tick)
        public static final double DT = 1.0 / TICK_RATE;      // seconds per tick
        public static final int BROADCAST_RATE = 30;          // Hz for WebSocket updates
    }
}
