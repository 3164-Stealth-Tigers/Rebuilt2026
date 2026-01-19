package team3164.simulator.engine;

import team3164.simulator.Constants;

/**
 * Complete state of the simulated robot for REBUILT 2026.
 * This is the single source of truth for all robot data.
 */
public class RobotState {

    // ========================================================================
    // ROBOT IDENTITY
    // ========================================================================
    public int robotId = 0;           // Unique robot ID (0-5)
    public int teamNumber = 3164;     // Team number for display
    public boolean isPlayerControlled = true;  // Player vs AI controlled

    // Robot alliance (use MatchState.Alliance)
    public MatchState.Alliance alliance = MatchState.Alliance.BLUE;

    // ========================================================================
    // POSE (Position on field)
    // ========================================================================
    public double x;         // meters from origin (blue alliance wall)
    public double y;         // meters from origin (bottom of field)
    public double heading;   // radians, 0 = facing red alliance, CCW positive

    // ========================================================================
    // VELOCITY
    // ========================================================================
    public double vx;        // m/s in field X direction
    public double vy;        // m/s in field Y direction
    public double omega;     // rad/s angular velocity

    // ========================================================================
    // SWERVE MODULES (FL, FR, RL, RR)
    // ========================================================================
    public double[] moduleAngles = new double[4];    // radians
    public double[] moduleSpeeds = new double[4];    // m/s

    // ========================================================================
    // SHOOTER (replaces Elevator/Arm)
    // ========================================================================
    public double shooterAngle;         // degrees (0 = horizontal, 75 = max)
    public double shooterAngleGoal;     // target angle in degrees
    public double shooterVelocity;      // current wheel velocity (m/s)
    public double shooterVelocityGoal;  // target ball exit velocity (m/s)
    public boolean shooterAtAngle;      // angle reached goal
    public boolean shooterAtSpeed;      // velocity reached goal
    public boolean shooterSpinningUp;   // shooter wheels spinning up

    // ========================================================================
    // INTAKE
    // ========================================================================
    public IntakeState intakeState = IntakeState.IDLE;
    public int fuelCount;               // Number of FUEL in robot
    public double intakeTimer;          // timer for intake animation

    // ========================================================================
    // CLIMBER
    // ========================================================================
    public double climberPosition;      // meters (height)
    public double climberVelocity;      // m/s
    public int climbLevel;              // 0 = not climbing, 1-3 = rung level
    public boolean isClimbing;          // currently attached to tower
    public boolean climbComplete;       // finished climbing

    // ========================================================================
    // ROBOT CONFIGURATION
    // ========================================================================
    public double robotHeight;          // current robot height (for trench)
    public boolean trenchMode;          // robot configured for trench
    public boolean onBump;              // robot currently on a bump

    // ========================================================================
    // CONTROL MODE
    // ========================================================================
    public boolean fieldRelative = true;
    public boolean slowMode = false;
    public String currentCommand = "";

    // ========================================================================
    // GAME STATE
    // ========================================================================
    public int fuelScored = 0;          // Total FUEL scored by this robot
    public int towerPoints = 0;         // Points from tower climbing
    public boolean isEnabled = true;

    public enum IntakeState {
        IDLE,           // Not doing anything
        INTAKING,       // Collecting FUEL from ground
        TRANSFERRING,   // Moving FUEL to shooter
        READY_TO_SHOOT, // FUEL loaded, ready to fire
        SHOOTING        // Currently shooting FUEL
    }

    /**
     * Initialize robot to starting position.
     */
    public RobotState() {
        reset();
    }

    /**
     * Reset to default starting state.
     */
    public void reset() {
        // Start near blue alliance wall
        x = 2.0;
        y = Constants.Field.CENTER_Y;
        heading = 0;

        vx = 0;
        vy = 0;
        omega = 0;

        for (int i = 0; i < 4; i++) {
            moduleAngles[i] = 0;
            moduleSpeeds[i] = 0;
        }

        // Shooter
        shooterAngle = 0;
        shooterAngleGoal = 0;
        shooterVelocity = 0;
        shooterVelocityGoal = 0;
        shooterAtAngle = true;
        shooterAtSpeed = false;
        shooterSpinningUp = false;

        // Intake
        intakeState = IntakeState.IDLE;
        fuelCount = Constants.Fuel.PRELOAD_PER_ROBOT;  // Start with preload
        intakeTimer = 0;

        // Climber
        climberPosition = 0;
        climberVelocity = 0;
        climbLevel = 0;
        isClimbing = false;
        climbComplete = false;

        // Configuration
        robotHeight = Constants.Robot.MAX_HEIGHT;
        trenchMode = false;
        onBump = false;

        // Control
        fieldRelative = true;
        slowMode = false;
        currentCommand = "";

        // Game state
        fuelScored = 0;
        towerPoints = 0;
        isEnabled = true;
    }

    /**
     * Check if robot can intake more FUEL.
     */
    public boolean canIntakeFuel() {
        return fuelCount < Constants.Intake.MAX_CAPACITY &&
               intakeState == IntakeState.IDLE;
    }

    /**
     * Check if robot has FUEL to shoot.
     */
    public boolean hasFuelToShoot() {
        return fuelCount > 0 &&
               (intakeState == IntakeState.READY_TO_SHOOT || intakeState == IntakeState.IDLE);
    }

    /**
     * Check if shooter is ready to fire.
     */
    public boolean isReadyToShoot() {
        return shooterAtAngle && shooterAtSpeed && hasFuelToShoot();
    }

    /**
     * Get shooter angle in radians.
     */
    public double getShooterAngleRadians() {
        return Math.toRadians(shooterAngle);
    }

    /**
     * Get heading in degrees for display.
     */
    public double getHeadingDegrees() {
        return Math.toDegrees(heading);
    }

    /**
     * Get speed magnitude for display.
     */
    public double getSpeed() {
        return Math.hypot(vx, vy);
    }

    /**
     * Check if robot can pass under trench.
     */
    public boolean canPassTrench() {
        return robotHeight <= Constants.Field.TRENCH_CLEARANCE || trenchMode;
    }

    /**
     * Set robot alliance.
     */
    public void setAlliance(MatchState.Alliance alliance) {
        this.alliance = alliance;
        // Reposition robot based on alliance
        if (alliance == MatchState.Alliance.RED) {
            x = Constants.Field.LENGTH - 2.0;
            heading = Math.PI;  // Facing blue alliance
        } else {
            x = 2.0;
            heading = 0;  // Facing red alliance
        }
        y = Constants.Field.CENTER_Y;
    }

    /**
     * Set starting position based on robot slot (0-2) within alliance.
     * Slot 0 = center, 1 = top, 2 = bottom
     */
    public void setStartingPosition(int slot) {
        double startX;
        double startY;
        double startHeading;

        // Y positions: spread out along alliance wall
        double[] yPositions = {
            Constants.Field.CENTER_Y,              // Center (slot 0)
            Constants.Field.CENTER_Y + 2.5,        // Top (slot 1)
            Constants.Field.CENTER_Y - 2.5         // Bottom (slot 2)
        };

        if (alliance == MatchState.Alliance.RED) {
            startX = Constants.Field.LENGTH - 2.0;
            startHeading = Math.PI;  // Facing blue
        } else {
            startX = 2.0;
            startHeading = 0;  // Facing red
        }

        startY = yPositions[Math.min(slot, 2)];

        this.x = startX;
        this.y = startY;
        this.heading = startHeading;
    }

    /**
     * Initialize robot with ID, team number, and alliance.
     */
    public void initialize(int robotId, int teamNumber, MatchState.Alliance alliance, int slot, boolean isPlayer) {
        this.robotId = robotId;
        this.teamNumber = teamNumber;
        this.alliance = alliance;
        this.isPlayerControlled = isPlayer;
        reset();
        setStartingPosition(slot);
    }

    /**
     * Add a FUEL to the robot's storage.
     *
     * @return true if FUEL was added, false if at capacity
     */
    public boolean addFuel() {
        if (fuelCount < Constants.Intake.MAX_CAPACITY) {
            fuelCount++;
            return true;
        }
        return false;
    }

    /**
     * Remove a FUEL from the robot (for shooting).
     *
     * @return true if FUEL was removed, false if empty
     */
    public boolean removeFuel() {
        if (fuelCount > 0) {
            fuelCount--;
            return true;
        }
        return false;
    }

    /**
     * Record scoring a FUEL.
     */
    public void recordFuelScored() {
        fuelScored++;
    }

    /**
     * Get climb level string for display.
     */
    public String getClimbLevelString() {
        if (!isClimbing && climbLevel == 0) return "Not Climbing";
        if (isClimbing && !climbComplete) return "Climbing L" + climbLevel;
        if (climbComplete) return "L" + climbLevel + " Complete";
        return "L" + climbLevel;
    }

    /**
     * Get the height the robot needs to reach for current climb level.
     */
    public double getTargetClimbHeight() {
        switch (climbLevel) {
            case 1: return Constants.Field.RUNG_LOW;
            case 2: return Constants.Field.RUNG_MID;
            case 3: return Constants.Field.RUNG_HIGH;
            default: return 0;
        }
    }
}
