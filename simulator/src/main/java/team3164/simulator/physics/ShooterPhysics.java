package team3164.simulator.physics;

import team3164.simulator.Constants;
import team3164.simulator.engine.FuelState;
import team3164.simulator.engine.FuelState.Fuel;
import team3164.simulator.engine.InputState;
import team3164.simulator.engine.MatchState;
import team3164.simulator.engine.MatchState.Alliance;
import team3164.simulator.engine.RobotState;
import team3164.simulator.engine.RobotState.IntakeState;

/**
 * Physics for shooter mechanism in REBUILT 2026.
 * Handles angle adjustment, velocity control, and FUEL launching.
 */
public class ShooterPhysics {

    // Shooter physical height on robot
    private static final double SHOOTER_HEIGHT = 1.0;  // meters above ground

    // Spinup parameters
    private static final double VELOCITY_RAMP_RATE = 20.0;  // m/s per second

    /**
     * Update shooter state based on inputs.
     *
     * @param state Robot state
     * @param input Input state
     * @param fuelState FUEL tracking state
     * @param matchState Match state
     * @param dt Time step
     * @return The launched FUEL if a shot was fired, null otherwise
     */
    public static Fuel update(RobotState state, InputState input, FuelState fuelState,
                             MatchState matchState, double dt) {
        Fuel launchedFuel = null;

        // Update shooter angle
        updateAngle(state, input, dt);

        // Update shooter velocity
        updateVelocity(state, input, dt);

        // Handle shooting - G407: Robot must be in alliance zone to score
        if (input.shoot && state.isReadyToShoot() && isInAllianceZone(state)) {
            launchedFuel = fireFuel(state, fuelState, matchState);
        }

        // Update intake/transfer state
        updateIntakeState(state, input, dt);

        return launchedFuel;
    }

    /**
     * Update shooter angle based on input.
     */
    private static void updateAngle(RobotState state, InputState input, double dt) {
        // Calculate target angle from input (0-1 maps to min-max angle)
        double targetAngle = Constants.Shooter.MIN_ANGLE +
            input.shooterAngle * (Constants.Shooter.MAX_ANGLE - Constants.Shooter.MIN_ANGLE);

        state.shooterAngleGoal = targetAngle;

        // Move toward target angle
        double angleDiff = targetAngle - state.shooterAngle;
        double maxChange = Constants.Shooter.ANGLE_RATE * dt;

        if (Math.abs(angleDiff) <= maxChange) {
            state.shooterAngle = targetAngle;
            state.shooterAtAngle = true;
        } else {
            state.shooterAngle += Math.signum(angleDiff) * maxChange;
            state.shooterAtAngle = false;
        }

        // Clamp to valid range
        state.shooterAngle = Math.max(Constants.Shooter.MIN_ANGLE,
                                       Math.min(Constants.Shooter.MAX_ANGLE, state.shooterAngle));
    }

    /**
     * Update shooter wheel velocity based on input.
     */
    private static void updateVelocity(RobotState state, InputState input, double dt) {
        // Calculate target velocity from input (0-1 maps to min-max velocity)
        double targetVelocity = Constants.Shooter.MIN_VELOCITY +
            input.shooterPower * (Constants.Shooter.MAX_VELOCITY - Constants.Shooter.MIN_VELOCITY);

        state.shooterVelocityGoal = targetVelocity;

        // Ramp velocity
        double velDiff = targetVelocity - state.shooterVelocity;
        double maxChange = VELOCITY_RAMP_RATE * dt;

        if (Math.abs(velDiff) <= maxChange) {
            state.shooterVelocity = targetVelocity;
        } else {
            state.shooterVelocity += Math.signum(velDiff) * maxChange;
        }

        // Check if at target
        state.shooterAtSpeed = Math.abs(state.shooterVelocity - state.shooterVelocityGoal) < 0.5;
        state.shooterSpinningUp = state.shooterVelocity < state.shooterVelocityGoal - 0.5;
    }

    /**
     * Update intake state machine.
     */
    private static void updateIntakeState(RobotState state, InputState input, double dt) {
        switch (state.intakeState) {
            case IDLE:
                if (input.intake && state.canIntakeFuel()) {
                    state.intakeState = IntakeState.INTAKING;
                    state.intakeTimer = Constants.Intake.INTAKE_TIME;
                    state.currentCommand = "Intaking";
                }
                break;

            case INTAKING:
                state.intakeTimer -= dt;
                if (state.intakeTimer <= 0) {
                    // Intake complete - FUEL added externally by FuelPhysics
                    state.intakeState = IntakeState.TRANSFERRING;
                    state.intakeTimer = Constants.Intake.TRANSFER_TIME;
                    state.currentCommand = "Transferring";
                } else if (!input.intake) {
                    // Cancelled
                    state.intakeState = IntakeState.IDLE;
                    state.currentCommand = "";
                }
                break;

            case TRANSFERRING:
                state.intakeTimer -= dt;
                if (state.intakeTimer <= 0) {
                    state.intakeState = IntakeState.READY_TO_SHOOT;
                    state.currentCommand = "Ready";
                }
                break;

            case READY_TO_SHOOT:
                // Waiting for shoot command - must also be in alliance zone per G407
                if (input.shoot && state.isReadyToShoot() && isInAllianceZone(state)) {
                    state.intakeState = IntakeState.SHOOTING;
                    state.intakeTimer = 0.1;  // Brief shooting animation
                    state.currentCommand = "Shooting";
                }
                break;

            case SHOOTING:
                state.intakeTimer -= dt;
                if (state.intakeTimer <= 0) {
                    // Shot complete
                    if (state.fuelCount > 0) {
                        state.intakeState = IntakeState.TRANSFERRING;
                        state.intakeTimer = Constants.Intake.TRANSFER_TIME;
                        state.currentCommand = "Transferring";
                    } else {
                        state.intakeState = IntakeState.IDLE;
                        state.currentCommand = "Empty";
                    }
                }
                break;
        }
    }

    /**
     * Fire a FUEL ball.
     *
     * @return The launched FUEL
     */
    private static Fuel fireFuel(RobotState state, FuelState fuelState, MatchState matchState) {
        if (!state.removeFuel()) {
            return null;  // No FUEL to fire
        }

        // Calculate launch parameters
        double angleRad = Math.toRadians(state.shooterAngle);
        double velocity = state.shooterVelocity;
        double heading = state.heading;

        // Calculate start position (at shooter output)
        double startX = state.x + Math.cos(heading) * 0.3;
        double startY = state.y + Math.sin(heading) * 0.3;
        double startZ = SHOOTER_HEIGHT;

        // Calculate velocity components
        double[] launchVel = FuelPhysics.calculateLaunchVelocity(
            startX, startY, startZ, angleRad, velocity, heading
        );

        // Create and launch the FUEL
        Fuel fuel = fuelState.launchFuel(startX, startY, startZ,
                                         launchVel[0], launchVel[1], launchVel[2]);

        state.currentCommand = "Shot Fired!";
        return fuel;
    }

    /**
     * Get estimated shot trajectory for aiming.
     *
     * @param state Robot state
     * @return Array of trajectory points [[x,y,z], ...]
     */
    public static double[][] getTrajectoryPreview(RobotState state) {
        if (state.shooterVelocity < 1.0) {
            return new double[0][];  // Not shooting
        }

        double angleRad = Math.toRadians(state.shooterAngle);
        double velocity = state.shooterVelocity;
        double heading = state.heading;

        double startX = state.x + Math.cos(heading) * 0.3;
        double startY = state.y + Math.sin(heading) * 0.3;
        double startZ = SHOOTER_HEIGHT;

        double[] launchVel = FuelPhysics.calculateLaunchVelocity(
            startX, startY, startZ, angleRad, velocity, heading
        );

        // Generate trajectory points
        int numPoints = 20;
        double[][] points = new double[numPoints][3];
        double dt = 0.1;

        double x = startX, y = startY, z = startZ;
        double vx = launchVel[0], vy = launchVel[1], vz = launchVel[2];

        for (int i = 0; i < numPoints; i++) {
            points[i] = new double[]{x, y, z};

            // Simple euler integration (no drag for preview)
            vz -= Constants.Fuel.GRAVITY * dt;
            x += vx * dt;
            y += vy * dt;
            z += vz * dt;

            if (z < 0) break;  // Hit ground
        }

        return points;
    }

    /**
     * Check if current shot will hit the HUB.
     */
    public static boolean willShotHitHub(RobotState state) {
        if (state.shooterVelocity < Constants.Shooter.MIN_VELOCITY) {
            return false;
        }

        double angleRad = Math.toRadians(state.shooterAngle);
        double velocity = state.shooterVelocity;
        double heading = state.heading;

        double startX = state.x + Math.cos(heading) * 0.3;
        double startY = state.y + Math.sin(heading) * 0.3;
        double startZ = SHOOTER_HEIGHT;

        double[] launchVel = FuelPhysics.calculateLaunchVelocity(
            startX, startY, startZ, angleRad, velocity, heading
        );

        // Check against alliance HUB
        return HubPhysics.willHitHub(startX, startY, startZ,
                                     launchVel[0], launchVel[1], launchVel[2],
                                     state.alliance);
    }

    /**
     * Set shooter to optimal angle and velocity for current position.
     */
    public static void setOptimalShot(RobotState state) {
        double[] optimal = HubPhysics.calculateOptimalShot(state, state.alliance);
        if (optimal != null) {
            state.shooterAngleGoal = optimal[0];
            state.shooterVelocityGoal = optimal[1];
        }
    }

    /**
     * Check if robot is in its alliance zone (G407 requirement).
     * Robot BUMPERS must be partially or fully within their ALLIANCE ZONE to score.
     *
     * @param state Robot state
     * @return true if robot can legally shoot
     */
    public static boolean isInAllianceZone(RobotState state) {
        double halfRobot = Constants.Robot.LENGTH_WITH_BUMPERS / 2.0;
        double allianceZoneDepth = Constants.Field.ALLIANCE_ZONE_DEPTH;

        if (state.alliance == Alliance.BLUE) {
            // Blue alliance zone: x = 0 to ALLIANCE_ZONE_DEPTH
            // Robot BUMPERS partially in zone means robot center x < zone + halfRobot
            return state.x - halfRobot < allianceZoneDepth;
        } else {
            // Red alliance zone: x = (LENGTH - ALLIANCE_ZONE_DEPTH) to LENGTH
            // Robot BUMPERS partially in zone means robot center x > (LENGTH - zone - halfRobot)
            double redZoneStart = Constants.Field.LENGTH - allianceZoneDepth;
            return state.x + halfRobot > redZoneStart;
        }
    }
}
