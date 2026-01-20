package team3164.simulator.engine;

import team3164.simulator.Constants;
import team3164.simulator.physics.CollisionPhysics;
import team3164.simulator.physics.CollisionPrediction;
import team3164.simulator.physics.TrenchPhysics;

import java.util.Random;

/**
 * AI controller for non-player robots in the simulation.
 * Each AI robot randomly selects an autonomous mode and executes it.
 * During teleop, AI robots perform simple behaviors based on their role.
 */
public class AIRobotController {

    private static final Random random = new Random();

    // AI behavior modes
    public enum AIBehavior {
        AGGRESSIVE,    // Seeks FUEL and scores
        DEFENSIVE,     // Blocks opponents, plays defense
        CLIMBER,       // Prioritizes climbing
        COLLECTOR      // Focuses on collecting FUEL
    }

    private final int robotId;
    private final MatchState.Alliance alliance;
    private final int teamNumber;

    private int selectedAutoMode;
    private AIBehavior teleopBehavior;
    private AutonomousController autoController;

    // AI state
    private double targetX;
    private double targetY;
    private double actionTimer;
    private boolean hasTarget;
    private AIPhase currentPhase;

    public enum AIPhase {
        IDLE,
        MOVING_TO_TARGET,
        SHOOTING,
        INTAKING,
        CLIMBING,
        DEFENDING
    }

    /**
     * Create a new AI robot controller.
     *
     * @param robotId Unique robot ID (0-5)
     * @param alliance Robot's alliance
     * @param teamNumber Team number for display
     */
    public AIRobotController(int robotId, MatchState.Alliance alliance, int teamNumber) {
        this.robotId = robotId;
        this.alliance = alliance;
        this.teamNumber = teamNumber;
        this.autoController = new AutonomousController();

        // Randomly select auto mode from all 10 modes
        this.selectedAutoMode = random.nextInt(AutonomousController.NUM_AUTO_MODES);
        autoController.setSelectedMode(selectedAutoMode);

        // Randomly select teleop behavior
        AIBehavior[] behaviors = AIBehavior.values();
        this.teleopBehavior = behaviors[random.nextInt(behaviors.length)];

        this.currentPhase = AIPhase.IDLE;
        this.hasTarget = false;
    }

    /**
     * Get the selected auto mode.
     */
    public int getSelectedAutoMode() {
        return selectedAutoMode;
    }

    /**
     * Set the autonomous mode (0-9).
     * Supports 10 auto modes via 4-bit DIP switch.
     *
     * @param mode The auto mode index
     */
    public void setAutoMode(int mode) {
        this.selectedAutoMode = Math.max(0, Math.min(AutonomousController.NUM_AUTO_MODES - 1, mode));
        autoController.setSelectedMode(this.selectedAutoMode);
    }

    /**
     * Get the auto mode name.
     */
    public String getAutoModeName() {
        return autoController.getSelectedModeName();
    }

    /**
     * Get the teleop behavior.
     */
    public AIBehavior getTeleopBehavior() {
        return teleopBehavior;
    }

    /**
     * Get the team number.
     */
    public int getTeamNumber() {
        return teamNumber;
    }

    /**
     * Start autonomous period.
     */
    public void startAuto(RobotState state) {
        autoController.startAuto(state);
    }

    /**
     * Reset the controller.
     */
    public void reset() {
        autoController.reset();
        currentPhase = AIPhase.IDLE;
        hasTarget = false;
        actionTimer = 0;

        // Re-randomize for next match (all 10 modes)
        selectedAutoMode = random.nextInt(AutonomousController.NUM_AUTO_MODES);
        autoController.setSelectedMode(selectedAutoMode);

        AIBehavior[] behaviors = AIBehavior.values();
        teleopBehavior = behaviors[random.nextInt(behaviors.length)];
    }

    /**
     * Update the AI controller.
     *
     * @param state This robot's state
     * @param input Input to modify
     * @param matchState Current match state
     * @param allRobots All robots for collision avoidance
     * @param dt Time delta
     */
    public void update(RobotState state, InputState input, MatchState matchState,
                       RobotState[] allRobots, double dt) {
        actionTimer += dt;

        // Clear inputs
        clearInputs(input);

        // AI robots use robot-relative control (we calculate the transform ourselves)
        state.fieldRelative = false;

        // Manage trench mode - enable when approaching trenches, disable when clear
        manageTrenchMode(state);

        if (!matchState.matchStarted || matchState.matchEnded) {
            return;
        }

        switch (matchState.currentPhase) {
            case AUTO:
                updateAuto(state, input, dt);
                break;
            case TRANSITION:
                // Wait during transition - robots hold position
                break;
            case SHIFT_1:
            case SHIFT_2:
            case SHIFT_3:
            case SHIFT_4:
            case END_GAME:
                updateTeleop(state, input, matchState, allRobots, dt);
                break;
            default:
                // PRE_MATCH, POST_MATCH - do nothing
                break;
        }
    }

    /**
     * Update during autonomous period.
     */
    private void updateAuto(RobotState state, InputState input, double dt) {
        autoController.update(state, input, dt);
    }

    /**
     * Update during teleop period.
     */
    private void updateTeleop(RobotState state, InputState input, MatchState matchState,
                              RobotState[] allRobots, double dt) {
        switch (teleopBehavior) {
            case AGGRESSIVE:
                updateAggressive(state, input, matchState, allRobots, dt);
                break;
            case DEFENSIVE:
                updateDefensive(state, input, matchState, allRobots, dt);
                break;
            case CLIMBER:
                updateClimber(state, input, matchState, allRobots, dt);
                break;
            case COLLECTOR:
                updateCollector(state, input, matchState, allRobots, dt);
                break;
        }
    }

    /**
     * Aggressive behavior - seek FUEL and score.
     */
    private void updateAggressive(RobotState state, InputState input, MatchState matchState,
                                  RobotState[] allRobots, double dt) {
        // If we have FUEL, shoot it
        if (state.fuelCount > 0) {
            currentPhase = AIPhase.SHOOTING;
            // Drive toward shooting position - MUST be in alliance zone per G407
            double hubX = alliance == MatchState.Alliance.RED ?
                    Constants.Field.RED_HUB_X : Constants.Field.BLUE_HUB_X;
            double hubY = Constants.Field.RED_HUB_Y;  // Center Y
            // G407: Robot must shoot from inside alliance zone
            // BLUE zone: x = 0 to 4.03m, RED zone: x = 12.51m to 16.54m
            // Position deep enough that 1.0m tolerance still keeps us in zone
            double shootPosX = alliance == MatchState.Alliance.RED ?
                    Constants.Field.LENGTH - 2.0 : 2.0;
            // Offset Y based on robot ID to spread robots out and avoid HUB collision
            // HUB is at center Y (~4.0), so offset by 2.5-3.5m to go around it
            double yOffset = ((state.robotId % 3) - 1) * 3.0;  // -3, 0, or +3 meters
            double shootPosY = hubY + yOffset;
            // Clamp Y to stay on field
            shootPosY = Math.max(1.5, Math.min(Constants.Field.WIDTH - 1.5, shootPosY));

            boolean atPosition = driveToTarget(state, input, shootPosX, shootPosY, 1.0, allRobots);

            // Calculate optimal shot parameters based on distance
            double distToHub = Math.hypot(hubX - state.x, hubY - state.y);
            // Lower angle for closer shots, higher for farther shots
            double optimalAngle = Math.min(0.7, Math.max(0.2, (distToHub - 2.0) / 10.0));
            double optimalPower = Math.min(0.9, Math.max(0.5, distToHub / 10.0));
            input.shooterAngle = optimalAngle;
            input.shooterPower = optimalPower;

            if (atPosition) {
                // At shooting position - turn to face the hub
                double angleToHub = Math.atan2(hubY - state.y, hubX - state.x);
                double headingError = normalizeAngle(angleToHub - state.heading);

                if (Math.abs(headingError) > 0.1) {
                    // Need to turn to face hub
                    input.turn = Math.signum(headingError) * Math.min(0.6, Math.abs(headingError));
                    state.currentCommand = "Aiming";
                } else if (state.shooterAtSpeed && state.shooterAtAngle) {
                    // Facing hub and shooter ready - fire!
                    input.shoot = true;
                    state.currentCommand = "Shooting";
                } else {
                    // Wait for shooter to spin up
                    state.currentCommand = "Spinning Up";
                }
            } else {
                state.currentCommand = "Approaching Hub";
            }
        } else {
            // Go collect FUEL
            currentPhase = AIPhase.INTAKING;
            if (!hasTarget || actionTimer > 3.0) {
                pickRandomNeutralTarget();
                actionTimer = 0;
            }

            input.intake = true;
            driveToTarget(state, input, targetX, targetY, 0.5, allRobots);
            state.currentCommand = "Collecting FUEL";
        }
    }

    /**
     * Defensive behavior - block opponents and opportunistically score.
     */
    private void updateDefensive(RobotState state, InputState input, MatchState matchState,
                                 RobotState[] allRobots, double dt) {
        // If we have FUEL, score it first (opportunity scoring)
        if (state.fuelCount >= 3) {
            currentPhase = AIPhase.SHOOTING;
            double hubX = alliance == MatchState.Alliance.RED ?
                    Constants.Field.RED_HUB_X : Constants.Field.BLUE_HUB_X;
            double hubY = Constants.Field.RED_HUB_Y;
            // G407: Robot must shoot from inside alliance zone
            double shootPosX = alliance == MatchState.Alliance.RED ?
                    Constants.Field.LENGTH - 2.0 : 2.0;
            double yOffset = ((state.robotId % 3) - 1) * 3.0;
            double shootPosY = hubY + yOffset;
            shootPosY = Math.max(1.5, Math.min(Constants.Field.WIDTH - 1.5, shootPosY));

            boolean atPosition = driveToTarget(state, input, shootPosX, shootPosY, 1.0, allRobots);
            input.shooterAngle = 0.55;
            input.shooterPower = 0.75;

            if (atPosition) {
                // Turn to face hub
                double angleToHub = Math.atan2(hubY - state.y, hubX - state.x);
                double headingError = normalizeAngle(angleToHub - state.heading);
                if (Math.abs(headingError) > 0.1) {
                    input.turn = Math.signum(headingError) * Math.min(0.6, Math.abs(headingError));
                    state.currentCommand = "Aiming";
                } else if (state.shooterAtSpeed && state.shooterAtAngle) {
                    input.shoot = true;
                    state.currentCommand = "Shooting";
                }
            }
            return;
        }

        currentPhase = AIPhase.DEFENDING;

        // Find nearest opponent with FUEL (prioritize blocking scorers)
        RobotState targetOpponent = null;
        double bestScore = -1;

        for (RobotState other : allRobots) {
            if (other == state || other.alliance == alliance) continue;

            double dist = Math.hypot(other.x - state.x, other.y - state.y);
            // Score based on: closer + has more fuel = better target
            double score = other.fuelCount * 10.0 - dist;
            if (score > bestScore) {
                bestScore = score;
                targetOpponent = other;
            }
        }

        if (targetOpponent != null) {
            // Position between opponent and their hub
            double hubX = targetOpponent.alliance == MatchState.Alliance.RED ?
                    Constants.Field.RED_HUB_X : Constants.Field.BLUE_HUB_X;
            double hubY = Constants.Field.CENTER_Y;

            // Intercept position (closer to hub to block shots)
            double interceptX = targetOpponent.x * 0.3 + hubX * 0.7;
            double interceptY = targetOpponent.y * 0.3 + hubY * 0.7;

            driveToTarget(state, input, interceptX, interceptY, 1.5, allRobots);
            state.currentCommand = "Defending";

            // Collect FUEL opportunistically while defending
            input.intake = true;
        } else {
            // Patrol neutral zone and collect FUEL
            if (!hasTarget || actionTimer > 3.0) {
                pickRandomNeutralTarget();
                actionTimer = 0;
            }
            input.intake = true;
            driveToTarget(state, input, targetX, targetY, 0.5, allRobots);
            state.currentCommand = "Patrolling";
        }
    }

    /**
     * Climber behavior - prioritize climbing in end game.
     */
    private void updateClimber(RobotState state, InputState input, MatchState matchState,
                               RobotState[] allRobots, double dt) {
        double timeRemaining = matchState.getRemainingTime();

        // Start climbing when 35 seconds left (need time to climb)
        if (timeRemaining < 35 && !state.climbComplete) {
            currentPhase = AIPhase.CLIMBING;

            // Drive to tower
            double towerX = alliance == MatchState.Alliance.RED ?
                    Constants.Field.RED_TOWER_X - 1.0 : Constants.Field.BLUE_TOWER_X + 1.0;
            double towerY = alliance == MatchState.Alliance.RED ?
                    Constants.Field.RED_TOWER_Y : Constants.Field.BLUE_TOWER_Y;

            if (driveToTarget(state, input, towerX, towerY, 0.5, allRobots)) {
                // At tower, climb to L2 (20 points)
                input.level2 = true;
                input.climberUp = true;
                state.currentCommand = "Climbing L2";
            } else {
                state.currentCommand = "Moving to Tower";
            }
        } else if (state.fuelCount > 0) {
            // Score any FUEL before climbing
            currentPhase = AIPhase.SHOOTING;
            double hubX = alliance == MatchState.Alliance.RED ?
                    Constants.Field.RED_HUB_X : Constants.Field.BLUE_HUB_X;
            double hubY = Constants.Field.RED_HUB_Y;
            // G407: Robot must shoot from inside alliance zone
            double shootPosX = alliance == MatchState.Alliance.RED ?
                    Constants.Field.LENGTH - 2.0 : 2.0;
            double yOffset = ((state.robotId % 3) - 1) * 3.0;
            double shootPosY = hubY + yOffset;
            shootPosY = Math.max(1.5, Math.min(Constants.Field.WIDTH - 1.5, shootPosY));

            boolean atPosition = driveToTarget(state, input, shootPosX, shootPosY, 1.0, allRobots);
            input.shooterAngle = 0.55;
            input.shooterPower = 0.75;

            if (atPosition) {
                double angleToHub = Math.atan2(hubY - state.y, hubX - state.x);
                double headingError = normalizeAngle(angleToHub - state.heading);
                if (Math.abs(headingError) > 0.1) {
                    input.turn = Math.signum(headingError) * Math.min(0.6, Math.abs(headingError));
                    state.currentCommand = "Aiming";
                } else if (state.shooterAtSpeed && state.shooterAtAngle) {
                    input.shoot = true;
                    state.currentCommand = "Shooting";
                }
            }
        } else {
            // Collect FUEL until closer to end game
            currentPhase = AIPhase.INTAKING;
            if (!hasTarget || actionTimer > 3.0) {
                pickRandomNeutralTarget();
                actionTimer = 0;
            }
            input.intake = true;
            driveToTarget(state, input, targetX, targetY, 0.5, allRobots);
        }
    }

    /**
     * Collector behavior - focus on gathering FUEL.
     */
    private void updateCollector(RobotState state, InputState input, MatchState matchState,
                                 RobotState[] allRobots, double dt) {
        if (state.fuelCount >= Constants.Intake.MAX_CAPACITY - 2) {
            // Nearly full, go score
            currentPhase = AIPhase.SHOOTING;
            double hubX = alliance == MatchState.Alliance.RED ?
                    Constants.Field.RED_HUB_X : Constants.Field.BLUE_HUB_X;
            double hubY = Constants.Field.RED_HUB_Y;
            // G407: Robot must shoot from inside alliance zone
            double shootPosX = alliance == MatchState.Alliance.RED ?
                    Constants.Field.LENGTH - 2.0 : 2.0;
            double yOffset = ((state.robotId % 3) - 1) * 3.0;
            double shootPosY = hubY + yOffset;
            shootPosY = Math.max(1.5, Math.min(Constants.Field.WIDTH - 1.5, shootPosY));

            boolean atPosition = driveToTarget(state, input, shootPosX, shootPosY, 1.0, allRobots);

            // Spin up shooter while approaching
            input.shooterAngle = 0.55;
            input.shooterPower = 0.75;

            if (atPosition) {
                // Turn to face hub (aim at hub center, not offset position)
                double angleToHub = Math.atan2(hubY - state.y, hubX - state.x);
                double headingError = normalizeAngle(angleToHub - state.heading);
                if (Math.abs(headingError) > 0.1) {
                    input.turn = Math.signum(headingError) * Math.min(0.6, Math.abs(headingError));
                    state.currentCommand = "Aiming";
                } else if (state.shooterAtSpeed && state.shooterAtAngle) {
                    input.shoot = true;
                    state.currentCommand = "Dumping FUEL";
                } else {
                    state.currentCommand = "Spinning Up";
                }
            } else {
                state.currentCommand = "Full - Scoring";
            }
        } else {
            // Collect FUEL
            currentPhase = AIPhase.INTAKING;
            if (!hasTarget || actionTimer > 4.0) {
                pickRandomFieldTarget();
                actionTimer = 0;
            }

            input.intake = true;
            driveToTarget(state, input, targetX, targetY, 0.5, allRobots);
            state.currentCommand = "Collecting (" + state.fuelCount + "/" + Constants.Intake.MAX_CAPACITY + ")";
        }
    }

    /**
     * Pick a random target in the neutral zone.
     */
    private void pickRandomNeutralTarget() {
        targetX = Constants.Field.CENTER_X + (random.nextDouble() - 0.5) * 6.0;
        targetY = Constants.Field.CENTER_Y + (random.nextDouble() - 0.5) * 4.0;
        hasTarget = true;
    }

    /**
     * Pick a random target anywhere on field (alliance side preferred).
     */
    private void pickRandomFieldTarget() {
        if (alliance == MatchState.Alliance.RED) {
            targetX = Constants.Field.CENTER_X + random.nextDouble() * 4.0;
        } else {
            targetX = Constants.Field.CENTER_X - random.nextDouble() * 4.0;
        }
        targetY = 1.0 + random.nextDouble() * (Constants.Field.WIDTH - 2.0);
        hasTarget = true;
    }

    /**
     * Drive toward a target position with collision avoidance.
     *
     * @param state This robot's state
     * @param input Input to modify
     * @param tgtX Target X position
     * @param tgtY Target Y position
     * @param tolerance Distance at which we consider ourselves "at target"
     * @param allRobots All robots for collision avoidance (can be null)
     * @return true if at target
     */
    private boolean driveToTarget(RobotState state, InputState input,
                                  double tgtX, double tgtY, double tolerance,
                                  RobotState[] allRobots) {
        double dx = tgtX - state.x;
        double dy = tgtY - state.y;
        double distance = Math.hypot(dx, dy);

        if (distance < tolerance) {
            return true;
        }

        // Check if path is blocked and adjust
        if (CollisionPhysics.wouldCollide(tgtX, tgtY)) {
            // Target is inside obstacle, adjust
            tgtX += (random.nextDouble() - 0.5) * 2.0;
            tgtY += (random.nextDouble() - 0.5) * 2.0;
            dx = tgtX - state.x;
            dy = tgtY - state.y;
            distance = Math.hypot(dx, dy);
        }

        // Calculate direction
        double targetAngle = Math.atan2(dy, dx);
        double speed = Math.min(0.7, distance * 0.3);

        // Field-relative control - but only if not in field-relative mode!
        // AI robots should use robot-relative control since they calculate the transform themselves
        double cos = Math.cos(state.heading);
        double sin = Math.sin(state.heading);

        // Transform field-relative target velocity to robot-relative
        double fieldVx = speed * Math.cos(targetAngle);
        double fieldVy = speed * Math.sin(targetAngle);

        // ========== Collision Avoidance ==========
        if (allRobots != null) {
            CollisionPrediction.AvoidanceVector avoidance =
                    CollisionPrediction.calculateCombinedAvoidance(state, allRobots);

            if (avoidance.needsAvoidance) {
                // Blend avoidance with target direction
                double avoidInfluence = avoidance.urgency *
                        Constants.CollisionAvoidance.MAX_AVOIDANCE_BLEND;
                double targetInfluence = 1.0 - avoidInfluence;

                fieldVx = fieldVx * targetInfluence +
                          avoidance.steerX * Constants.Swerve.MAX_SPEED * avoidInfluence;
                fieldVy = fieldVy * targetInfluence +
                          avoidance.steerY * Constants.Swerve.MAX_SPEED * avoidInfluence;

                // Reduce speed during avoidance
                double speedScale = 1.0 - (avoidance.urgency *
                        Constants.CollisionAvoidance.MAX_SPEED_REDUCTION);
                fieldVx *= speedScale;
                fieldVy *= speedScale;
            }
        }

        // Robot-relative velocities
        input.forward = fieldVx * cos + fieldVy * sin;
        input.strafe = -fieldVx * sin + fieldVy * cos;

        // Turn towards movement direction
        double headingError = normalizeAngle(targetAngle - state.heading);
        if (Math.abs(headingError) > 0.2) {
            input.turn = Math.signum(headingError) * Math.min(0.4, Math.abs(headingError) * 0.5);
        }

        return false;
    }

    private void clearInputs(InputState input) {
        input.forward = 0;
        input.strafe = 0;
        input.turn = 0;
        input.intake = false;
        input.shoot = false;
        input.spinUp = false;
        input.climberUp = false;
        input.climberDown = false;
        input.level1 = false;
        input.level2 = false;
        input.level3 = false;
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    /**
     * Manage trench mode for AI robots.
     * Automatically enables trench mode when approaching a trench,
     * and disables it when clear of trenches.
     */
    private void manageTrenchMode(RobotState state) {
        // Get nearest trench info
        double[] nearestTrench = TrenchPhysics.getNearestTrench(state.x, state.y);
        double trenchDist = nearestTrench[2];

        // Check if we're in or near a trench
        TrenchPhysics.TrenchInfo trenchInfo = TrenchPhysics.getTrenchInfo(state.x, state.y);

        if (trenchInfo.inTrench) {
            // We're inside a trench - must be in trench mode to pass
            if (!state.trenchMode) {
                enableTrenchMode(state);
            }
        } else if (trenchDist < 2.0) {
            // Approaching a trench - check if we're heading toward it
            if (TrenchPhysics.isApproachingTrench(state.x, state.y, state.vx, state.vy)) {
                if (!state.trenchMode) {
                    enableTrenchMode(state);
                }
            }
        } else if (trenchDist > 3.0 && state.trenchMode) {
            // Far from any trench, safe to exit trench mode
            disableTrenchMode(state);
        }
    }

    /**
     * Enable trench mode - lower robot height to fit under trenches.
     */
    private void enableTrenchMode(RobotState state) {
        state.trenchMode = true;
        state.robotHeight = Constants.Robot.TRENCH_CONFIG_HEIGHT;
    }

    /**
     * Disable trench mode - return to normal height.
     */
    private void disableTrenchMode(RobotState state) {
        state.trenchMode = false;
        state.robotHeight = Constants.Robot.MAX_HEIGHT;
    }
}
