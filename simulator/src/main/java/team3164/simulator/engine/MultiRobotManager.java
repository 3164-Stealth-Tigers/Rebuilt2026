package team3164.simulator.engine;

import team3164.simulator.Constants;
import team3164.simulator.physics.CollisionPhysics;

import java.util.Random;

/**
 * Manages all 6 robots in the simulation (3 per alliance).
 * One robot is player-controlled, the rest are AI-controlled.
 */
public class MultiRobotManager {

    public static final int ROBOTS_PER_ALLIANCE = 3;
    public static final int TOTAL_ROBOTS = ROBOTS_PER_ALLIANCE * 2;

    // Robot collision radius
    private static final double ROBOT_COLLISION_RADIUS = Constants.Robot.LENGTH_WITH_BUMPERS / 2.0;

    // All robots in the match
    private final RobotState[] robots;
    private final InputState[] inputs;
    private final AIRobotController[] aiControllers;

    // Player's robot index (default is blue alliance, slot 0)
    private int playerRobotIndex = 0;

    // Team numbers for AI robots
    private static final int[] BLUE_TEAM_NUMBERS = {3164, 254, 1678};  // Player + 2 AI
    private static final int[] RED_TEAM_NUMBERS = {118, 2056, 148};

    private final Random random = new Random();

    /**
     * Create the multi-robot manager.
     */
    public MultiRobotManager() {
        robots = new RobotState[TOTAL_ROBOTS];
        inputs = new InputState[TOTAL_ROBOTS];
        aiControllers = new AIRobotController[TOTAL_ROBOTS];

        initializeRobots();
    }

    /**
     * Initialize all robots with their positions and controllers.
     */
    private void initializeRobots() {
        // Blue alliance robots (indices 0-2)
        for (int i = 0; i < ROBOTS_PER_ALLIANCE; i++) {
            robots[i] = new RobotState();
            inputs[i] = new InputState();

            boolean isPlayer = (i == 0);  // First blue robot is player
            robots[i].initialize(i, BLUE_TEAM_NUMBERS[i], MatchState.Alliance.BLUE, i, isPlayer);

            if (!isPlayer) {
                aiControllers[i] = new AIRobotController(i, MatchState.Alliance.BLUE, BLUE_TEAM_NUMBERS[i]);
            }
        }

        // Red alliance robots (indices 3-5)
        for (int i = 0; i < ROBOTS_PER_ALLIANCE; i++) {
            int index = ROBOTS_PER_ALLIANCE + i;
            robots[index] = new RobotState();
            inputs[index] = new InputState();

            robots[index].initialize(index, RED_TEAM_NUMBERS[i], MatchState.Alliance.RED, i, false);
            aiControllers[index] = new AIRobotController(index, MatchState.Alliance.RED, RED_TEAM_NUMBERS[i]);
        }

        playerRobotIndex = 0;
    }

    /**
     * Reset all robots to starting positions.
     */
    public void reset() {
        for (int i = 0; i < TOTAL_ROBOTS; i++) {
            int slot = i % ROBOTS_PER_ALLIANCE;
            MatchState.Alliance alliance = i < ROBOTS_PER_ALLIANCE ?
                    MatchState.Alliance.BLUE : MatchState.Alliance.RED;
            int teamNumber = i < ROBOTS_PER_ALLIANCE ?
                    BLUE_TEAM_NUMBERS[slot] : RED_TEAM_NUMBERS[slot];

            robots[i].initialize(i, teamNumber, alliance, slot, i == playerRobotIndex);

            if (aiControllers[i] != null) {
                aiControllers[i].reset();
            }
        }
    }

    /**
     * Start autonomous for all AI robots.
     */
    public void startAuto() {
        for (int i = 0; i < TOTAL_ROBOTS; i++) {
            if (aiControllers[i] != null) {
                aiControllers[i].startAuto(robots[i]);
            }
        }
    }

    /**
     * Update all robots.
     *
     * @param playerInput Input from the player
     * @param matchState Current match state
     * @param dt Time delta
     */
    public void update(InputState playerInput, MatchState matchState, double dt) {
        // Copy player input to player's robot
        copyInput(playerInput, inputs[playerRobotIndex]);

        // Update AI controllers
        for (int i = 0; i < TOTAL_ROBOTS; i++) {
            if (aiControllers[i] != null && i != playerRobotIndex) {
                aiControllers[i].update(robots[i], inputs[i], matchState, robots, dt);
            }
        }
    }

    /**
     * Check and resolve robot-robot collisions.
     */
    public void resolveRobotCollisions() {
        for (int i = 0; i < TOTAL_ROBOTS; i++) {
            for (int j = i + 1; j < TOTAL_ROBOTS; j++) {
                resolveCollision(robots[i], robots[j]);
            }
        }
    }

    /**
     * Resolve collision between two robots.
     */
    private void resolveCollision(RobotState a, RobotState b) {
        double dx = b.x - a.x;
        double dy = b.y - a.y;
        double dist = Math.hypot(dx, dy);

        double minDist = ROBOT_COLLISION_RADIUS * 2;

        if (dist < minDist && dist > 0.001) {
            // Collision detected - push robots apart
            double overlap = minDist - dist;
            double pushX = (dx / dist) * overlap * 0.5;
            double pushY = (dy / dist) * overlap * 0.5;

            a.x -= pushX;
            a.y -= pushY;
            b.x += pushX;
            b.y += pushY;

            // Dampen velocities
            double dampFactor = 0.5;

            // Exchange some momentum
            double avx = a.vx;
            double avy = a.vy;
            a.vx = b.vx * dampFactor;
            a.vy = b.vy * dampFactor;
            b.vx = avx * dampFactor;
            b.vy = avy * dampFactor;
        }
    }

    /**
     * Get all robots.
     */
    public RobotState[] getAllRobots() {
        return robots;
    }

    /**
     * Get all input states.
     */
    public InputState[] getAllInputs() {
        return inputs;
    }

    /**
     * Get the player's robot state.
     */
    public RobotState getPlayerRobot() {
        return robots[playerRobotIndex];
    }

    /**
     * Get the player's input state.
     */
    public InputState getPlayerInput() {
        return inputs[playerRobotIndex];
    }

    /**
     * Get a specific robot by index.
     */
    public RobotState getRobot(int index) {
        if (index >= 0 && index < TOTAL_ROBOTS) {
            return robots[index];
        }
        return null;
    }

    /**
     * Get input for a specific robot.
     */
    public InputState getInput(int index) {
        if (index >= 0 && index < TOTAL_ROBOTS) {
            return inputs[index];
        }
        return null;
    }

    /**
     * Get AI controller for a robot.
     */
    public AIRobotController getAIController(int index) {
        if (index >= 0 && index < TOTAL_ROBOTS) {
            return aiControllers[index];
        }
        return null;
    }

    /**
     * Get blue alliance robots.
     */
    public RobotState[] getBlueRobots() {
        RobotState[] blue = new RobotState[ROBOTS_PER_ALLIANCE];
        System.arraycopy(robots, 0, blue, 0, ROBOTS_PER_ALLIANCE);
        return blue;
    }

    /**
     * Get red alliance robots.
     */
    public RobotState[] getRedRobots() {
        RobotState[] red = new RobotState[ROBOTS_PER_ALLIANCE];
        System.arraycopy(robots, ROBOTS_PER_ALLIANCE, red, 0, ROBOTS_PER_ALLIANCE);
        return red;
    }

    /**
     * Set which robot the player controls.
     */
    public void setPlayerRobot(int index) {
        if (index >= 0 && index < TOTAL_ROBOTS) {
            // Remove player control from old robot
            robots[playerRobotIndex].isPlayerControlled = false;
            if (aiControllers[playerRobotIndex] == null) {
                // Create AI controller for the robot we're leaving
                aiControllers[playerRobotIndex] = new AIRobotController(
                        playerRobotIndex,
                        robots[playerRobotIndex].alliance,
                        robots[playerRobotIndex].teamNumber
                );
            }

            // Set new player robot
            playerRobotIndex = index;
            robots[index].isPlayerControlled = true;
            aiControllers[index] = null;  // Remove AI from player robot
        }
    }

    /**
     * Get the player robot index.
     */
    public int getPlayerRobotIndex() {
        return playerRobotIndex;
    }

    /**
     * Convert the player robot to AI control for benchmarking purposes.
     *
     * @param index    Robot index to convert
     * @param autoMode Auto mode to use (0-9)
     */
    public void convertPlayerToAI(int index, int autoMode) {
        if (index >= 0 && index < TOTAL_ROBOTS) {
            robots[index].isPlayerControlled = false;
            aiControllers[index] = new AIRobotController(
                    index,
                    robots[index].alliance,
                    robots[index].teamNumber
            );
            aiControllers[index].setAutoMode(autoMode);
        }
    }

    /**
     * Copy input state.
     */
    private void copyInput(InputState from, InputState to) {
        to.forward = from.forward;
        to.strafe = from.strafe;
        to.turn = from.turn;
        to.shooterAngle = from.shooterAngle;
        to.shooterPower = from.shooterPower;
        to.intake = from.intake;
        to.shoot = from.shoot;
        to.spinUp = from.spinUp;
        to.climberUp = from.climberUp;
        to.climberDown = from.climberDown;
        to.level1 = from.level1;
        to.level2 = from.level2;
        to.level3 = from.level3;
        to.toggleTrenchMode = from.toggleTrenchMode;
        to.toggleSpeed = from.toggleSpeed;
        to.toggleFieldRel = from.toggleFieldRel;
        to.resetGyro = from.resetGyro;
        to.skiStop = from.skiStop;
        to.resetRobot = from.resetRobot;
        to.redChuteRelease = from.redChuteRelease;
        to.blueChuteRelease = from.blueChuteRelease;
        to.redCorralTransfer = from.redCorralTransfer;
        to.blueCorralTransfer = from.blueCorralTransfer;
        to.startMatch = from.startMatch;
        to.pauseMatch = from.pauseMatch;
    }

    /**
     * Get summary of AI auto modes for logging.
     */
    public String getAutoModesSummary() {
        StringBuilder sb = new StringBuilder();
        sb.append("Auto Modes:\n");

        sb.append("  BLUE: ");
        for (int i = 0; i < ROBOTS_PER_ALLIANCE; i++) {
            sb.append(robots[i].teamNumber).append("=");
            if (i == playerRobotIndex) {
                sb.append("PLAYER");
            } else {
                sb.append(aiControllers[i].getAutoModeName());
            }
            if (i < ROBOTS_PER_ALLIANCE - 1) sb.append(", ");
        }
        sb.append("\n");

        sb.append("  RED:  ");
        for (int i = 0; i < ROBOTS_PER_ALLIANCE; i++) {
            int idx = ROBOTS_PER_ALLIANCE + i;
            sb.append(robots[idx].teamNumber).append("=");
            sb.append(aiControllers[idx].getAutoModeName());
            if (i < ROBOTS_PER_ALLIANCE - 1) sb.append(", ");
        }

        return sb.toString();
    }
}
