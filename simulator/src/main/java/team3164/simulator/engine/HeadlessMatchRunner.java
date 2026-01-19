package team3164.simulator.engine;

import team3164.simulator.Constants;
import team3164.simulator.physics.*;
import team3164.simulator.engine.FuelState.Fuel;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;

/**
 * Runs a complete match without UI, collecting detailed statistics.
 * Useful for automated testing and AI behavior analysis.
 */
public class HeadlessMatchRunner {

    private final MultiRobotManager robotManager;
    private final MatchState matchState;
    private final FuelState fuelState;
    private final MatchObserver observer;

    private final List<String> logs = new ArrayList<>();
    private boolean verbose = true;

    /**
     * Create a headless match runner.
     */
    public HeadlessMatchRunner() {
        this.robotManager = new MultiRobotManager();
        this.matchState = new MatchState();
        this.fuelState = new FuelState();
        this.observer = new MatchObserver();
    }

    /**
     * Run a complete match and return the summary.
     *
     * @param verbose If true, print progress to console
     * @return Match summary string
     */
    public String runMatch(boolean verbose) {
        this.verbose = verbose;

        // Initialize
        robotManager.reset();
        matchState.reset();
        fuelState.reset();
        logs.clear();

        RobotState[] robots = robotManager.getAllRobots();
        InputState[] inputs = robotManager.getAllInputs();

        observer.initialize(robots, robotManager);

        log("=".repeat(70));
        log("REBUILT 2026 HEADLESS MATCH SIMULATOR");
        log("=".repeat(70));
        log("");

        // Log field setup
        log("Field Setup:");
        log(String.format("  Neutral Zone FUEL: %d", fuelState.getFieldFuel().size()));
        log(String.format("  RED Depot FUEL: %d", fuelState.getRedDepotFuel().size()));
        log(String.format("  BLUE Depot FUEL: %d", fuelState.getBlueDepotFuel().size()));
        log(String.format("  RED Chute FUEL: %d", fuelState.getRedChuteFuel().size()));
        log(String.format("  BLUE Chute FUEL: %d", fuelState.getBlueChuteFuel().size()));
        log(String.format("  Total FUEL on field: %d", fuelState.getTotalFuelCount()));
        log("");

        log("Robot Configuration:");
        for (RobotState robot : robots) {
            AIRobotController ai = robotManager.getAIController(robot.robotId);
            String mode = robot.isPlayerControlled ? "PLAYER (idle)" :
                    (ai != null ? ai.getAutoModeName() + " / " + ai.getTeleopBehavior().name() : "AI");
            log(String.format("  [%s] Team %d: %s (preload: %d FUEL)",
                    robot.alliance.name().charAt(0), robot.teamNumber, mode, robot.fuelCount));
        }
        log("");

        // Start match
        matchState.startMatch();
        robotManager.startAuto();
        observer.onMatchStart(matchState);

        log("Match started!");
        log("");

        // Run simulation at accelerated speed
        double dt = Constants.Simulation.DT;
        double matchDuration = Constants.Match.TOTAL_DURATION;
        long startTime = System.currentTimeMillis();

        MatchState.MatchPhase lastPhase = null;

        while (!matchState.matchEnded) {
            // Update match time (accelerated)
            matchState.matchTime += dt;
            matchState.updatePhase();

            // Log phase changes
            if (matchState.currentPhase != lastPhase) {
                lastPhase = matchState.currentPhase;
                logPhaseChange();
            }

            // Update AI controllers
            robotManager.update(inputs[0], matchState, dt); // Pass empty player input

            // Update physics for all robots
            for (int i = 0; i < robots.length; i++) {
                RobotState robot = robots[i];
                InputState input = inputs[i];

                // Physics updates
                SwervePhysics.update(robot, input, dt);
                CollisionPhysics.checkAndResolveCollisions(robot);
                BumpPhysics.update(robot, dt);

                if (!TrenchPhysics.update(robot, dt)) {
                    double[] pushOut = TrenchPhysics.getPushOutVector(robot.x, robot.y);
                    if (pushOut != null) {
                        robot.x += pushOut[0];
                        robot.y += pushOut[1];
                        robot.vx = 0;
                        robot.vy = 0;
                    }
                }

                ShooterPhysics.update(robot, input, fuelState, matchState, dt);
                IntakePhysics.update(robot, input, fuelState, dt);
                TowerPhysics.update(robot, input, matchState, dt);

                for (Fuel fuel : fuelState.getFieldFuel()) {
                    FuelPhysics.checkRobotCollision(fuel, robot);
                }
            }

            robotManager.resolveRobotCollisions();
            FuelPhysics.update(fuelState, matchState, dt);

            // Update observer
            observer.update(robots, matchState, dt);

            // Progress indicator every 20 seconds
            if (verbose && Math.abs(matchState.matchTime % 20.0) < dt) {
                System.out.printf("\r  Progress: %.0f%% (%.0fs / %.0fs)  RED: %d  BLUE: %d",
                        (matchState.matchTime / matchDuration) * 100,
                        matchState.matchTime, matchDuration,
                        matchState.redTotalScore, matchState.blueTotalScore);
            }
        }

        if (verbose) {
            System.out.println(); // New line after progress
        }

        long elapsed = System.currentTimeMillis() - startTime;
        log("");
        log(String.format("Match simulation completed in %.1f seconds (real time)", elapsed / 1000.0));
        log("");

        observer.onMatchEnd(matchState);

        // Generate summary
        String summary = observer.generateSummary(matchState);
        log(summary);

        return summary;
    }

    /**
     * Run a match and return JSON results.
     */
    public String runMatchJson() {
        runMatch(false);
        return observer.generateJsonSummary(matchState);
    }

    /**
     * Run multiple matches and return aggregate statistics.
     */
    public String runMultipleMatches(int count, boolean verbose) {
        int redWins = 0;
        int blueWins = 0;
        int ties = 0;
        int totalRedScore = 0;
        int totalBlueScore = 0;
        int totalRedFuel = 0;
        int totalBlueFuel = 0;

        for (int i = 0; i < count; i++) {
            if (verbose) {
                System.out.println("\n=== Running Match " + (i + 1) + " of " + count + " ===\n");
            }

            // Reset for new match
            robotManager.reset();
            matchState.reset();
            fuelState.reset();

            runMatch(verbose);

            // Aggregate results
            if (matchState.redTotalScore > matchState.blueTotalScore) {
                redWins++;
            } else if (matchState.blueTotalScore > matchState.redTotalScore) {
                blueWins++;
            } else {
                ties++;
            }

            totalRedScore += matchState.redTotalScore;
            totalBlueScore += matchState.blueTotalScore;
            totalRedFuel += matchState.redFuelScored;
            totalBlueFuel += matchState.blueFuelScored;
        }

        StringBuilder sb = new StringBuilder();
        sb.append("\n");
        sb.append("=".repeat(70)).append("\n");
        sb.append("AGGREGATE RESULTS (" + count + " matches)\n");
        sb.append("=".repeat(70)).append("\n");
        sb.append(String.format("Red Wins: %d (%.0f%%)\n", redWins, (redWins * 100.0 / count)));
        sb.append(String.format("Blue Wins: %d (%.0f%%)\n", blueWins, (blueWins * 100.0 / count)));
        sb.append(String.format("Ties: %d\n", ties));
        sb.append("\n");
        sb.append(String.format("Avg Red Score: %.1f\n", totalRedScore / (double) count));
        sb.append(String.format("Avg Blue Score: %.1f\n", totalBlueScore / (double) count));
        sb.append(String.format("Avg Red FUEL: %.1f\n", totalRedFuel / (double) count));
        sb.append(String.format("Avg Blue FUEL: %.1f\n", totalBlueFuel / (double) count));
        sb.append("=".repeat(70)).append("\n");

        return sb.toString();
    }

    /**
     * Get the robot manager for configuration.
     */
    public MultiRobotManager getRobotManager() {
        return robotManager;
    }

    /**
     * Get match state.
     */
    public MatchState getMatchState() {
        return matchState;
    }

    /**
     * Get the observer for detailed analysis.
     */
    public MatchObserver getObserver() {
        return observer;
    }

    /**
     * Get all log messages.
     */
    public List<String> getLogs() {
        return new ArrayList<>(logs);
    }

    private void logPhaseChange() {
        switch (matchState.currentPhase) {
            case AUTO:
                log(">>> AUTO PERIOD (0-20s)");
                break;
            case TRANSITION:
                log(">>> TRANSITION (20-30s)");
                String autoResult = matchState.autoWinner != null ?
                        matchState.autoWinner.name() + " wins AUTO" : "AUTO TIE";
                log("    " + autoResult + " - Red: " + matchState.redFuelScored +
                        " FUEL, Blue: " + matchState.blueFuelScored + " FUEL");
                break;
            case SHIFT_1:
                log(">>> SHIFT 1 (30-55s) - " + getActiveHub() + " HUB ACTIVE");
                break;
            case SHIFT_2:
                log(">>> SHIFT 2 (55-80s) - " + getActiveHub() + " HUB ACTIVE");
                break;
            case SHIFT_3:
                log(">>> SHIFT 3 (80-105s) - " + getActiveHub() + " HUB ACTIVE");
                break;
            case SHIFT_4:
                log(">>> SHIFT 4 (105-130s) - " + getActiveHub() + " HUB ACTIVE");
                break;
            case END_GAME:
                log(">>> END GAME (130-160s) - BOTH HUBS ACTIVE");
                log("    Current: Red " + matchState.redTotalScore + " - Blue " + matchState.blueTotalScore);
                break;
            case POST_MATCH:
                log(">>> MATCH COMPLETE");
                break;
        }
    }

    private String getActiveHub() {
        boolean redActive = matchState.redHubStatus == MatchState.HubStatus.ACTIVE;
        boolean blueActive = matchState.blueHubStatus == MatchState.HubStatus.ACTIVE;
        if (redActive && blueActive) return "BOTH";
        if (redActive) return "RED";
        if (blueActive) return "BLUE";
        return "NONE";
    }

    private void log(String message) {
        logs.add(message);
        if (verbose) {
            System.out.println(message);
        }
    }

    /**
     * Main entry point for running headless matches.
     */
    public static void main(String[] args) {
        HeadlessMatchRunner runner = new HeadlessMatchRunner();

        int matchCount = 1;
        if (args.length > 0) {
            try {
                matchCount = Integer.parseInt(args[0]);
            } catch (NumberFormatException e) {
                System.out.println("Usage: HeadlessMatchRunner [matchCount]");
                return;
            }
        }

        if (matchCount == 1) {
            runner.runMatch(true);
        } else {
            String results = runner.runMultipleMatches(matchCount, false);
            System.out.println(results);
        }
    }
}
