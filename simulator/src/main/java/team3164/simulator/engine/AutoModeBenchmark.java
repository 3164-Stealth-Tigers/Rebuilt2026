package team3164.simulator.engine;

import team3164.simulator.Constants;
import team3164.simulator.physics.*;
import team3164.simulator.engine.FuelState.Fuel;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Benchmark runner for testing specific autonomous modes.
 * Runs multiple simulations and outputs detailed statistics.
 */
public class AutoModeBenchmark {

    private final int targetTeam;
    private final int targetAutoMode;
    private final String autoModeName;

    /**
     * Create a benchmark for a specific team and auto mode.
     *
     * @param targetTeam     The team number to track (e.g., 3164)
     * @param targetAutoMode The auto mode to force (0-9)
     */
    public AutoModeBenchmark(int targetTeam, int targetAutoMode) {
        this.targetTeam = targetTeam;
        this.targetAutoMode = targetAutoMode;
        this.autoModeName = AutonomousController.AUTO_MODE_NAMES[targetAutoMode];
    }

    /**
     * Result data for a single match.
     */
    public static class MatchResult {
        public int matchNumber;
        public int fuelScored;
        public int fuelCollectedDuringAuto;
        public int climbLevel;
        public boolean climbComplete;
        public double totalDistance;
        public double timeMoving;
        public double timeIdle;
        public double timeShooting;
        public double timeIntaking;
        public String alliance;
        public int allianceScore;
        public int opponentScore;
        public boolean allianceWon;
        public boolean autoWon;
        public int autoFuelScored;

        // Phase-specific metrics
        public double autoEndFuelScored;
        public double autoEndX;
        public double autoEndY;
        public String autoEndPhase;

        // Timing breakdown
        public double timeInShootingPhase;
        public double timeInDrivingPhase;
        public double timeInIntakePhase;
        public double firstShotTime;
        public double lastShotTime;
    }

    /**
     * Run the benchmark with specified number of matches.
     *
     * @param matchCount Number of matches to simulate
     * @param outputFile Path to output JSON file (null for no file output)
     * @return List of match results
     */
    public List<MatchResult> runBenchmark(int matchCount, String outputFile) {
        List<MatchResult> results = new ArrayList<>();

        System.out.println("=".repeat(70));
        System.out.println("AUTO MODE BENCHMARK");
        System.out.println("Team: " + targetTeam + " | Mode: " + autoModeName);
        System.out.println("Running " + matchCount + " simulations...");
        System.out.println("=".repeat(70));

        long startTime = System.currentTimeMillis();

        for (int i = 0; i < matchCount; i++) {
            if ((i + 1) % 50 == 0 || i == 0) {
                System.out.printf("  Progress: %d / %d (%.0f%%)%n", i + 1, matchCount,
                    ((i + 1) * 100.0 / matchCount));
            }

            MatchResult result = runSingleMatch(i + 1);
            results.add(result);
        }

        long elapsed = System.currentTimeMillis() - startTime;
        System.out.printf("%nCompleted %d matches in %.1f seconds (%.1f matches/sec)%n",
            matchCount, elapsed / 1000.0, matchCount * 1000.0 / elapsed);

        // Calculate and print summary statistics
        printSummary(results);

        // Output to file if specified
        if (outputFile != null) {
            try {
                writeResultsToJson(results, outputFile);
                System.out.println("\nResults saved to: " + outputFile);
            } catch (IOException e) {
                System.err.println("Failed to write results: " + e.getMessage());
            }
        }

        return results;
    }

    /**
     * Run only the autonomous period (20 seconds) and collect statistics.
     */
    private MatchResult runSingleMatch(int matchNumber) {
        MatchState matchState = new MatchState();
        FuelState fuelState = new FuelState();

        // Reset
        matchState.reset();
        fuelState.reset();

        // Create single robot state for team 3164
        RobotState targetRobot = new RobotState();
        targetRobot.robotId = 0;
        targetRobot.teamNumber = targetTeam;
        targetRobot.alliance = MatchState.Alliance.BLUE;
        targetRobot.isPlayerControlled = false;

        // Starting position (typical blue alliance starting position)
        targetRobot.x = 1.5;
        targetRobot.y = Constants.Field.CENTER_Y;
        targetRobot.heading = 0;
        targetRobot.fuelCount = Constants.Fuel.PRELOAD_PER_ROBOT; // 3 preloaded FUEL

        InputState input = new InputState();

        // Create autonomous controller for this robot
        AutonomousController autoController = new AutonomousController();
        autoController.setSelectedMode(targetAutoMode);

        // Start match and AUTO
        matchState.startMatch();
        autoController.startAuto(targetRobot);

        // Run simulation for AUTO period only (20 seconds)
        double dt = Constants.Simulation.DT;
        MatchResult result = new MatchResult();
        result.matchNumber = matchNumber;
        result.alliance = targetRobot.alliance.name();

        // Track metrics during AUTO
        double startX = targetRobot.x;
        double startY = targetRobot.y;
        double totalDistance = 0;
        double lastX = targetRobot.x;
        double lastY = targetRobot.y;
        int fuelCollected = 0;
        int startingFuel = targetRobot.fuelCount;

        // Timing tracking
        double timeInShootingPhase = 0;
        double timeInDrivingPhase = 0;
        double timeInIntakePhase = 0;
        double firstShotTime = -1;
        double lastShotTime = -1;
        int lastFuelScored = 0;

        // Run only AUTO period (0-20 seconds)
        while (matchState.matchTime < Constants.Match.AUTO_END) {
            matchState.matchTime += dt;
            matchState.updatePhase();

            // Update autonomous controller
            autoController.update(targetRobot, input, dt);

            // Update physics
            SwervePhysics.update(targetRobot, input, dt);
            CollisionPhysics.checkAndResolveCollisions(targetRobot);
            BumpPhysics.update(targetRobot, dt);

            if (!TrenchPhysics.update(targetRobot, dt)) {
                double[] pushOut = TrenchPhysics.getPushOutVector(targetRobot.x, targetRobot.y);
                if (pushOut != null) {
                    targetRobot.x += pushOut[0];
                    targetRobot.y += pushOut[1];
                    targetRobot.vx = 0;
                    targetRobot.vy = 0;
                }
            }

            ShooterPhysics.update(targetRobot, input, fuelState, matchState, dt);
            IntakePhysics.update(targetRobot, input, fuelState, dt);

            // Track distance
            double dist = Math.hypot(targetRobot.x - lastX, targetRobot.y - lastY);
            totalDistance += dist;
            lastX = targetRobot.x;
            lastY = targetRobot.y;

            // Track timing by phase
            String phaseName = autoController.getCurrentPhaseName();
            if (phaseName.contains("SCORING") || phaseName.contains("SHOOTING")) {
                timeInShootingPhase += dt;
            } else if (phaseName.contains("DRIVING") || phaseName.contains("POSITIONING")) {
                timeInDrivingPhase += dt;
            } else if (phaseName.contains("INTAKING")) {
                timeInIntakePhase += dt;
            }

            // Track shot times
            if (matchState.blueFuelScored > lastFuelScored) {
                if (firstShotTime < 0) {
                    firstShotTime = matchState.matchTime;
                }
                lastShotTime = matchState.matchTime;
                lastFuelScored = matchState.blueFuelScored;
            }

            // Update fuel physics
            for (Fuel fuel : fuelState.getFieldFuel()) {
                FuelPhysics.checkRobotCollision(fuel, targetRobot);
            }
            FuelPhysics.update(fuelState, matchState, dt);
        }

        // Collect results - use matchState for accurate scoring
        result.fuelScored = matchState.blueFuelScored;
        result.autoFuelScored = matchState.blueFuelScored;
        result.totalDistance = totalDistance;
        result.autoEndX = targetRobot.x;
        result.autoEndY = targetRobot.y;
        result.autoEndPhase = autoController.getCurrentPhaseName();

        // Calculate fuel collected (current fuel + scored - starting fuel)
        result.fuelCollectedDuringAuto = (targetRobot.fuelCount + matchState.blueFuelScored) - startingFuel;
        if (result.fuelCollectedDuringAuto < 0) result.fuelCollectedDuringAuto = 0;

        // Climbing during AUTO
        result.climbLevel = targetRobot.climbLevel;
        result.climbComplete = targetRobot.climbComplete;

        // Alliance scoring (from matchState)
        result.allianceScore = matchState.blueAutoScore;
        result.autoWon = matchState.blueFuelScored > 0;

        // Timing breakdown
        result.timeInShootingPhase = timeInShootingPhase;
        result.timeInDrivingPhase = timeInDrivingPhase;
        result.timeInIntakePhase = timeInIntakePhase;
        result.firstShotTime = firstShotTime;
        result.lastShotTime = lastShotTime;

        return result;
    }

    /**
     * Print summary statistics.
     */
    private void printSummary(List<MatchResult> results) {
        System.out.println("\n" + "=".repeat(70));
        System.out.println("BENCHMARK SUMMARY - " + autoModeName);
        System.out.println("=".repeat(70));

        // Calculate statistics
        double avgFuelScored = results.stream().mapToInt(r -> r.fuelScored).average().orElse(0);
        double avgAutoFuel = results.stream().mapToInt(r -> r.autoFuelScored).average().orElse(0);
        double avgDistance = results.stream().mapToDouble(r -> r.totalDistance).average().orElse(0);
        double avgTimeMoving = results.stream().mapToDouble(r -> r.timeMoving).average().orElse(0);
        double avgTimeIdle = results.stream().mapToDouble(r -> r.timeIdle).average().orElse(0);

        int climbCount = (int) results.stream().filter(r -> r.climbComplete).count();
        double climbRate = climbCount * 100.0 / results.size();

        int allianceWins = (int) results.stream().filter(r -> r.allianceWon).count();
        double winRate = allianceWins * 100.0 / results.size();

        int autoWins = (int) results.stream().filter(r -> r.autoWon).count();
        double autoWinRate = autoWins * 100.0 / results.size();

        // Find min/max
        int minFuel = results.stream().mapToInt(r -> r.fuelScored).min().orElse(0);
        int maxFuel = results.stream().mapToInt(r -> r.fuelScored).max().orElse(0);

        // Calculate standard deviation
        double variance = results.stream()
            .mapToDouble(r -> Math.pow(r.fuelScored - avgFuelScored, 2))
            .average().orElse(0);
        double stdDev = Math.sqrt(variance);

        System.out.printf("Total Matches: %d%n", results.size());
        System.out.println();
        System.out.println("FUEL SCORING:");
        System.out.printf("  Average FUEL Scored: %.2f%n", avgFuelScored);
        System.out.printf("  Std Deviation: %.2f%n", stdDev);
        System.out.printf("  Min: %d, Max: %d%n", minFuel, maxFuel);
        System.out.printf("  Average AUTO FUEL: %.2f%n", avgAutoFuel);
        System.out.println();
        System.out.println("CLIMBING:");
        System.out.printf("  Climb Success Rate: %.1f%% (%d/%d)%n", climbRate, climbCount, results.size());
        System.out.println();
        System.out.println("MOVEMENT:");
        System.out.printf("  Average Distance: %.1f m%n", avgDistance);

        // Timing breakdown
        double avgShootTime = results.stream().mapToDouble(r -> r.timeInShootingPhase).average().orElse(0);
        double avgDriveTime = results.stream().mapToDouble(r -> r.timeInDrivingPhase).average().orElse(0);
        double avgIntakeTime = results.stream().mapToDouble(r -> r.timeInIntakePhase).average().orElse(0);
        double avgFirstShot = results.stream().mapToDouble(r -> r.firstShotTime).filter(t -> t >= 0).average().orElse(-1);
        double avgLastShot = results.stream().mapToDouble(r -> r.lastShotTime).filter(t -> t >= 0).average().orElse(-1);
        double avgEndX = results.stream().mapToDouble(r -> r.autoEndX).average().orElse(0);
        double avgEndY = results.stream().mapToDouble(r -> r.autoEndY).average().orElse(0);

        System.out.println();
        System.out.println("TIMING BREAKDOWN (20s AUTO):");
        System.out.printf("  Time in Shooting Phase: %.1f s (%.0f%%)%n", avgShootTime, avgShootTime/20*100);
        System.out.printf("  Time in Driving Phase: %.1f s (%.0f%%)%n", avgDriveTime, avgDriveTime/20*100);
        System.out.printf("  Time in Intake Phase: %.1f s (%.0f%%)%n", avgIntakeTime, avgIntakeTime/20*100);
        System.out.printf("  First Shot Time: %.2f s%n", avgFirstShot);
        System.out.printf("  Last Shot Time: %.2f s%n", avgLastShot);
        System.out.println();
        System.out.println("END POSITION:");
        System.out.printf("  Average End Position: (%.1f, %.1f)%n", avgEndX, avgEndY);
        System.out.printf("  Distance to Neutral Zone: %.1f m%n", Math.abs(Constants.Field.CENTER_X - avgEndX));
        System.out.printf("  End Phase: %s%n", results.get(0).autoEndPhase);
        System.out.println();
        System.out.println("WIN RATES:");
        System.out.printf("  Alliance Win Rate: %.1f%% (%d/%d)%n", winRate, allianceWins, results.size());
        System.out.printf("  AUTO Win Rate: %.1f%% (%d/%d)%n", autoWinRate, autoWins, results.size());
        System.out.println("=".repeat(70));
    }

    /**
     * Write results to JSON file.
     */
    private void writeResultsToJson(List<MatchResult> results, String filePath) throws IOException {
        try (FileWriter writer = new FileWriter(filePath)) {
            writer.write("{\n");
            writer.write("  \"benchmark\": {\n");
            writer.write("    \"team\": " + targetTeam + ",\n");
            writer.write("    \"autoMode\": " + targetAutoMode + ",\n");
            writer.write("    \"autoModeName\": \"" + autoModeName + "\",\n");
            writer.write("    \"matchCount\": " + results.size() + "\n");
            writer.write("  },\n");
            writer.write("  \"results\": [\n");

            for (int i = 0; i < results.size(); i++) {
                MatchResult r = results.get(i);
                writer.write("    {\n");
                writer.write("      \"match\": " + r.matchNumber + ",\n");
                writer.write("      \"fuelScored\": " + r.fuelScored + ",\n");
                writer.write("      \"autoFuelScored\": " + r.autoFuelScored + ",\n");
                writer.write("      \"climbLevel\": " + r.climbLevel + ",\n");
                writer.write("      \"climbComplete\": " + r.climbComplete + ",\n");
                writer.write("      \"totalDistance\": " + String.format("%.1f", r.totalDistance) + ",\n");
                writer.write("      \"timeMoving\": " + String.format("%.1f", r.timeMoving) + ",\n");
                writer.write("      \"timeIdle\": " + String.format("%.1f", r.timeIdle) + ",\n");
                writer.write("      \"alliance\": \"" + r.alliance + "\",\n");
                writer.write("      \"allianceScore\": " + r.allianceScore + ",\n");
                writer.write("      \"opponentScore\": " + r.opponentScore + ",\n");
                writer.write("      \"allianceWon\": " + r.allianceWon + ",\n");
                writer.write("      \"autoWon\": " + r.autoWon + ",\n");
                writer.write("      \"autoEndX\": " + String.format("%.2f", r.autoEndX) + ",\n");
                writer.write("      \"autoEndY\": " + String.format("%.2f", r.autoEndY) + "\n");
                writer.write("    }" + (i < results.size() - 1 ? "," : "") + "\n");
            }

            writer.write("  ]\n");
            writer.write("}\n");
        }
    }

    /**
     * Main entry point for running benchmarks.
     */
    public static void main(String[] args) {
        int team = 3164;
        int autoMode = 1;  // Score & Collect
        int matchCount = 500;
        String outputFile = "benchmark_results.json";

        if (args.length >= 1) {
            matchCount = Integer.parseInt(args[0]);
        }
        if (args.length >= 2) {
            autoMode = Integer.parseInt(args[1]);
        }
        if (args.length >= 3) {
            outputFile = args[2];
        }

        AutoModeBenchmark benchmark = new AutoModeBenchmark(team, autoMode);
        benchmark.runBenchmark(matchCount, outputFile);
    }
}
