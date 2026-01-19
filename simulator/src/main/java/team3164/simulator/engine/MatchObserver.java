package team3164.simulator.engine;

import team3164.simulator.Constants;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Observes and records match events for analysis.
 * Tracks robot actions, scoring, and generates summary reports.
 */
public class MatchObserver {

    // Event log
    private final List<MatchEvent> events = new ArrayList<>();

    // Per-robot tracking
    private final Map<Integer, RobotStats> robotStats = new HashMap<>();

    // Timing
    private double lastSampleTime = 0;
    private static final double SAMPLE_INTERVAL = 1.0; // Sample every 1 second

    // Phase tracking
    private MatchState.MatchPhase lastPhase = null;

    /**
     * Event types for logging.
     */
    public enum EventType {
        MATCH_START,
        PHASE_CHANGE,
        FUEL_SCORED,
        FUEL_COLLECTED,
        FUEL_SHOT,
        CLIMB_START,
        CLIMB_COMPLETE,
        ROBOT_MOVEMENT,
        MATCH_END
    }

    /**
     * A single match event.
     */
    public static class MatchEvent {
        public final double time;
        public final EventType type;
        public final int robotId;
        public final String alliance;
        public final String description;
        public final Map<String, Object> data;

        public MatchEvent(double time, EventType type, int robotId, String alliance, String description) {
            this.time = time;
            this.type = type;
            this.robotId = robotId;
            this.alliance = alliance;
            this.description = description;
            this.data = new HashMap<>();
        }

        public MatchEvent withData(String key, Object value) {
            this.data.put(key, value);
            return this;
        }

        @Override
        public String toString() {
            return String.format("[%5.1fs] %-15s Robot %d (%s): %s",
                    time, type, robotId, alliance, description);
        }
    }

    /**
     * Per-robot statistics.
     */
    public static class RobotStats {
        public int robotId;
        public String alliance;
        public int teamNumber;
        public boolean isPlayer;

        // Actions
        public int fuelCollected = 0;
        public int fuelShot = 0;
        public int fuelScored = 0;
        public int climbLevel = 0;
        public boolean climbComplete = false;

        // Movement
        public double totalDistance = 0;
        public double lastX = 0;
        public double lastY = 0;
        public boolean positionInitialized = false;

        // Time tracking
        public double timeMoving = 0;
        public double timeShooting = 0;
        public double timeIntaking = 0;
        public double timeIdle = 0;

        public String autoMode = "";
        public String teleopBehavior = "";
    }

    /**
     * Initialize tracking for all robots.
     */
    public void initialize(RobotState[] robots, MultiRobotManager manager) {
        events.clear();
        robotStats.clear();
        lastSampleTime = 0;
        lastPhase = null;

        for (RobotState robot : robots) {
            RobotStats stats = new RobotStats();
            stats.robotId = robot.robotId;
            stats.alliance = robot.alliance.name();
            stats.teamNumber = robot.teamNumber;
            stats.isPlayer = robot.isPlayerControlled;

            if (!robot.isPlayerControlled && manager != null) {
                AIRobotController ai = manager.getAIController(robot.robotId);
                if (ai != null) {
                    stats.autoMode = ai.getAutoModeName();
                    stats.teleopBehavior = ai.getTeleopBehavior().name();
                }
            } else {
                stats.autoMode = "PLAYER";
                stats.teleopBehavior = "PLAYER";
            }

            robotStats.put(robot.robotId, stats);
        }
    }

    /**
     * Record match start.
     */
    public void onMatchStart(MatchState matchState) {
        events.add(new MatchEvent(0, EventType.MATCH_START, -1, "ALL", "Match started"));
    }

    /**
     * Update observer with current state.
     * Should be called every simulation tick.
     */
    public void update(RobotState[] robots, MatchState matchState, double dt) {
        double currentTime = matchState.matchTime;

        // Check phase changes
        if (matchState.currentPhase != lastPhase) {
            lastPhase = matchState.currentPhase;
            events.add(new MatchEvent(currentTime, EventType.PHASE_CHANGE, -1, "ALL",
                    "Phase: " + matchState.currentPhase.name())
                    .withData("phase", matchState.currentPhase.name())
                    .withData("redHubActive", matchState.redHubStatus == MatchState.HubStatus.ACTIVE)
                    .withData("blueHubActive", matchState.blueHubStatus == MatchState.HubStatus.ACTIVE));
        }

        // Sample robot states periodically
        if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
            lastSampleTime = currentTime;

            for (RobotState robot : robots) {
                RobotStats stats = robotStats.get(robot.robotId);
                if (stats == null) continue;

                // Track movement
                if (stats.positionInitialized) {
                    double dist = Math.hypot(robot.x - stats.lastX, robot.y - stats.lastY);
                    stats.totalDistance += dist;

                    // Categorize time
                    double speed = Math.hypot(robot.vx, robot.vy);
                    if (speed > 0.5) {
                        stats.timeMoving += SAMPLE_INTERVAL;
                    } else if (robot.intakeState == RobotState.IntakeState.SHOOTING) {
                        stats.timeShooting += SAMPLE_INTERVAL;
                    } else if (robot.intakeState == RobotState.IntakeState.INTAKING) {
                        stats.timeIntaking += SAMPLE_INTERVAL;
                    } else {
                        stats.timeIdle += SAMPLE_INTERVAL;
                    }
                }

                stats.lastX = robot.x;
                stats.lastY = robot.y;
                stats.positionInitialized = true;

                // Track scoring
                if (robot.fuelScored > stats.fuelScored) {
                    int scored = robot.fuelScored - stats.fuelScored;
                    stats.fuelScored = robot.fuelScored;
                    events.add(new MatchEvent(currentTime, EventType.FUEL_SCORED,
                            robot.robotId, robot.alliance.name(),
                            "Scored " + scored + " FUEL (total: " + stats.fuelScored + ")")
                            .withData("count", scored)
                            .withData("total", stats.fuelScored));
                }

                // Track climbing
                if (robot.climbComplete && !stats.climbComplete) {
                    stats.climbComplete = true;
                    stats.climbLevel = robot.climbLevel;
                    events.add(new MatchEvent(currentTime, EventType.CLIMB_COMPLETE,
                            robot.robotId, robot.alliance.name(),
                            "Completed climb to L" + robot.climbLevel)
                            .withData("level", robot.climbLevel));
                } else if (robot.isClimbing && stats.climbLevel != robot.climbLevel) {
                    stats.climbLevel = robot.climbLevel;
                    events.add(new MatchEvent(currentTime, EventType.CLIMB_START,
                            robot.robotId, robot.alliance.name(),
                            "Started climbing to L" + robot.climbLevel)
                            .withData("level", robot.climbLevel));
                }
            }
        }
    }

    /**
     * Record match end.
     */
    public void onMatchEnd(MatchState matchState) {
        events.add(new MatchEvent(matchState.matchTime, EventType.MATCH_END, -1, "ALL",
                "Match ended - RED: " + matchState.redTotalScore + " BLUE: " + matchState.blueTotalScore)
                .withData("redScore", matchState.redTotalScore)
                .withData("blueScore", matchState.blueTotalScore)
                .withData("redFuel", matchState.redFuelScored)
                .withData("blueFuel", matchState.blueFuelScored)
                .withData("redTower", matchState.redTowerPoints)
                .withData("blueTower", matchState.blueTowerPoints));
    }

    /**
     * Generate a text summary of the match.
     */
    public String generateSummary(MatchState matchState) {
        StringBuilder sb = new StringBuilder();

        sb.append("╔══════════════════════════════════════════════════════════════════╗\n");
        sb.append("║                    REBUILT 2026 MATCH SUMMARY                    ║\n");
        sb.append("╠══════════════════════════════════════════════════════════════════╣\n");

        // Final scores
        sb.append("║ FINAL SCORE                                                      ║\n");
        sb.append(String.format("║   RED:  %3d pts (FUEL: %3d, TOWER: %3d)                          ║\n",
                matchState.redTotalScore, matchState.redFuelScored, matchState.redTowerPoints));
        sb.append(String.format("║   BLUE: %3d pts (FUEL: %3d, TOWER: %3d)                          ║\n",
                matchState.blueTotalScore, matchState.blueFuelScored, matchState.blueTowerPoints));

        // Winner
        String winner = matchState.redTotalScore > matchState.blueTotalScore ? "RED WINS!" :
                matchState.blueTotalScore > matchState.redTotalScore ? "BLUE WINS!" : "TIE!";
        sb.append(String.format("║   RESULT: %-54s ║\n", winner));

        // Ranking points
        sb.append("║                                                                  ║\n");
        sb.append("║ RANKING POINTS                                                   ║\n");
        sb.append(String.format("║   RED:  Energized=%s, Traversal=%s                             ║\n",
                matchState.redEnergized ? "YES" : "NO ", matchState.redTraversal ? "YES" : "NO "));
        sb.append(String.format("║   BLUE: Energized=%s, Traversal=%s                             ║\n",
                matchState.blueEnergized ? "YES" : "NO ", matchState.blueTraversal ? "YES" : "NO "));

        // Auto results
        sb.append("║                                                                  ║\n");
        sb.append("║ AUTO PERIOD                                                      ║\n");
        sb.append(String.format("║   Winner: %-54s ║\n",
                matchState.autoWinner != null ? matchState.autoWinner.name() : "TIE"));

        // Per-robot stats
        sb.append("║                                                                  ║\n");
        sb.append("╠══════════════════════════════════════════════════════════════════╣\n");
        sb.append("║ ROBOT STATISTICS                                                 ║\n");
        sb.append("╠══════════════════════════════════════════════════════════════════╣\n");

        for (RobotStats stats : robotStats.values()) {
            sb.append(String.format("║ Team %4d (%s) - %s                                    ║\n",
                    stats.teamNumber, stats.alliance.substring(0, 1),
                    stats.isPlayer ? "PLAYER" : stats.teleopBehavior));
            sb.append(String.format("║   Auto Mode: %-51s ║\n", stats.autoMode));
            sb.append(String.format("║   FUEL Scored: %d, Climb: %s                                     ║\n",
                    stats.fuelScored,
                    stats.climbComplete ? "L" + stats.climbLevel : "None"));
            sb.append(String.format("║   Distance: %.1fm, Moving: %.0fs, Idle: %.0fs                    ║\n",
                    stats.totalDistance, stats.timeMoving, stats.timeIdle));
            sb.append("║                                                                  ║\n");
        }

        sb.append("╠══════════════════════════════════════════════════════════════════╣\n");
        sb.append("║ KEY EVENTS                                                       ║\n");
        sb.append("╠══════════════════════════════════════════════════════════════════╣\n");

        // Show important events
        int eventCount = 0;
        for (MatchEvent event : events) {
            if (event.type == EventType.PHASE_CHANGE ||
                event.type == EventType.FUEL_SCORED ||
                event.type == EventType.CLIMB_COMPLETE ||
                event.type == EventType.MATCH_END) {
                sb.append(String.format("║ [%5.1fs] %-56s ║\n",
                        event.time, truncate(event.description, 56)));
                eventCount++;
                if (eventCount > 15) {
                    sb.append("║   ... and more events                                            ║\n");
                    break;
                }
            }
        }

        sb.append("╚══════════════════════════════════════════════════════════════════╝\n");

        return sb.toString();
    }

    /**
     * Generate a JSON-style summary for programmatic analysis.
     */
    public String generateJsonSummary(MatchState matchState) {
        StringBuilder sb = new StringBuilder();
        sb.append("{\n");
        sb.append("  \"finalScore\": {\n");
        sb.append("    \"red\": ").append(matchState.redTotalScore).append(",\n");
        sb.append("    \"blue\": ").append(matchState.blueTotalScore).append(",\n");
        sb.append("    \"redFuel\": ").append(matchState.redFuelScored).append(",\n");
        sb.append("    \"blueFuel\": ").append(matchState.blueFuelScored).append(",\n");
        sb.append("    \"redTower\": ").append(matchState.redTowerPoints).append(",\n");
        sb.append("    \"blueTower\": ").append(matchState.blueTowerPoints).append("\n");
        sb.append("  },\n");

        sb.append("  \"rankingPoints\": {\n");
        sb.append("    \"redEnergized\": ").append(matchState.redEnergized).append(",\n");
        sb.append("    \"blueEnergized\": ").append(matchState.blueEnergized).append(",\n");
        sb.append("    \"redTraversal\": ").append(matchState.redTraversal).append(",\n");
        sb.append("    \"blueTraversal\": ").append(matchState.blueTraversal).append("\n");
        sb.append("  },\n");

        sb.append("  \"autoWinner\": \"").append(matchState.autoWinner != null ? matchState.autoWinner.name() : "TIE").append("\",\n");

        sb.append("  \"robots\": [\n");
        boolean first = true;
        for (RobotStats stats : robotStats.values()) {
            if (!first) sb.append(",\n");
            first = false;
            sb.append("    {\n");
            sb.append("      \"robotId\": ").append(stats.robotId).append(",\n");
            sb.append("      \"teamNumber\": ").append(stats.teamNumber).append(",\n");
            sb.append("      \"alliance\": \"").append(stats.alliance).append("\",\n");
            sb.append("      \"isPlayer\": ").append(stats.isPlayer).append(",\n");
            sb.append("      \"autoMode\": \"").append(stats.autoMode).append("\",\n");
            sb.append("      \"teleopBehavior\": \"").append(stats.teleopBehavior).append("\",\n");
            sb.append("      \"fuelScored\": ").append(stats.fuelScored).append(",\n");
            sb.append("      \"climbLevel\": ").append(stats.climbLevel).append(",\n");
            sb.append("      \"climbComplete\": ").append(stats.climbComplete).append(",\n");
            sb.append("      \"totalDistance\": ").append(String.format("%.1f", stats.totalDistance)).append(",\n");
            sb.append("      \"timeMoving\": ").append(String.format("%.0f", stats.timeMoving)).append(",\n");
            sb.append("      \"timeIdle\": ").append(String.format("%.0f", stats.timeIdle)).append("\n");
            sb.append("    }");
        }
        sb.append("\n  ],\n");

        sb.append("  \"eventCount\": ").append(events.size()).append(",\n");
        sb.append("  \"scoringEvents\": ").append(events.stream().filter(e -> e.type == EventType.FUEL_SCORED).count()).append("\n");
        sb.append("}\n");

        return sb.toString();
    }

    /**
     * Get all events for detailed analysis.
     */
    public List<MatchEvent> getEvents() {
        return new ArrayList<>(events);
    }

    /**
     * Get robot stats.
     */
    public Map<Integer, RobotStats> getRobotStats() {
        return new HashMap<>(robotStats);
    }

    private String truncate(String s, int maxLen) {
        if (s.length() <= maxLen) return s;
        return s.substring(0, maxLen - 3) + "...";
    }
}
