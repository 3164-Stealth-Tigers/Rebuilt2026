package team3164.simulator.web;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import io.javalin.Javalin;
import io.javalin.websocket.WsContext;
import team3164.simulator.Constants;
import team3164.simulator.engine.*;
import team3164.simulator.engine.FuelState.Fuel;

import java.util.Map;
import java.util.concurrent.*;

/**
 * HTTP and WebSocket server for the REBUILT 2026 simulator.
 *
 * Serves the web UI and handles real-time communication with the browser.
 */
public class SimulatorServer {

    private final SimulationEngine engine;
    private final Gson gson;
    private final int port;

    private Javalin app;
    private final Map<String, WsContext> clients = new ConcurrentHashMap<>();
    private ScheduledExecutorService broadcastExecutor;

    /**
     * Create a new simulator server.
     *
     * @param engine The simulation engine to use
     * @param port Port to listen on
     */
    public SimulatorServer(SimulationEngine engine, int port) {
        this.engine = engine;
        this.port = port;
        this.gson = new Gson();
    }

    /**
     * Start the server.
     */
    public void start() {
        app = Javalin.create(config -> {
            config.staticFiles.add(staticFiles -> {
                staticFiles.hostedPath = "/";
                staticFiles.directory = "/web";
                staticFiles.location = io.javalin.http.staticfiles.Location.CLASSPATH;
            });
            config.http.defaultContentType = "text/html";
        });

        // WebSocket endpoint for real-time communication
        app.ws("/ws", ws -> {
            ws.onConnect(ctx -> {
                String id = Integer.toHexString(ctx.hashCode());
                clients.put(id, ctx);
                System.out.println("Client connected: " + id + " (total: " + clients.size() + ")");

                // Send initial state
                sendFullState(ctx);
            });

            ws.onClose(ctx -> {
                String id = Integer.toHexString(ctx.hashCode());
                clients.remove(id);
                System.out.println("Client disconnected: " + id + " (total: " + clients.size() + ")");
            });

            ws.onMessage(ctx -> {
                try {
                    String message = ctx.message();
                    handleMessage(message);
                } catch (Exception e) {
                    System.err.println("Error handling message: " + e.getMessage());
                }
            });

            ws.onError(ctx -> {
                System.err.println("WebSocket error: " + ctx.error());
            });
        });

        // Health check endpoint
        app.get("/health", ctx -> {
            ctx.json(Map.of(
                "status", "ok",
                "running", engine.isRunning(),
                "paused", engine.isPaused(),
                "clients", clients.size(),
                "ticks", engine.getTickCount()
            ));
        });

        // Start the server
        app.start(port);

        // Start broadcast loop
        startBroadcastLoop();

        System.out.println("REBUILT 2026 Simulator Server started at http://localhost:" + port);
    }

    /**
     * Stop the server.
     */
    public void stop() {
        if (broadcastExecutor != null) {
            broadcastExecutor.shutdown();
        }

        if (app != null) {
            app.stop();
        }
    }

    /**
     * Start the periodic state broadcast loop.
     */
    private void startBroadcastLoop() {
        broadcastExecutor = Executors.newSingleThreadScheduledExecutor(r -> {
            Thread t = new Thread(r, "StateBroadcast");
            t.setDaemon(true);
            return t;
        });

        long periodMs = 1000 / Constants.Simulation.BROADCAST_RATE;
        broadcastExecutor.scheduleAtFixedRate(() -> {
            try {
                broadcastState();
            } catch (Exception e) {
                System.err.println("Broadcast error: " + e.getMessage());
            }
        }, 0, periodMs, TimeUnit.MILLISECONDS);
    }

    /**
     * Broadcast current state to all connected clients.
     */
    private void broadcastState() {
        if (clients.isEmpty()) return;

        String json = buildFullStateJson();

        for (WsContext ctx : clients.values()) {
            try {
                ctx.send(json);
            } catch (Exception e) {
                // Client probably disconnected
            }
        }
    }

    /**
     * Send full state to a single client.
     */
    private void sendFullState(WsContext ctx) {
        try {
            ctx.send(buildFullStateJson());
        } catch (Exception e) {
            System.err.println("Error sending state: " + e.getMessage());
        }
    }

    /**
     * Build JSON representation of full game state.
     */
    private String buildFullStateJson() {
        RobotState robotState = engine.getState();
        MatchState matchState = engine.getMatchState();
        FuelState fuelState = engine.getFuelState();

        JsonObject json = new JsonObject();
        json.addProperty("type", "state");

        // Robot state (player's robot for backward compatibility)
        json.add("robot", buildRobotJson(robotState));

        // Swerve modules (player's robot)
        json.add("swerve", buildSwerveJson(robotState));

        // Shooter (player's robot)
        json.add("shooter", buildShooterJson(robotState));

        // Climber (player's robot)
        json.add("climber", buildClimberJson(robotState));

        // Match state
        json.add("match", buildMatchJson(matchState));

        // FUEL on field
        json.add("fuel", buildFuelJson(fuelState));

        // Control state (player's robot)
        json.add("control", buildControlJson(robotState));

        // Multi-robot state
        json.addProperty("multiRobotEnabled", engine.isMultiRobotEnabled());
        if (engine.isMultiRobotEnabled()) {
            json.add("allRobots", buildAllRobotsJson());
        }

        return json.toString();
    }

    /**
     * Build JSON for all robots in multi-robot mode.
     */
    private JsonArray buildAllRobotsJson() {
        JsonArray robotsArray = new JsonArray();
        RobotState[] allRobots = engine.getAllRobots();
        MultiRobotManager robotManager = engine.getRobotManager();

        for (RobotState robot : allRobots) {
            JsonObject robotJson = new JsonObject();
            robotJson.addProperty("id", robot.robotId);
            robotJson.addProperty("teamNumber", robot.teamNumber);
            robotJson.addProperty("alliance", robot.alliance.name());
            robotJson.addProperty("isPlayer", robot.isPlayerControlled);
            robotJson.addProperty("x", round(robot.x, 3));
            robotJson.addProperty("y", round(robot.y, 3));
            robotJson.addProperty("heading", round(Math.toDegrees(robot.heading), 1));
            robotJson.addProperty("vx", round(robot.vx, 2));
            robotJson.addProperty("vy", round(robot.vy, 2));
            robotJson.addProperty("fuelCount", robot.fuelCount);
            robotJson.addProperty("isClimbing", robot.isClimbing);
            robotJson.addProperty("climbLevel", robot.climbLevel);
            robotJson.addProperty("climbComplete", robot.climbComplete);
            robotJson.addProperty("command", robot.currentCommand);

            // Add auto mode name for AI robots
            if (!robot.isPlayerControlled && robotManager != null) {
                AIRobotController aiController = robotManager.getAIController(robot.robotId);
                if (aiController != null) {
                    robotJson.addProperty("autoMode", aiController.getAutoModeName());
                    robotJson.addProperty("teleopBehavior", aiController.getTeleopBehavior().name());
                }
            } else {
                robotJson.addProperty("autoMode", "PLAYER");
                robotJson.addProperty("teleopBehavior", "PLAYER");
            }

            robotsArray.add(robotJson);
        }

        return robotsArray;
    }

    /**
     * Build robot position/velocity JSON.
     */
    private JsonObject buildRobotJson(RobotState state) {
        JsonObject robot = new JsonObject();
        robot.addProperty("x", round(state.x, 3));
        robot.addProperty("y", round(state.y, 3));
        robot.addProperty("heading", round(Math.toDegrees(state.heading), 1));
        robot.addProperty("vx", round(state.vx, 2));
        robot.addProperty("vy", round(state.vy, 2));
        robot.addProperty("omega", round(Math.toDegrees(state.omega), 1));
        robot.addProperty("height", round(state.robotHeight, 3));
        robot.addProperty("onBump", state.onBump);
        robot.addProperty("trenchMode", state.trenchMode);
        robot.addProperty("alliance", state.alliance.name());
        return robot;
    }

    /**
     * Build swerve module JSON.
     */
    private JsonObject buildSwerveJson(RobotState state) {
        JsonObject swerve = new JsonObject();
        String[] moduleNames = {"fl", "fr", "rl", "rr"};
        for (int i = 0; i < 4; i++) {
            JsonObject module = new JsonObject();
            module.addProperty("angle", round(Math.toDegrees(state.moduleAngles[i]), 1));
            module.addProperty("speed", round(state.moduleSpeeds[i], 2));
            swerve.add(moduleNames[i], module);
        }
        return swerve;
    }

    /**
     * Build shooter state JSON.
     */
    private JsonObject buildShooterJson(RobotState state) {
        JsonObject shooter = new JsonObject();
        shooter.addProperty("angle", round(state.shooterAngle, 1));
        shooter.addProperty("angleGoal", round(state.shooterAngleGoal, 1));
        shooter.addProperty("velocity", round(state.shooterVelocity, 1));
        shooter.addProperty("velocityGoal", round(state.shooterVelocityGoal, 1));
        shooter.addProperty("atAngle", state.shooterAtAngle);
        shooter.addProperty("atSpeed", state.shooterAtSpeed);
        shooter.addProperty("spinningUp", state.shooterSpinningUp);
        shooter.addProperty("fuelCount", state.fuelCount);
        shooter.addProperty("intakeState", state.intakeState.name());
        shooter.addProperty("readyToShoot", state.isReadyToShoot());
        return shooter;
    }

    /**
     * Build climber state JSON.
     */
    private JsonObject buildClimberJson(RobotState state) {
        JsonObject climber = new JsonObject();
        climber.addProperty("position", round(state.climberPosition, 3));
        climber.addProperty("velocity", round(state.climberVelocity, 3));
        climber.addProperty("level", state.climbLevel);
        climber.addProperty("isClimbing", state.isClimbing);
        climber.addProperty("complete", state.climbComplete);
        climber.addProperty("points", state.towerPoints);
        return climber;
    }

    /**
     * Build match state JSON.
     */
    private JsonObject buildMatchJson(MatchState state) {
        JsonObject match = new JsonObject();
        match.addProperty("time", round(state.matchTime, 1));
        match.addProperty("remaining", round(state.getRemainingTime(), 1));
        match.addProperty("phase", state.currentPhase.name());
        match.addProperty("phaseName", state.getPhaseName());
        match.addProperty("started", state.matchStarted);
        match.addProperty("ended", state.matchEnded);

        // Auto mode info
        AutonomousController autoController = engine.getAutoController();
        match.addProperty("autoMode", autoController.getSelectedMode());
        match.addProperty("autoModeName", autoController.getSelectedModeName());
        match.addProperty("autoLocked", autoController.isLocked());
        match.addProperty("autoPhase", autoController.getCurrentPhaseName());

        // HUB status
        match.addProperty("redHubActive", state.redHubStatus == MatchState.HubStatus.ACTIVE);
        match.addProperty("blueHubActive", state.blueHubStatus == MatchState.HubStatus.ACTIVE);

        // Scores
        JsonObject scores = new JsonObject();
        scores.addProperty("redTotal", state.redTotalScore);
        scores.addProperty("blueTotal", state.blueTotalScore);
        scores.addProperty("redFuel", state.redFuelScored);
        scores.addProperty("blueFuel", state.blueFuelScored);
        scores.addProperty("redTower", state.redTowerPoints);
        scores.addProperty("blueTower", state.blueTowerPoints);
        match.add("scores", scores);

        // Outpost counts
        JsonObject outpost = new JsonObject();
        outpost.addProperty("redChute", state.redChuteCount);
        outpost.addProperty("blueChute", state.blueChuteCount);
        outpost.addProperty("redCorral", state.redCorralCount);
        outpost.addProperty("blueCorral", state.blueCorralCount);
        match.add("outpost", outpost);

        // Ranking points
        JsonObject rp = new JsonObject();
        rp.addProperty("redEnergized", state.redEnergized);
        rp.addProperty("blueEnergized", state.blueEnergized);
        rp.addProperty("redTraversal", state.redTraversal);
        rp.addProperty("blueTraversal", state.blueTraversal);
        match.add("rankingPoints", rp);

        return match;
    }

    /**
     * Build FUEL state JSON.
     */
    private JsonObject buildFuelJson(FuelState state) {
        JsonObject fuel = new JsonObject();

        // Field FUEL (positions of all active FUEL)
        JsonArray fieldFuel = new JsonArray();
        for (Fuel f : state.getFieldFuel()) {
            JsonObject fuelObj = new JsonObject();
            fuelObj.addProperty("id", f.id);
            fuelObj.addProperty("x", round(f.x, 3));
            fuelObj.addProperty("y", round(f.y, 3));
            fuelObj.addProperty("z", round(f.z, 3));
            fuelObj.addProperty("moving", f.isMoving);
            fieldFuel.add(fuelObj);
        }
        fuel.add("field", fieldFuel);

        // Flight FUEL
        JsonArray flightFuel = new JsonArray();
        for (Fuel f : state.getFlightFuel()) {
            JsonObject fuelObj = new JsonObject();
            fuelObj.addProperty("id", f.id);
            fuelObj.addProperty("x", round(f.x, 3));
            fuelObj.addProperty("y", round(f.y, 3));
            fuelObj.addProperty("z", round(f.z, 3));
            fuelObj.addProperty("vx", round(f.vx, 2));
            fuelObj.addProperty("vy", round(f.vy, 2));
            fuelObj.addProperty("vz", round(f.vz, 2));
            flightFuel.add(fuelObj);
        }
        fuel.add("flight", flightFuel);

        // Counts
        fuel.addProperty("totalOnField", state.getFieldFuel().size());
        fuel.addProperty("totalInFlight", state.getFlightFuel().size());

        return fuel;
    }

    /**
     * Build control state JSON.
     */
    private JsonObject buildControlJson(RobotState state) {
        JsonObject control = new JsonObject();
        control.addProperty("fieldRelative", state.fieldRelative);
        control.addProperty("slowMode", state.slowMode);
        control.addProperty("command", state.currentCommand);
        control.addProperty("enabled", state.isEnabled);
        return control;
    }

    /**
     * Handle incoming WebSocket message.
     */
    private void handleMessage(String message) {
        try {
            JsonObject json = gson.fromJson(message, JsonObject.class);
            String type = json.get("type").getAsString();

            switch (type) {
                case "input":
                    InputState input = parseInput(json);
                    engine.updateInput(input);
                    break;
                case "reset":
                    // Reset all components including multi-robot manager
                    if (engine.isMultiRobotEnabled()) {
                        engine.getRobotManager().reset();
                    } else {
                        engine.getState().reset();
                    }
                    engine.getMatchState().reset();
                    engine.getFuelState().reset();
                    engine.getAutoController().reset();
                    break;
                case "startMatch":
                    engine.startMatch();
                    break;
                case "setAutoMode":
                    // Set the autonomous mode (0-3)
                    if (json.has("mode")) {
                        int mode = json.get("mode").getAsInt();
                        engine.setAutoMode(mode);
                    }
                    break;
                case "addFuel":
                    // Debug: add FUEL to robot
                    engine.getState().addFuel();
                    break;
            }
        } catch (Exception e) {
            System.err.println("Error parsing message: " + e.getMessage());
        }
    }

    /**
     * Parse input state from JSON.
     */
    private InputState parseInput(JsonObject json) {
        InputState input = new InputState();

        // Continuous inputs
        if (json.has("forward")) input.forward = json.get("forward").getAsDouble();
        if (json.has("strafe")) input.strafe = json.get("strafe").getAsDouble();
        if (json.has("turn")) input.turn = json.get("turn").getAsDouble();
        if (json.has("shooterAngle")) input.shooterAngle = json.get("shooterAngle").getAsDouble();
        if (json.has("shooterPower")) input.shooterPower = json.get("shooterPower").getAsDouble();

        // Button inputs
        if (json.has("intake")) input.intake = json.get("intake").getAsBoolean();
        if (json.has("shoot")) input.shoot = json.get("shoot").getAsBoolean();
        if (json.has("spinUp")) input.spinUp = json.get("spinUp").getAsBoolean();

        if (json.has("climberUp")) input.climberUp = json.get("climberUp").getAsBoolean();
        if (json.has("climberDown")) input.climberDown = json.get("climberDown").getAsBoolean();
        if (json.has("level1")) input.level1 = json.get("level1").getAsBoolean();
        if (json.has("level2")) input.level2 = json.get("level2").getAsBoolean();
        if (json.has("level3")) input.level3 = json.get("level3").getAsBoolean();

        if (json.has("toggleTrenchMode")) input.toggleTrenchMode = json.get("toggleTrenchMode").getAsBoolean();
        if (json.has("toggleSpeed")) input.toggleSpeed = json.get("toggleSpeed").getAsBoolean();
        if (json.has("toggleFieldRel")) input.toggleFieldRel = json.get("toggleFieldRel").getAsBoolean();
        if (json.has("resetGyro")) input.resetGyro = json.get("resetGyro").getAsBoolean();
        if (json.has("skiStop")) input.skiStop = json.get("skiStop").getAsBoolean();
        if (json.has("resetRobot")) input.resetRobot = json.get("resetRobot").getAsBoolean();

        // HP inputs
        if (json.has("redChuteRelease")) input.redChuteRelease = json.get("redChuteRelease").getAsBoolean();
        if (json.has("blueChuteRelease")) input.blueChuteRelease = json.get("blueChuteRelease").getAsBoolean();
        if (json.has("redCorralTransfer")) input.redCorralTransfer = json.get("redCorralTransfer").getAsBoolean();
        if (json.has("blueCorralTransfer")) input.blueCorralTransfer = json.get("blueCorralTransfer").getAsBoolean();

        // Match control
        if (json.has("startMatch")) input.startMatch = json.get("startMatch").getAsBoolean();
        if (json.has("pauseMatch")) input.pauseMatch = json.get("pauseMatch").getAsBoolean();

        return input;
    }

    /**
     * Round a double to specified decimal places.
     */
    private double round(double value, int places) {
        double scale = Math.pow(10, places);
        return Math.round(value * scale) / scale;
    }

    /**
     * Get the server port.
     */
    public int getPort() {
        return port;
    }
}
