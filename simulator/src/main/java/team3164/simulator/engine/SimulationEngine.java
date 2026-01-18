package team3164.simulator.engine;

import team3164.simulator.Constants;
import team3164.simulator.physics.*;
import team3164.simulator.engine.FuelState.Fuel;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;

/**
 * Core simulation engine for REBUILT 2026.
 *
 * Runs at 50Hz (20ms per tick) to match WPILib robot code timing.
 * Updates all subsystem physics and broadcasts state changes.
 */
public class SimulationEngine {

    private final RobotState state;
    private final InputState input;
    private final MatchState matchState;
    private final FuelState fuelState;

    private ScheduledExecutorService executor;
    private Consumer<RobotState> stateListener;
    private Consumer<MatchState> matchStateListener;
    private Consumer<FuelState> fuelStateListener;
    private Consumer<String> logListener;

    private boolean running = false;
    private boolean paused = false;
    private long tickCount = 0;
    private long startTimeMs;

    /**
     * Create a new simulation engine.
     */
    public SimulationEngine() {
        this.state = new RobotState();
        this.input = new InputState();
        this.matchState = new MatchState();
        this.fuelState = new FuelState();
    }

    /**
     * Start the simulation loop.
     */
    public void start() {
        if (running) return;

        running = true;
        startTimeMs = System.currentTimeMillis();
        tickCount = 0;

        // Initialize FUEL for the match
        fuelState.reset();

        executor = Executors.newSingleThreadScheduledExecutor(r -> {
            Thread t = new Thread(r, "SimulationEngine");
            t.setDaemon(true);
            return t;
        });

        long periodMs = (long) (1000.0 / Constants.Simulation.TICK_RATE);
        executor.scheduleAtFixedRate(this::tick, 0, periodMs, TimeUnit.MILLISECONDS);

        log("Simulation started - REBUILT 2026");
    }

    /**
     * Stop the simulation loop.
     */
    public void stop() {
        if (!running) return;

        running = false;
        if (executor != null) {
            executor.shutdown();
            try {
                executor.awaitTermination(1, TimeUnit.SECONDS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        log("Simulation stopped");
    }

    /**
     * Start a new match.
     */
    public void startMatch() {
        if (matchState.matchStarted) return;

        matchState.startMatch();
        startTimeMs = System.currentTimeMillis();
        log("MATCH STARTED - AUTO PERIOD");
    }

    /**
     * Single simulation tick - called at 50Hz.
     */
    private void tick() {
        try {
            if (paused) return;

            double dt = Constants.Simulation.DT;
            tickCount++;

            // Update match time
            if (matchState.matchStarted && !matchState.matchEnded) {
                matchState.matchTime = (System.currentTimeMillis() - startTimeMs) / 1000.0;
                matchState.updatePhase();

                // Log phase changes
                logPhaseChange();
            }

            // Process mode toggles
            processToggles();

            // Process match control
            processMatchControl();

            // Update all physics
            updatePhysics(dt);

            // Update command name based on current actions
            updateCommandName();

            // Notify listeners
            notifyListeners();

        } catch (Exception e) {
            log("Error in simulation tick: " + e.getMessage());
            e.printStackTrace();
        }
    }

    /**
     * Update all physics systems.
     */
    private void updatePhysics(double dt) {
        // Update swerve drive
        SwervePhysics.update(state, input, dt);

        // Update bump physics (affects robot height)
        BumpPhysics.update(state, dt);

        // Check trench traversal
        if (!TrenchPhysics.update(state, dt)) {
            // Robot blocked by trench - push back
            double[] pushOut = TrenchPhysics.getPushOutVector(state.x, state.y);
            if (pushOut != null) {
                state.x += pushOut[0];
                state.y += pushOut[1];
                state.vx = 0;
                state.vy = 0;
            }
        }

        // Update shooter physics
        Fuel launchedFuel = ShooterPhysics.update(state, input, fuelState, matchState, dt);
        if (launchedFuel != null) {
            log("FUEL launched at " + String.format("%.1f m/s, %.0f°",
                state.shooterVelocity, state.shooterAngle));
        }

        // Update intake physics
        if (IntakePhysics.update(state, input, fuelState, dt)) {
            log("FUEL collected! (" + state.fuelCount + "/" + Constants.Intake.MAX_CAPACITY + ")");
        }

        // Update climber/tower physics
        TowerPhysics.update(state, input, matchState, dt);

        // Update FUEL physics (all balls on field and in flight)
        FuelPhysics.update(fuelState, matchState, dt);

        // Update outpost physics (HP controls)
        OutpostPhysics.update(fuelState, matchState, input, dt);

        // Check robot collision with FUEL
        for (Fuel fuel : fuelState.getFieldFuel()) {
            FuelPhysics.checkRobotCollision(fuel, state);
        }
    }

    /**
     * Process toggle button inputs.
     */
    private void processToggles() {
        if (input.resetGyro) {
            SwervePhysics.resetGyro(state, 180);
            input.resetGyro = false;
            log("Gyro reset to 180°");
        }

        if (input.toggleFieldRel) {
            state.fieldRelative = !state.fieldRelative;
            input.toggleFieldRel = false;
            log("Field-relative: " + (state.fieldRelative ? "ON" : "OFF"));
        }

        if (input.toggleSpeed) {
            state.slowMode = !state.slowMode;
            input.toggleSpeed = false;
            log("Slow mode: " + (state.slowMode ? "ON" : "OFF"));
        }

        if (input.toggleTrenchMode) {
            TrenchPhysics.toggleTrenchMode(state);
            input.toggleTrenchMode = false;
            log("Trench mode: " + (state.trenchMode ? "ON" : "OFF"));
        }

        if (input.resetRobot) {
            state.reset();
            matchState.reset();
            fuelState.reset();
            input.resetRobot = false;
            log("Simulation reset");
        }
    }

    /**
     * Process match control inputs.
     */
    private void processMatchControl() {
        if (input.startMatch && !matchState.matchStarted) {
            startMatch();
            input.startMatch = false;
        }

        if (input.pauseMatch) {
            paused = !paused;
            input.pauseMatch = false;
            log(paused ? "PAUSED" : "RESUMED");
        }
    }

    private MatchState.MatchPhase lastPhase = null;

    /**
     * Log match phase changes.
     */
    private void logPhaseChange() {
        if (matchState.currentPhase != lastPhase) {
            lastPhase = matchState.currentPhase;

            switch (matchState.currentPhase) {
                case AUTO:
                    log("=== AUTO PERIOD ===");
                    break;
                case TRANSITION:
                    log("=== TRANSITION - " +
                        (matchState.autoWinner != null ? matchState.autoWinner + " wins AUTO" : "AUTO TIE") +
                        " ===");
                    break;
                case SHIFT_1:
                    log("=== SHIFT 1 - " + getActiveHubString() + " HUB ACTIVE ===");
                    break;
                case SHIFT_2:
                    log("=== SHIFT 2 - " + getActiveHubString() + " HUB ACTIVE ===");
                    break;
                case SHIFT_3:
                    log("=== SHIFT 3 - " + getActiveHubString() + " HUB ACTIVE ===");
                    break;
                case SHIFT_4:
                    log("=== SHIFT 4 - " + getActiveHubString() + " HUB ACTIVE ===");
                    break;
                case END_GAME:
                    log("=== END GAME - BOTH HUBS ACTIVE ===");
                    break;
                case POST_MATCH:
                    log("=== MATCH COMPLETE ===");
                    logFinalScore();
                    break;
            }
        }
    }

    /**
     * Get string describing which HUB is active.
     */
    private String getActiveHubString() {
        if (matchState.redHubStatus == MatchState.HubStatus.ACTIVE &&
            matchState.blueHubStatus == MatchState.HubStatus.ACTIVE) {
            return "BOTH";
        } else if (matchState.redHubStatus == MatchState.HubStatus.ACTIVE) {
            return "RED";
        } else if (matchState.blueHubStatus == MatchState.HubStatus.ACTIVE) {
            return "BLUE";
        }
        return "NONE";
    }

    /**
     * Log final match score.
     */
    private void logFinalScore() {
        log("Final Score:");
        log("  RED: " + matchState.redTotalScore + " pts " +
            "(FUEL: " + matchState.redFuelScored + ", TOWER: " + matchState.redTowerPoints + ")");
        log("  BLUE: " + matchState.blueTotalScore + " pts " +
            "(FUEL: " + matchState.blueFuelScored + ", TOWER: " + matchState.blueTowerPoints + ")");

        if (matchState.redEnergized) log("  RED ENERGIZED RP!");
        if (matchState.blueEnergized) log("  BLUE ENERGIZED RP!");
        if (matchState.redTraversal) log("  RED TRAVERSAL RP!");
        if (matchState.blueTraversal) log("  BLUE TRAVERSAL RP!");
    }

    /**
     * Update the current command name for display.
     */
    private void updateCommandName() {
        if (state.isClimbing) {
            state.currentCommand = "Climbing L" + state.climbLevel;
        } else if (state.climbComplete) {
            state.currentCommand = "L" + state.climbLevel + " Complete!";
        } else if (state.intakeState == RobotState.IntakeState.INTAKING) {
            state.currentCommand = "Intaking";
        } else if (state.intakeState == RobotState.IntakeState.SHOOTING) {
            state.currentCommand = "Shooting";
        } else if (state.intakeState == RobotState.IntakeState.READY_TO_SHOOT) {
            state.currentCommand = "Ready to Shoot";
        } else if (state.shooterSpinningUp) {
            state.currentCommand = "Spinning Up";
        } else if (state.fuelCount > 0) {
            state.currentCommand = "Holding " + state.fuelCount + " FUEL";
        } else if (input.hasDriveInput()) {
            state.currentCommand = "Driving";
        } else {
            state.currentCommand = "Idle";
        }
    }

    /**
     * Notify all listeners of state updates.
     */
    private void notifyListeners() {
        if (stateListener != null) {
            stateListener.accept(state);
        }
        if (matchStateListener != null) {
            matchStateListener.accept(matchState);
        }
        if (fuelStateListener != null) {
            fuelStateListener.accept(fuelState);
        }
    }

    /**
     * Update input state from external source (WebSocket).
     */
    public void updateInput(InputState newInput) {
        synchronized (input) {
            // Continuous inputs
            input.forward = newInput.forward;
            input.strafe = newInput.strafe;
            input.turn = newInput.turn;
            input.shooterAngle = newInput.shooterAngle;
            input.shooterPower = newInput.shooterPower;

            // Button inputs
            input.intake = newInput.intake;
            input.shoot = newInput.shoot;
            input.spinUp = newInput.spinUp;

            input.climberUp = newInput.climberUp;
            input.climberDown = newInput.climberDown;
            input.level1 = newInput.level1;
            input.level2 = newInput.level2;
            input.level3 = newInput.level3;

            input.skiStop = newInput.skiStop;

            // HP inputs
            input.redChuteRelease = newInput.redChuteRelease;
            input.blueChuteRelease = newInput.blueChuteRelease;
            input.redCorralTransfer = newInput.redCorralTransfer;
            input.blueCorralTransfer = newInput.blueCorralTransfer;

            // Edge-triggered inputs
            if (newInput.toggleSpeed) input.toggleSpeed = true;
            if (newInput.toggleFieldRel) input.toggleFieldRel = true;
            if (newInput.toggleTrenchMode) input.toggleTrenchMode = true;
            if (newInput.resetGyro) input.resetGyro = true;
            if (newInput.resetRobot) input.resetRobot = true;
            if (newInput.startMatch) input.startMatch = true;
            if (newInput.pauseMatch) input.pauseMatch = true;
        }
    }

    /**
     * Get current robot state (for broadcasting).
     */
    public RobotState getState() {
        return state;
    }

    /**
     * Get current match state.
     */
    public MatchState getMatchState() {
        return matchState;
    }

    /**
     * Get current FUEL state.
     */
    public FuelState getFuelState() {
        return fuelState;
    }

    /**
     * Set listener for robot state updates.
     */
    public void setStateListener(Consumer<RobotState> listener) {
        this.stateListener = listener;
    }

    /**
     * Set listener for match state updates.
     */
    public void setMatchStateListener(Consumer<MatchState> listener) {
        this.matchStateListener = listener;
    }

    /**
     * Set listener for FUEL state updates.
     */
    public void setFuelStateListener(Consumer<FuelState> listener) {
        this.fuelStateListener = listener;
    }

    /**
     * Set listener for log messages.
     */
    public void setLogListener(Consumer<String> listener) {
        this.logListener = listener;
    }

    /**
     * Log a message to console and listeners.
     */
    private void log(String message) {
        String timestamp;
        if (matchState.matchStarted) {
            timestamp = "[" + matchState.getFormattedTime() + "]";
        } else {
            timestamp = String.format("[%02d:%02d]",
                (int)(tickCount / 50 / 60) % 60,
                (int)(tickCount / 50) % 60);
        }

        String fullMessage = timestamp + " " + message;
        System.out.println(fullMessage);

        if (logListener != null) {
            logListener.accept(fullMessage);
        }
    }

    /**
     * Check if simulation is running.
     */
    public boolean isRunning() {
        return running;
    }

    /**
     * Check if simulation is paused.
     */
    public boolean isPaused() {
        return paused;
    }

    /**
     * Get tick count for diagnostics.
     */
    public long getTickCount() {
        return tickCount;
    }
}
