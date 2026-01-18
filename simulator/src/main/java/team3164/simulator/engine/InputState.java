package team3164.simulator.engine;

/**
 * Current state of all user inputs for REBUILT 2026.
 * Updated from keyboard/gamepad via WebSocket.
 */
public class InputState {

    // ========================================================================
    // DRIVE INPUTS (continuous, -1 to 1)
    // ========================================================================
    public double forward;    // W/S or left stick Y
    public double strafe;     // A/D or left stick X
    public double turn;       // Q/E or right stick X

    // ========================================================================
    // SHOOTER INPUTS (continuous, 0 to 1)
    // ========================================================================
    public double shooterAngle;   // Shooter angle control (0 = flat, 1 = steep)
    public double shooterPower;   // Shooter power/velocity (0 = min, 1 = max)

    // ========================================================================
    // BUTTON INPUTS (discrete, true/false)
    // ========================================================================
    // Intake/Shooter controls
    public boolean intake;        // Intake FUEL (Space)
    public boolean shoot;         // Fire FUEL (Shift)
    public boolean spinUp;        // Spin up shooter wheels (F)

    // Climber controls
    public boolean climberUp;     // Extend climber (Up arrow)
    public boolean climberDown;   // Retract climber (Down arrow)
    public boolean level1;        // Target LOW rung (1 key)
    public boolean level2;        // Target MID rung (2 key)
    public boolean level3;        // Target HIGH rung (3 key)

    // Robot configuration
    public boolean toggleTrenchMode;  // Toggle trench configuration (T key)

    // Mode toggles
    public boolean toggleSpeed;       // Toggle slow mode (X key)
    public boolean toggleFieldRel;    // Toggle field relative (C key)
    public boolean resetGyro;         // Reset gyro heading (G key)
    public boolean skiStop;           // Ski stop maneuver (V key)
    public boolean resetRobot;        // Reset robot state (Escape key)

    // ========================================================================
    // HUMAN PLAYER CONTROLS
    // ========================================================================
    public boolean redChuteRelease;     // Release FUEL from red CHUTE (Q key)
    public boolean blueChuteRelease;    // Release FUEL from blue CHUTE (P key)
    public boolean redCorralTransfer;   // Transfer red CORRAL to CHUTE (W key when holding modifier)
    public boolean blueCorralTransfer;  // Transfer blue CORRAL to CHUTE (O key when holding modifier)

    // ========================================================================
    // MATCH CONTROL
    // ========================================================================
    public boolean startMatch;          // Start the match (Enter key)
    public boolean pauseMatch;          // Pause/unpause match (Space when modifier held)

    /**
     * Reset all inputs to neutral.
     */
    public void reset() {
        forward = 0;
        strafe = 0;
        turn = 0;

        shooterAngle = 0;
        shooterPower = 0;

        intake = false;
        shoot = false;
        spinUp = false;

        climberUp = false;
        climberDown = false;
        level1 = false;
        level2 = false;
        level3 = false;

        toggleTrenchMode = false;

        toggleSpeed = false;
        toggleFieldRel = false;
        resetGyro = false;
        skiStop = false;
        resetRobot = false;

        redChuteRelease = false;
        blueChuteRelease = false;
        redCorralTransfer = false;
        blueCorralTransfer = false;

        startMatch = false;
        pauseMatch = false;
    }

    /**
     * Check if any level button is pressed.
     */
    public int getRequestedClimbLevel() {
        if (level3) return 3;
        if (level2) return 2;
        if (level1) return 1;
        return 0;  // No level requested
    }

    /**
     * Check if any drive input is active.
     */
    public boolean hasDriveInput() {
        return Math.abs(forward) > 0.05 ||
               Math.abs(strafe) > 0.05 ||
               Math.abs(turn) > 0.05;
    }

    /**
     * Check if any shooter input is active.
     */
    public boolean hasShooterInput() {
        return shooterAngle > 0.05 || shooterPower > 0.05 || spinUp;
    }

    /**
     * Check if any HP input is active.
     */
    public boolean hasHPInput() {
        return redChuteRelease || blueChuteRelease ||
               redCorralTransfer || blueCorralTransfer;
    }

    @Override
    public String toString() {
        return String.format("Input[fwd=%.2f, str=%.2f, turn=%.2f, angle=%.2f, pwr=%.2f]",
            forward, strafe, turn, shooterAngle, shooterPower);
    }
}
