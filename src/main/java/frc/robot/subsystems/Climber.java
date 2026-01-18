package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

/**
 * Climber subsystem for climbing to different rung levels.
 *
 * Supports three climb levels:
 * - L1: Low rung (27" / 0.686m)
 * - L2: Mid rung (45" / 1.143m)
 * - L3: High rung (63" / 1.60m)
 *
 * NOTE: Motor control is stubbed - actual hardware control TBD.
 * Currently uses simulated position for testing command flow.
 */
public class Climber extends SubsystemBase {

    // ================================================================
    // HARDWARE (STUBBED)
    // ================================================================

    // TODO: Add motor controller when hardware is finalized
    // private final SparkMax climberMotor;
    // private final RelativeEncoder climberEncoder;

    // ================================================================
    // STATE TRACKING
    // ================================================================

    /** Current state of the climber */
    private ClimberState state = ClimberState.STOWED;

    /** Target height for climb (meters) */
    private double targetHeight = ClimberConstants.STOWED_HEIGHT;

    /** Simulated current position (for testing without hardware) */
    private double simulatedPosition = 0.0;

    /** Speed for simulated movement (m/s) */
    private static final double SIMULATED_SPEED = 0.5;

    // ================================================================
    // STATE ENUM
    // ================================================================

    public enum ClimberState {
        STOWED,
        EXTENDING,
        AT_L1,
        AT_L2,
        AT_L3,
        RETRACTING,
        MANUAL
    }

    // ================================================================
    // CONSTRUCTOR
    // ================================================================

    public Climber() {
        // TODO: Initialize motor controller when hardware is finalized
        // climberMotor = new SparkMax(ClimberConstants.CLIMBER_MOTOR_ID, MotorType.kBrushless);
        // climberEncoder = climberMotor.getEncoder();
        // configureMotor();
    }

    // ================================================================
    // CONFIGURATION (STUBBED)
    // ================================================================

    @SuppressWarnings("unused")
    private void configureMotor() {
        // TODO: Configure motor when hardware is finalized
        // SparkMaxConfig config = new SparkMaxConfig();
        // config.idleMode(IdleMode.kBrake);
        // config.smartCurrentLimit(60);
        // climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // ================================================================
    // CLIMB LEVEL METHODS
    // ================================================================

    /** Initiate climb to Level 1 (Low Rung) */
    public void climbToL1() {
        targetHeight = ClimberConstants.L1_HEIGHT;
        state = ClimberState.EXTENDING;
        // TODO: Set motor to extend to L1 height
        // climberController.setReference(targetHeight, SparkMax.ControlType.kPosition);
    }

    /** Initiate climb to Level 2 (Mid Rung) */
    public void climbToL2() {
        targetHeight = ClimberConstants.L2_HEIGHT;
        state = ClimberState.EXTENDING;
        // TODO: Set motor to extend to L2 height
        // climberController.setReference(targetHeight, SparkMax.ControlType.kPosition);
    }

    /** Initiate climb to Level 3 (High Rung) */
    public void climbToL3() {
        targetHeight = ClimberConstants.L3_HEIGHT;
        state = ClimberState.EXTENDING;
        // TODO: Set motor to extend to L3 height
        // climberController.setReference(targetHeight, SparkMax.ControlType.kPosition);
    }

    /** Retract climber to stowed position */
    public void stow() {
        targetHeight = ClimberConstants.STOWED_HEIGHT;
        state = ClimberState.RETRACTING;
        // TODO: Set motor to retract
        // climberController.setReference(targetHeight, SparkMax.ControlType.kPosition);
    }

    // ================================================================
    // POSITION METHODS
    // ================================================================

    /** Get current climber height (meters) */
    public double getPosition() {
        // TODO: Return actual encoder position when hardware is available
        // return climberEncoder.getPosition();
        return simulatedPosition;
    }

    /** Check if climber is at target level */
    public boolean isAtTargetLevel() {
        return Math.abs(getPosition() - targetHeight) < ClimberConstants.POSITION_TOLERANCE;
    }

    /** Check if climber is stowed */
    public boolean isStowed() {
        return Math.abs(getPosition() - ClimberConstants.STOWED_HEIGHT)
               < ClimberConstants.POSITION_TOLERANCE;
    }

    /** Check if at L1 height */
    public boolean isAtL1() {
        return Math.abs(getPosition() - ClimberConstants.L1_HEIGHT)
               < ClimberConstants.POSITION_TOLERANCE;
    }

    /** Check if at L2 height */
    public boolean isAtL2() {
        return Math.abs(getPosition() - ClimberConstants.L2_HEIGHT)
               < ClimberConstants.POSITION_TOLERANCE;
    }

    /** Check if at L3 height */
    public boolean isAtL3() {
        return Math.abs(getPosition() - ClimberConstants.L3_HEIGHT)
               < ClimberConstants.POSITION_TOLERANCE;
    }

    // ================================================================
    // MANUAL CONTROL (FOR TESTING)
    // ================================================================

    /** Manually extend the climber */
    public void manualExtend() {
        state = ClimberState.MANUAL;
        // TODO: Set motor to extend at fixed speed
        // climberMotor.set(0.5);
    }

    /** Manually retract the climber */
    public void manualRetract() {
        state = ClimberState.MANUAL;
        // TODO: Set motor to retract at fixed speed
        // climberMotor.set(-0.5);
    }

    /** Stop the climber motor */
    public void stop() {
        // TODO: Stop the motor
        // climberMotor.set(0);
        if (isAtL1()) {
            state = ClimberState.AT_L1;
        } else if (isAtL2()) {
            state = ClimberState.AT_L2;
        } else if (isAtL3()) {
            state = ClimberState.AT_L3;
        } else if (isStowed()) {
            state = ClimberState.STOWED;
        }
    }

    // ================================================================
    // COMMAND FACTORIES
    // ================================================================

    /** Command to climb to L1 and wait until reached */
    public Command climbToL1Command() {
        return Commands.sequence(
            Commands.runOnce(this::climbToL1, this),
            Commands.waitUntil(this::isAtL1),
            Commands.runOnce(() -> state = ClimberState.AT_L1)
        ).withName("Climb to L1");
    }

    /** Command to climb to L2 and wait until reached */
    public Command climbToL2Command() {
        return Commands.sequence(
            Commands.runOnce(this::climbToL2, this),
            Commands.waitUntil(this::isAtL2),
            Commands.runOnce(() -> state = ClimberState.AT_L2)
        ).withName("Climb to L2");
    }

    /** Command to climb to L3 and wait until reached */
    public Command climbToL3Command() {
        return Commands.sequence(
            Commands.runOnce(this::climbToL3, this),
            Commands.waitUntil(this::isAtL3),
            Commands.runOnce(() -> state = ClimberState.AT_L3)
        ).withName("Climb to L3");
    }

    /** Command to stow climber and wait until stowed */
    public Command stowCommand() {
        return Commands.sequence(
            Commands.runOnce(this::stow, this),
            Commands.waitUntil(this::isStowed),
            Commands.runOnce(() -> state = ClimberState.STOWED)
        ).withName("Stow Climber");
    }

    /** Command to manually extend while button held */
    public Command manualExtendCommand() {
        return Commands.startEnd(
            this::manualExtend,
            this::stop,
            this
        ).withName("Manual Extend");
    }

    /** Command to manually retract while button held */
    public Command manualRetractCommand() {
        return Commands.startEnd(
            this::manualRetract,
            this::stop,
            this
        ).withName("Manual Retract");
    }

    // ================================================================
    // GETTERS
    // ================================================================

    /** Get current climber state */
    public ClimberState getState() {
        return state;
    }

    /** Get target height */
    public double getTargetHeight() {
        return targetHeight;
    }

    // ================================================================
    // PERIODIC
    // ================================================================

    @Override
    public void periodic() {
        // Simulate movement toward target for testing
        double error = targetHeight - simulatedPosition;
        if (Math.abs(error) > ClimberConstants.POSITION_TOLERANCE) {
            double movement = Math.signum(error) * SIMULATED_SPEED * 0.02; // 20ms loop
            simulatedPosition += movement;
            // Clamp position
            simulatedPosition = Math.max(0, Math.min(simulatedPosition, ClimberConstants.L3_HEIGHT + 0.1));
        }

        // Update state when target reached
        if (state == ClimberState.EXTENDING && isAtTargetLevel()) {
            if (isAtL1()) state = ClimberState.AT_L1;
            else if (isAtL2()) state = ClimberState.AT_L2;
            else if (isAtL3()) state = ClimberState.AT_L3;
        } else if (state == ClimberState.RETRACTING && isStowed()) {
            state = ClimberState.STOWED;
        }

        // Update SmartDashboard
        SmartDashboard.putString("Climber/State", state.toString());
        SmartDashboard.putNumber("Climber/Position", getPosition());
        SmartDashboard.putNumber("Climber/TargetHeight", targetHeight);
        SmartDashboard.putBoolean("Climber/AtTarget", isAtTargetLevel());
        SmartDashboard.putBoolean("Climber/IsStowed", isStowed());
    }
}
