package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * Intake subsystem for collecting FUEL game pieces.
 *
 * Features:
 * - Deploy/retract mechanism to lower/raise intake
 * - Roller system to pull in FUEL
 */
public class Intake extends SubsystemBase {

    // ================================================================
    // HARDWARE
    // ================================================================

    /** Motor that deploys/retracts the intake mechanism */
    private final SparkMax deployMotor;

    /** Motor that spins the rollers to intake/outtake FUEL */
    private final SparkMax rollerMotor;

    /** Deploy motor encoder for position feedback */
    private final RelativeEncoder deployEncoder;

    /** PID controller for deploy position */
    private final SparkClosedLoopController deployController;

    // ================================================================
    // STATE TRACKING
    // ================================================================

    /** Current state of the intake */
    private IntakeState state = IntakeState.STOWED;

    /** Target position for deploy mechanism */
    private double targetPosition = IntakeConstants.STOWED_POSITION;

    // ================================================================
    // STATE ENUM
    // ================================================================

    public enum IntakeState {
        STOWED,
        DEPLOYING,
        DEPLOYED,
        RETRACTING,
        INTAKING,
        OUTTAKING,
        FEEDING
    }

    // ================================================================
    // CONSTRUCTOR
    // ================================================================

    public Intake() {
        // Initialize deploy motor
        deployMotor = new SparkMax(IntakeConstants.DEPLOY_MOTOR_ID, MotorType.kBrushless);
        deployEncoder = deployMotor.getEncoder();
        deployController = deployMotor.getClosedLoopController();

        // Initialize roller motor
        rollerMotor = new SparkMax(IntakeConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);

        configureMotors();
    }

    // ================================================================
    // CONFIGURATION
    // ================================================================

    private void configureMotors() {
        // Deploy motor configuration
        SparkMaxConfig deployConfig = new SparkMaxConfig();
        deployConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30);

        deployConfig.closedLoop
            .p(IntakeConstants.DEPLOY_kP)
            .i(0)
            .d(0);

        deployMotor.configure(deployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Roller motor configuration
        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40);

        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Reset encoder to zero (assumes starting stowed)
        deployEncoder.setPosition(0);
    }

    // ================================================================
    // DEPLOY/RETRACT METHODS
    // ================================================================

    /** Deploy the intake to collect FUEL */
    public void deploy() {
        targetPosition = IntakeConstants.DEPLOYED_POSITION;
        deployController.setReference(targetPosition, SparkMax.ControlType.kPosition);
        state = IntakeState.DEPLOYING;
    }

    /** Retract the intake to stowed position */
    public void retract() {
        stopRollers();
        targetPosition = IntakeConstants.STOWED_POSITION;
        deployController.setReference(targetPosition, SparkMax.ControlType.kPosition);
        state = IntakeState.RETRACTING;
    }

    /** Check if intake is at deployed position */
    public boolean isDeployed() {
        return Math.abs(deployEncoder.getPosition() - IntakeConstants.DEPLOYED_POSITION)
               < IntakeConstants.POSITION_TOLERANCE;
    }

    /** Check if intake is at stowed position */
    public boolean isStowed() {
        return Math.abs(deployEncoder.getPosition() - IntakeConstants.STOWED_POSITION)
               < IntakeConstants.POSITION_TOLERANCE;
    }

    // ================================================================
    // ROLLER METHODS
    // ================================================================

    /** Run intake rollers to collect FUEL */
    public void runIntake() {
        rollerMotor.set(IntakeConstants.INTAKE_SPEED);
        if (isDeployed()) {
            state = IntakeState.INTAKING;
        }
    }

    /** Reverse rollers to eject FUEL */
    public void runOuttake() {
        rollerMotor.set(IntakeConstants.OUTTAKE_SPEED);
        state = IntakeState.OUTTAKING;
    }

    /** Feed FUEL to the shooter mechanism */
    public void feedToShooter() {
        rollerMotor.set(IntakeConstants.INTAKE_SPEED);
        state = IntakeState.FEEDING;
    }

    /** Stop the roller motors */
    public void stopRollers() {
        rollerMotor.set(0);
        if (isDeployed()) {
            state = IntakeState.DEPLOYED;
        } else if (isStowed()) {
            state = IntakeState.STOWED;
        }
    }

    // ================================================================
    // COMMAND FACTORIES
    // ================================================================

    /** Command to deploy intake and run rollers */
    public Command intakeCommand() {
        return Commands.sequence(
            // Deploy the intake
            Commands.runOnce(this::deploy, this),
            Commands.waitUntil(this::isDeployed),
            // Run rollers
            Commands.runOnce(this::runIntake, this)
        ).withName("Intake FUEL");
    }

    /** Command to retract intake to stowed position */
    public Command retractCommand() {
        return Commands.sequence(
            Commands.runOnce(this::retract, this),
            Commands.waitUntil(this::isStowed)
        ).withName("Retract Intake");
    }

    /** Command to run outtake while button held */
    public Command outtakeCommand() {
        return Commands.startEnd(
            this::runOuttake,
            this::stopRollers,
            this
        ).withName("Outtake FUEL");
    }

    /** Command to deploy and continuously intake (hold to run) */
    public Command holdToIntakeCommand() {
        return Commands.sequence(
            Commands.runOnce(this::deploy, this),
            Commands.waitUntil(this::isDeployed),
            Commands.run(this::runIntake, this)
        ).finallyDo(interrupted -> {
            stopRollers();
            retract();
        }).withName("Hold to Intake");
    }

    // ================================================================
    // GETTERS
    // ================================================================

    /** Get current intake state */
    public IntakeState getState() {
        return state;
    }

    /** Get current deploy position */
    public double getDeployPosition() {
        return deployEncoder.getPosition();
    }

    // ================================================================
    // PERIODIC
    // ================================================================

    @Override
    public void periodic() {
        // Update state based on position
        if (state == IntakeState.DEPLOYING && isDeployed()) {
            state = IntakeState.DEPLOYED;
        } else if (state == IntakeState.RETRACTING && isStowed()) {
            state = IntakeState.STOWED;
        }

        // Update SmartDashboard
        SmartDashboard.putString("Intake/State", state.toString());
        SmartDashboard.putNumber("Intake/Position", deployEncoder.getPosition());
        SmartDashboard.putBoolean("Intake/IsDeployed", isDeployed());
        SmartDashboard.putBoolean("Intake/IsStowed", isStowed());
    }
}
