package frc.robot.subsystems.swerve;

/*
 * ========================================================================
 * SWERVE MODULE - One wheel unit of the swerve drive
 * ========================================================================
 *
 * Each swerve module has:
 *   - A DRIVE motor (TalonFX) that spins the wheel
 *   - An AZIMUTH motor (TalonFX) that steers the wheel
 *   - A CANCoder that knows the absolute wheel angle
 *
 * -> Set desired state: module.setDesiredState(state, openLoop)
 * -> Reset encoder: module.resetToAbsolute()
 *
 * ========================================================================
 */

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

  private final int moduleNumber;
  private final boolean invertDrive;
  private final boolean invertAzimuth;
  private final boolean invertCANcoder;

  private final TalonFX driveMotor;
  private final TalonFX azimuthMotor;
  private final CANcoder canCoder;

  private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0).withSlot(0);
  private final PositionVoltage azimuthPositionRequest = new PositionVoltage(0).withSlot(0);

  private final SimpleMotorFeedforward driveFeedforward;
  private final Rotation2d encoderOffset;

  public SwerveModule(
      int moduleNumber,
      int driveMotorId,
      int azimuthMotorId,
      int canCoderId,
      double encoderOffset) {
    this(moduleNumber, driveMotorId, azimuthMotorId, canCoderId, 
        encoderOffset, 
        false, false, false);
  }

  public SwerveModule(
      int moduleNumber,
      int driveMotorId,
      int azimuthMotorId,
      int canCoderId,
      double encoderOffset,
      boolean invertDrive,
      boolean invertAzimuth,
      boolean invertCANcoder) {
    this.moduleNumber = moduleNumber;
    this.encoderOffset = Rotation2d.fromRotations(encoderOffset);
    this.invertDrive = invertDrive;
    this.invertAzimuth = invertAzimuth;
    this.invertCANcoder = invertCANcoder;

    // Drive motor (TalonFX)
    driveMotor = new TalonFX(driveMotorId);

    // Azimuth motor (TalonFX)
    azimuthMotor = new TalonFX(azimuthMotorId);

    // CANCoder (absolute encoder)
    canCoder = new CANcoder(canCoderId);

    // Feedforward for drive
    driveFeedforward =
        new SimpleMotorFeedforward(
            SwerveConstants.DRIVE_kS,
            SwerveConstants.DRIVE_kV,
            SwerveConstants.DRIVE_kA);

    configureMotors();
    configureCANCoder();
    resetToAbsolute();
  }

  private void configureMotors() {
    // ================================================================
    // DRIVE MOTOR CONFIGURATION
    // ================================================================
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();

    driveConfig.MotorOutput.NeutralMode = SwerveConstants.DRIVE_COAST ? 
      NeutralModeValue.Coast : NeutralModeValue.Brake;

    driveConfig.MotorOutput.Inverted = invertDrive ?
      InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    driveConfig.CurrentLimits.StatorCurrentLimit = SwerveConstants.DRIVE_STATOR_LIMIT;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = SwerveConstants.DRIVE_STATOR_LIMIT_ENABLE;
    driveConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.DRIVE_SUPPLY_LIMIT;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.DRIVE_SUPPLY_LIMIT_ENABLE;

    driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = SwerveConstants.DRIVE_OPEN_LOOP_RAMP;
    driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = SwerveConstants.DRIVE_CLOSED_LOOP_RAMP;

    driveConfig.Slot0.kP = SwerveConstants.DRIVE_kP;
    driveConfig.Slot0.kI = SwerveConstants.DRIVE_kI;
    driveConfig.Slot0.kD = SwerveConstants.DRIVE_kD;

    driveMotor.getConfigurator().apply(driveConfig);

    // ================================================================
    // AZIMUTH MOTOR CONFIGURATION (TalonFX with CANCoder feedback)
    // ================================================================
    TalonFXConfiguration azimuthConfig = new TalonFXConfiguration();

    azimuthConfig.MotorOutput.NeutralMode = SwerveConstants.AZIMUTH_COAST ? 
      NeutralModeValue.Coast : NeutralModeValue.Brake;

    azimuthConfig.MotorOutput.Inverted = invertAzimuth ?
      InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    azimuthConfig.CurrentLimits.StatorCurrentLimit = SwerveConstants.AZIMUTH_STATOR_LIMIT;
    azimuthConfig.CurrentLimits.StatorCurrentLimitEnable = SwerveConstants.AZIMUTH_STATOR_LIMIT_ENABLE;
    azimuthConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.AZIMUTH_SUPPLY_LIMIT;
    azimuthConfig.CurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.AZIMUTH_SUPPLY_LIMIT_ENABLE;

    // Use the remote CANCoder as the feedback sensor for closed-loop
    azimuthConfig.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
    azimuthConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    // Gear ratio between motor and CANCoder (motor rotations per CANCoder rotation)
    azimuthConfig.Feedback.RotorToSensorRatio = SwerveConstants.AZIMUTH_GEAR_RATIO;

    // Enable continuous wrap so the motor takes the shortest path (e.g., 350° → 10° goes +20°)
    azimuthConfig.ClosedLoopGeneral.ContinuousWrap = true;

    azimuthConfig.Slot0.kP = SwerveConstants.AZIMUTH_kP;
    azimuthConfig.Slot0.kI = SwerveConstants.AZIMUTH_kI;
    azimuthConfig.Slot0.kD = SwerveConstants.AZIMUTH_kD;

    azimuthMotor.getConfigurator().apply(azimuthConfig);
  }

  private void configureCANCoder() {
    CANcoderConfiguration CANcoderconfig = new CANcoderConfiguration();
    CANcoderconfig.MagnetSensor.SensorDirection = invertCANcoder ?
      SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;

    // Apply the magnet offset so CANCoder reads 0 when wheel faces forward
    CANcoderconfig.MagnetSensor.MagnetOffset = encoderOffset.getRotations();
    canCoder.getConfigurator().apply(CANcoderconfig);
  }

  /** Sync is not needed — CANCoder is the direct feedback sensor for the azimuth TalonFX. */
  public void resetToAbsolute() {
    // No-op: the TalonFX uses the CANCoder directly via RemoteCANcoder feedback,
    // so there's no relative encoder to reset.
  }

  // ================================================================
  // GETTERS
  // ================================================================

  public Rotation2d getAbsoluteAngle() {
    return Rotation2d.fromRotations(canCoder.getAbsolutePosition().getValueAsDouble());
  }

  /** Raw CANCoder reading in degrees (no offset applied). For calibration only. */
  public double getRawCANCoderDegrees() {
    return canCoder.getAbsolutePosition().getValueAsDouble() * 360.0;
  }

  /** Get the current angle from the azimuth feedback (CANCoder via TalonFX). */
  public Rotation2d getAngle() {
    // TalonFX position with RemoteCANcoder returns rotations
    return Rotation2d.fromRotations(azimuthMotor.getPosition().getValueAsDouble());
  }

  public double getVelocity() {
    return driveMotor.getVelocity().getValueAsDouble()
        * SwerveConstants.WHEEL_CIRCUMFERENCE
        / SwerveConstants.DRIVE_GEAR_RATIO;
  }

  public double getPosition() {
    return driveMotor.getPosition().getValueAsDouble()
        * SwerveConstants.WHEEL_CIRCUMFERENCE
        / SwerveConstants.DRIVE_GEAR_RATIO;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocity(), getAngle());
  }

  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(getPosition(), getAngle());
  }
  public TalonFX GetDriveMotor() {
    return driveMotor;
  }
  public TalonFX GetAzimuthMotor() {
    return azimuthMotor;
  }

  // ================================================================
  // CONTROL
  // ================================================================

  public void setDesiredState(SwerveModuleState desiredState, boolean openLoop) {
    desiredState.optimize(getAngle());

    // Azimuth — position control in rotations (CANCoder units)
    double targetRotations = desiredState.angle.getRotations();
    azimuthMotor.setControl(azimuthPositionRequest.withPosition(targetRotations));

    // Drive
    if (openLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.MAX_SPEED;
      driveMotor.set(percentOutput);
    } else {
      double targetRPS = desiredState.speedMetersPerSecond
          * SwerveConstants.DRIVE_GEAR_RATIO
          / SwerveConstants.WHEEL_CIRCUMFERENCE;
      double ffVolts = driveFeedforward.calculate(desiredState.speedMetersPerSecond);
      driveMotor.setControl(driveVelocityRequest.withVelocity(targetRPS).withFeedForward(ffVolts));
    }
  }

  public void stop() {
    driveMotor.set(0);
    azimuthMotor.set(0);
  }

  public int getModuleNumber() {
    return moduleNumber;
  }
}
