package frc.robot.subsystems.swerve;

/*
 * ========================================================================
 * SWERVE MODULE - One wheel unit of the swerve drive
 * ========================================================================
 *
 * Each swerve module has:
 *   - A DRIVE motor (NEO on SparkMax) that spins the wheel
 *   - An AZIMUTH motor (NEO on SparkMax) that steers the wheel
 *
 * There is NO absolute encoder (no CANcoder). Azimuth position comes from
 * the NEO's built-in relative encoder, which is zeroed to "wheel pointing
 * forward" the moment this class is constructed — see the AZIMUTH ZEROING
 * note in SwerveConstants for the physical setup this requires on boot.
 *
 * -> Set desired state: module.setDesiredState(state, openLoop)
 * -> Re-zero azimuth (wheel must be pointed forward first): module.resetToForward()
 *
 * ========================================================================
 */

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

  private final int moduleNumber;

  private final SparkMax driveMotor;
  private final SparkMax azimuthMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder azimuthEncoder;

  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController azimuthController;

  private final SimpleMotorFeedforward driveFeedforward;

  public SwerveModule(
      int moduleNumber,
      int driveMotorId,
      int azimuthMotorId) {
    this(moduleNumber, driveMotorId, azimuthMotorId, false, false);
  }

  public SwerveModule(
      int moduleNumber,
      int driveMotorId,
      int azimuthMotorId,
      boolean invertDrive,
      boolean invertAzimuth) {
    this.moduleNumber = moduleNumber;

    driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
    azimuthMotor = new SparkMax(azimuthMotorId, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    azimuthEncoder = azimuthMotor.getEncoder();

    driveController = driveMotor.getClosedLoopController();
    azimuthController = azimuthMotor.getClosedLoopController();

    driveFeedforward =
        new SimpleMotorFeedforward(
            SwerveConstants.DRIVE_kS,
            SwerveConstants.DRIVE_kV,
            SwerveConstants.DRIVE_kA);

    configureDriveMotor(invertDrive);
    configureAzimuthMotor(invertAzimuth);

    resetToForward();
  }

  private void configureDriveMotor(boolean invertDrive) {
    SparkMaxConfig config = new SparkMaxConfig();

    config.inverted(invertDrive);
    config.idleMode(SwerveConstants.DRIVE_COAST ? IdleMode.kCoast : IdleMode.kBrake);
    config.smartCurrentLimit(SwerveConstants.DRIVE_CURRENT_LIMIT);
    config.openLoopRampRate(SwerveConstants.DRIVE_OPEN_LOOP_RAMP);
    config.closedLoopRampRate(SwerveConstants.DRIVE_CLOSED_LOOP_RAMP);

    // Convert motor rotations/RPM straight to wheel meters / meters-per-second so the rest of
    // the code (and getPosition()/getVelocity()) can work in meters like it did with TalonFX.
    config.encoder
        .positionConversionFactor(SwerveConstants.WHEEL_CIRCUMFERENCE / SwerveConstants.DRIVE_GEAR_RATIO)
        .velocityConversionFactor(
            SwerveConstants.WHEEL_CIRCUMFERENCE / SwerveConstants.DRIVE_GEAR_RATIO / 60.0);

    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(SwerveConstants.DRIVE_kP)
        .i(SwerveConstants.DRIVE_kI)
        .d(SwerveConstants.DRIVE_kD);

    driveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private void configureAzimuthMotor(boolean invertAzimuth) {
    SparkMaxConfig config = new SparkMaxConfig();

    config.inverted(invertAzimuth);
    config.idleMode(SwerveConstants.AZIMUTH_COAST ? IdleMode.kCoast : IdleMode.kBrake);
    config.smartCurrentLimit(SwerveConstants.AZIMUTH_CURRENT_LIMIT);

    // Report azimuth position in wheel rotations (not motor rotations) so 1.0 = one full
    // wheel turn, matching Rotation2d.fromRotations() usage elsewhere in this class.
    config.encoder.positionConversionFactor(1.0 / SwerveConstants.AZIMUTH_GEAR_RATIO);

    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(SwerveConstants.AZIMUTH_kP)
        .i(SwerveConstants.AZIMUTH_kI)
        .d(SwerveConstants.AZIMUTH_kD)
        // Continuous wrap, in wheel rotations, so the module takes the shortest path
        // (e.g. 0.97 rotations -> 0.02 rotations goes +0.05, not -0.95).
        .positionWrappingEnabled(true)
        .positionWrappingMinInput(0.0)
        .positionWrappingMaxInput(1.0);

    azimuthMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Zeroes the azimuth relative encoder to the wheel's CURRENT physical position. There is no
   * absolute encoder, so call this only when the wheel is actually pointed forward (see the
   * AZIMUTH ZEROING note in SwerveConstants). Called automatically once at construction time.
   */
  public void resetToForward() {
    azimuthEncoder.setPosition(0);
  }

  // ================================================================
  // GETTERS
  // ================================================================

  /** Current azimuth angle, relative to wherever it was zeroed on boot (see resetToForward()). */
  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(azimuthEncoder.getPosition());
  }

  public double getVelocity() {
    return driveEncoder.getVelocity();
  }

  public double getPosition() {
    return driveEncoder.getPosition();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocity(), getAngle());
  }

  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(getPosition(), getAngle());
  }

  // ================================================================
  // CONTROL
  // ================================================================

  public void setDesiredState(SwerveModuleState desiredState, boolean openLoop) {
    desiredState.optimize(getAngle());

    // Azimuth — position control in wheel rotations
    azimuthController.setSetpoint(desiredState.angle.getRotations(), SparkMax.ControlType.kPosition);

    // Drive
    if (openLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.MAX_SPEED;
      driveMotor.set(percentOutput);
    } else {
      double ffVolts = driveFeedforward.calculate(desiredState.speedMetersPerSecond);
      driveController.setSetpoint(
          desiredState.speedMetersPerSecond,
          SparkMax.ControlType.kVelocity,
          ClosedLoopSlot.kSlot0,
          ffVolts,
          ArbFFUnits.kVoltage);
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