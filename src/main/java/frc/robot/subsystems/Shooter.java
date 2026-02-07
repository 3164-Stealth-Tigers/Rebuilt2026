package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.Optional;

/**
 * Shooter subsystem for launching FUEL into the HUB.
 *
 * <p>Features: - Pitch servo to adjust up/down angle - Flywheel motor to launch FUEL - Trajectory
 * calculations for automated shooting
 */
public class Shooter extends SubsystemBase {

  // ================================================================
  // HARDWARE
  // ================================================================

  /** Servo motor for pitch (up/down) angle control */
  private final Servo pitchServo;

  /** Flywheel motor for launching FUEL */
  private final SparkMax flywheelMotor;

  /** Flywheel encoder for velocity feedback */
  private final RelativeEncoder flywheelEncoder;

  /** PID controller for flywheel velocity */
  private final SparkClosedLoopController flywheelController;

  // ================================================================
  // DEPENDENCIES
  // ================================================================

  private final Vision vision;
  private final SwerveDrive swerve;

  // ================================================================
  // STATE TRACKING
  // ================================================================

  /** Current shooter state */
  private ShooterState state = ShooterState.IDLE;

  /** Target pitch angle in degrees */
  private double targetPitchAngle = ShooterConstants.PITCH_STOW_ANGLE;

  /** Target flywheel RPM */
  private double targetFlywheelRPM = 0.0;

  // ================================================================
  // TRAJECTORY CALCULATION FIELDS
  // ================================================================

  private double x, y, z;
  private Rotation2d heading;
  private Alliance alliance;
  private double[] hub = new double[2];

  // ================================================================
  // STATE ENUM
  // ================================================================

  public enum ShooterState {
    IDLE, // Flywheel stopped, servo at stow
    SPINNING_UP, // Flywheel accelerating to target speed
    READY, // Flywheel at speed, ready to shoot
    SHOOTING, // Actively feeding and shooting
    SPINNING_DOWN // Flywheel decelerating
  }

  // ================================================================
  // CONSTRUCTOR
  // ================================================================

  public Shooter(Vision vision, SwerveDrive swerve) {
    this.vision = vision;
    this.swerve = swerve;

    // Initialize pitch servo
    pitchServo = new Servo(ShooterConstants.PITCH_SERVO_CHANNEL);

    // Initialize flywheel motor
    flywheelMotor = new SparkMax(ShooterConstants.FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
    flywheelEncoder = flywheelMotor.getEncoder();
    flywheelController = flywheelMotor.getClosedLoopController();

    configureMotors();
    UpdateHubLocation();

    // Start at stow position
    setPitchAngle(ShooterConstants.PITCH_STOW_ANGLE);
  }

  // ================================================================
  // CONFIGURATION
  // ================================================================

  private void configureMotors() {
    SparkMaxConfig flywheelConfig = new SparkMaxConfig();
    flywheelConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.FLYWHEEL_CURRENT_LIMIT);

    flywheelConfig
        .closedLoop
        .p(ShooterConstants.FLYWHEEL_kP)
        .i(ShooterConstants.FLYWHEEL_kI)
        .d(ShooterConstants.FLYWHEEL_kD)
        .feedForward
        .kV(ShooterConstants.FLYWHEEL_kFF);

    flywheelMotor.configure(
        flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // ================================================================
  // PITCH CONTROL METHODS
  // ================================================================

  /**
   * Set the pitch angle of the shooter.
   *
   * @param angleDegrees Target angle in degrees (0 = horizontal, 90 = vertical)
   */
  public void setPitchAngle(double angleDegrees) {
    // Clamp to valid range
    targetPitchAngle =
        Math.max(
            ShooterConstants.PITCH_MIN_ANGLE,
            Math.min(ShooterConstants.PITCH_MAX_ANGLE, angleDegrees));

    // Convert angle to servo position (0.0 to 1.0)
    double servoPosition =
        (targetPitchAngle - ShooterConstants.PITCH_MIN_ANGLE)
            / (ShooterConstants.PITCH_MAX_ANGLE - ShooterConstants.PITCH_MIN_ANGLE);

    pitchServo.set(servoPosition);
  }

  /**
   * Get the current target pitch angle.
   *
   * @return Target pitch angle in degrees
   */
  public double getTargetPitchAngle() {
    return targetPitchAngle;
  }

  /**
   * Check if the pitch servo is approximately at the target angle. Note: Standard servos don't
   * provide position feedback, so this assumes the servo reaches position after a brief delay.
   *
   * @return true if servo should be at target
   */
  public boolean isPitchAtTarget() {
    // Servos typically reach position within ~200ms
    // For more precision, consider using a servo with feedback
    return true;
  }

  /** Set pitch to stow position. */
  public void stowPitch() {
    setPitchAngle(ShooterConstants.PITCH_STOW_ANGLE);
  }

  // ================================================================
  // FLYWHEEL CONTROL METHODS
  // ================================================================

  /**
   * Set the flywheel to a target velocity.
   *
   * @param rpm Target velocity in RPM
   */
  public void setFlywheelRPM(double rpm) {
    targetFlywheelRPM = rpm;

    if (rpm <= 0) {
      flywheelMotor.set(0);
      state = ShooterState.IDLE;
    } else {
      flywheelController.setSetpoint(rpm, SparkMax.ControlType.kVelocity);
      state = ShooterState.SPINNING_UP;
    }
  }

  /** Spin up the flywheel to shooting speed. */
  public void spinUp() {
    setFlywheelRPM(ShooterConstants.FLYWHEEL_SHOOT_RPM);
  }

  /** Spin up the flywheel to idle/warmup speed. */
  public void spinUpIdle() {
    setFlywheelRPM(ShooterConstants.FLYWHEEL_IDLE_RPM);
  }

  /** Stop the flywheel. */
  public void stopFlywheel() {
    setFlywheelRPM(0);
    state = ShooterState.SPINNING_DOWN;
  }

  /**
   * Get current flywheel velocity.
   *
   * @return Current velocity in RPM
   */
  public double getFlywheelRPM() {
    return flywheelEncoder.getVelocity();
  }

  /**
   * Check if flywheel is at target speed.
   *
   * @return true if flywheel is within tolerance of target
   */
  public boolean isFlywheelAtSpeed() {
    return Math.abs(getFlywheelRPM() - targetFlywheelRPM) < ShooterConstants.FLYWHEEL_RPM_TOLERANCE;
  }

  /**
   * Check if shooter is ready to fire (flywheel at speed and pitch set).
   *
   * @return true if ready to shoot
   */
  public boolean isReadyToShoot() {
    return isFlywheelAtSpeed() && isPitchAtTarget() && targetFlywheelRPM > 0;
  }

  // ================================================================
  // COMBINED SHOOTING METHODS
  // ================================================================

  /**
   * Prepare shooter for a shot at a specific angle and speed.
   *
   * @param pitchDegrees Pitch angle in degrees
   * @param flywheelRPM Flywheel speed in RPM
   */
  public void prepareShot(double pitchDegrees, double flywheelRPM) {
    setPitchAngle(pitchDegrees);
    setFlywheelRPM(flywheelRPM);
  }

  /** Prepare shooter with default shooting parameters. */
  public void prepareDefaultShot() {
    prepareShot(ShooterConstants.LAUNCH_ANGLE, ShooterConstants.FLYWHEEL_SHOOT_RPM);
  }

  /** Stop all shooter mechanisms and return to idle. */
  public void stop() {
    stopFlywheel();
    stowPitch();
    state = ShooterState.IDLE;
  }

  // ================================================================
  // COMMAND FACTORIES
  // ================================================================

  /** Command to spin up flywheel and wait until ready. */
  public Command spinUpCommand() {
    return Commands.sequence(
            Commands.runOnce(this::spinUp, this), Commands.waitUntil(this::isFlywheelAtSpeed))
        .withName("Spin Up Shooter");
  }

  /**
   * Command to prepare shooter with automatic pitch calculation. Uses trajectory calculation to
   * determine optimal pitch angle.
   */
  public Command prepareAutoShotCommand() {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  double calculatedPitch = calculateOptimalPitch();
                  prepareShot(calculatedPitch, ShooterConstants.FLYWHEEL_SHOOT_RPM);
                },
                this),
            Commands.waitUntil(this::isReadyToShoot))
        .withName("Prepare Auto Shot");
  }

  /**
   * Command to prepare shooter using vision-assisted targeting. Uses AprilTags for more precise
   * distance/angle calculation. Falls back to odometry if no tags visible.
   */
  public Command prepareVisionShotCommand() {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  VisionAimedShot shot = calculateOptimalPitchWithVision();
                  prepareShot(shot.pitchAngle(), ShooterConstants.FLYWHEEL_SHOOT_RPM);

                  // Log the shot info for driver feedback
                  SmartDashboard.putBoolean("Shooter/VisionAssisted", shot.visionAssisted());
                  SmartDashboard.putString("Shooter/ShotConfidence", shot.confidenceDescription());
                  SmartDashboard.putNumber("Shooter/VisionDistance", shot.distanceToHub());
                },
                this),
            Commands.waitUntil(this::isReadyToShoot))
        .withName("Prepare Vision Shot");
  }

  /**
   * Command to continuously track hub with vision and update pitch. Run this while waiting for a
   * shot opportunity.
   */
  public Command trackHubCommand() {
    return Commands.run(
            () -> {
              if (hasReliableVisionTarget()) {
                VisionAimedShot shot = calculateOptimalPitchWithVision();
                setPitchAngle(shot.pitchAngle());
              }
            },
            this)
        .withName("Track Hub");
  }

  /**
   * Command to spin up and aim using vision, then wait for driver trigger. Continuously updates
   * pitch based on AprilTag data.
   */
  public Command aimAndSpinUpCommand() {
    return Commands.parallel(
            // Keep flywheel spinning
            Commands.run(this::spinUp, this),
            // Continuously track the hub
            Commands.run(
                () -> {
                  VisionAimedShot shot = calculateOptimalPitchWithVision();
                  setPitchAngle(shot.pitchAngle());

                  // Update dashboard
                  SmartDashboard.putBoolean("Shooter/VisionAssisted", shot.visionAssisted());
                  SmartDashboard.putBoolean("Shooter/HighConfidence", shot.isHighConfidence());
                  SmartDashboard.putNumber("Shooter/AimPitch", shot.pitchAngle());
                },
                this))
        .withName("Aim and Spin Up");
  }

  /** Command to stop the shooter. */
  public Command stopCommand() {
    return Commands.runOnce(this::stop, this).withName("Stop Shooter");
  }

  /** Command to hold flywheel at idle speed (for warmup). */
  public Command idleCommand() {
    return Commands.run(this::spinUpIdle, this).withName("Idle Shooter");
  }

  // ================================================================
  // TRAJECTORY CALCULATION METHODS
  // ================================================================

  /** Get robot pose from swerve odometry. */
  private Pose2d getRobotPose() {
    return swerve.getPose();
  }

  /** Update hub location based on alliance. */
  @SuppressWarnings("unlikely-arg-type")
  private void UpdateHubLocation() {
    hub[1] = ShooterConstants.UNIV_Y;

    hub[0] =
        ((DriverStation.getAlliance().equals(Alliance.Blue))
            ? ShooterConstants.BLUE_X
            : ((DriverStation.getAlliance().equals(Alliance.Red))
                ? ShooterConstants.RED_X
                : -180.0));
  }

  /** Check if scoring is allowed based on game state and position. */
  private boolean canScore() {
    String gameData = DriverStation.getGameSpecificMessage();
    char active = (gameData.length() > 0) ? gameData.charAt(0) : 'Z';

    return ((active == 'B' && alliance.equals(Alliance.Blue) && x >= ShooterConstants.BLUE_X)
        || (active == 'R' && alliance.equals(Alliance.Red) && x <= ShooterConstants.RED_X));
  }

  /** Calculate angle from robot heading to shooter direction. */
  private Rotation2d robotToShooter() {
    return Rotation2d.fromDegrees(0.0);
  }

  /**
   * Calculate angle difference between robot and hub.
   *
   * @return Angle difference in degrees
   */
  public double robotToHub() {
    UpdateHubLocation();

    Optional<Pose3d> poseOpt = vision.getPose3d(getRobotPose());
    if (poseOpt.isEmpty()) {
      return 0.0;
    }

    Pose3d pose = poseOpt.get();
    x = pose.getX();
    y = pose.getY();
    z = pose.getZ();
    heading = pose.getRotation().toRotation2d();

    Rotation2d targetAngle = new Rotation2d(hub[0] - x, hub[1] - y);
    return heading.minus(targetAngle).minus(robotToShooter()).getDegrees();
  }

  /**
   * Calculate distance to hub.
   *
   * @return Optional distance in meters, empty if scoring not allowed
   */
  public Optional<Double> hubDistance() {
    UpdateHubLocation();

    if (!canScore()) {
      return Optional.empty();
    }

    Optional<Pose3d> poseOpt = vision.getPose3d(getRobotPose());
    if (poseOpt.isEmpty()) {
      return Optional.empty();
    }

    Pose3d pose = poseOpt.get();
    x = pose.getX();
    y = pose.getY();
    z = pose.getZ();
    heading = pose.getRotation().toRotation2d();

    Translation2d a = new Translation2d(x, y);
    Translation2d b = new Translation2d(hub[0], hub[1]);

    return Optional.of(a.getDistance(b));
  }

  /**
   * Calculate required launch velocity based on distance.
   *
   * @return Required velocity in m/s, or 0 if shot not possible
   */
  public double getRequiredVelocity() {
    Optional<Double> dx_proxy = hubDistance();

    if (dx_proxy.isEmpty()) {
      return 0.0;
    }

    Optional<Pose3d> poseOpt = vision.getPose3d(getRobotPose());
    if (poseOpt.isEmpty()) {
      return 0.0;
    }

    Pose3d pose = poseOpt.get();
    z = pose.getZ();

    double dx = dx_proxy.get();
    double dy = ShooterConstants.HUB_RIM_HEIGHT - (z + ShooterConstants.Z_OFFSET);
    double g = ShooterConstants.G_ACCEL;
    double theta = Math.toRadians(ShooterConstants.LAUNCH_ANGLE);

    return dx / Math.cos(theta) * Math.sqrt(g / 2 * (-dy + dx * Math.tan(theta)));
  }

  /**
   * Calculate optimal pitch angle for current position. This is a simplified calculation - adjust
   * based on testing.
   *
   * @return Optimal pitch angle in degrees
   */
  public double calculateOptimalPitch() {
    Optional<Double> distanceOpt = hubDistance();

    if (distanceOpt.isEmpty()) {
      return ShooterConstants.LAUNCH_ANGLE;
    }

    double distance = distanceOpt.get();
    return calculatePitchFromDistance(distance);
  }

  /**
   * Calculate optimal pitch angle using AprilTag vision for precise targeting.
   *
   * <p>This method uses the vision system to get a more accurate position estimate when AprilTags
   * are visible. Falls back to odometry-based calculation if vision data is unavailable or
   * unreliable.
   *
   * @return VisionAimedShot containing pitch angle and confidence metrics
   */
  public VisionAimedShot calculateOptimalPitchWithVision() {
    UpdateHubLocation();

    // Try to get vision-based pose
    Optional<Vision.VisionUpdate> visionUpdateOpt = vision.getBestVisionUpdate(getRobotPose());

    if (visionUpdateOpt.isEmpty()) {
      // No vision data - fall back to odometry
      double odometryPitch = calculateOptimalPitch();
      return new VisionAimedShot(
          odometryPitch,
          false, // not vision-assisted
          0, // no tags
          1.0, // max ambiguity (no confidence)
          0.0, // unknown distance
          "No AprilTags detected - using odometry");
    }

    Vision.VisionUpdate visionUpdate = visionUpdateOpt.get();

    // Calculate distance to hub using vision pose
    double visionX = visionUpdate.pose3d().getX();
    double visionY = visionUpdate.pose3d().getY();
    double visionZ = visionUpdate.pose3d().getZ();

    double dx = hub[0] - visionX;
    double dy = hub[1] - visionY;
    double horizontalDistance = Math.sqrt(dx * dx + dy * dy);

    // Calculate vertical component for more accurate pitch
    double dz = ShooterConstants.HUB_RIM_HEIGHT - (visionZ + ShooterConstants.Z_OFFSET);

    // Use physics-based pitch calculation when we have good vision data
    double pitch;
    if (visionUpdate.tagCount() >= 2 && visionUpdate.avgAmbiguity() < 0.2) {
      // High confidence - use physics-based calculation
      pitch = calculatePitchPhysics(horizontalDistance, dz);
    } else {
      // Lower confidence - use interpolation table
      pitch = calculatePitchFromDistance(horizontalDistance);
    }

    // Determine confidence level
    String confidence;
    if (visionUpdate.tagCount() >= 2 && visionUpdate.avgAmbiguity() < 0.15) {
      confidence = "HIGH - Multiple tags, low ambiguity";
    } else if (visionUpdate.tagCount() >= 1 && visionUpdate.avgAmbiguity() < 0.3) {
      confidence = "MEDIUM - Vision-assisted";
    } else {
      confidence = "LOW - High ambiguity, verify manually";
    }

    return new VisionAimedShot(
        pitch,
        true,
        visionUpdate.tagCount(),
        visionUpdate.avgAmbiguity(),
        horizontalDistance,
        confidence);
  }

  /**
   * Check if vision targeting is available and reliable.
   *
   * @return true if AprilTags are visible with acceptable ambiguity
   */
  public boolean hasReliableVisionTarget() {
    Optional<Vision.VisionUpdate> visionUpdateOpt = vision.getBestVisionUpdate(getRobotPose());

    if (visionUpdateOpt.isEmpty()) {
      return false;
    }

    Vision.VisionUpdate update = visionUpdateOpt.get();
    return update.tagCount() >= 1 && update.avgAmbiguity() < 0.4;
  }

  /**
   * Calculate pitch angle from horizontal distance using interpolation. Tune these values based on
   * testing.
   *
   * @param distance Horizontal distance to hub in meters
   * @return Pitch angle in degrees
   */
  private double calculatePitchFromDistance(double distance) {
    // Interpolation table - tune based on testing
    double minDistance = 2.0; // meters
    double maxDistance = 8.0; // meters
    double closeAngle = 70.0; // degrees for close shots
    double farAngle = 35.0; // degrees for far shots

    if (distance <= minDistance) {
      return closeAngle;
    } else if (distance >= maxDistance) {
      return farAngle;
    }

    // Linear interpolation
    double t = (distance - minDistance) / (maxDistance - minDistance);
    return closeAngle + t * (farAngle - closeAngle);
  }

  /**
   * Calculate pitch angle using physics-based projectile motion. Used when we have high-confidence
   * vision data.
   *
   * @param horizontalDistance Distance to target (meters)
   * @param verticalDistance Height difference to target (meters, positive = target higher)
   * @return Optimal pitch angle in degrees
   */
  private double calculatePitchPhysics(double horizontalDistance, double verticalDistance) {
    // Projectile motion: we need to solve for launch angle
    // Using the equation: tan(theta) = (v^2 +/- sqrt(v^4 - g(gx^2 + 2yv^2))) / (gx)
    // For simplicity, we'll use an iterative approach

    double g = ShooterConstants.G_ACCEL;

    // Estimate launch velocity from flywheel RPM (this needs calibration)
    // Assuming linear relationship between RPM and ball exit velocity
    double estimatedVelocity =
        ShooterConstants.FLYWHEEL_SHOOT_RPM * 0.001; // (placeholder TODO - calibrate)

    // Use high arc solution (+ sqrt term) for better accuracy
    double v2 = estimatedVelocity * estimatedVelocity;
    double v4 = v2 * v2;
    double x = horizontalDistance;
    double y = verticalDistance;

    double discriminant = v4 - g * (g * x * x + 2 * y * v2);

    if (discriminant < 0) {
      // Target unreachable at current velocity - return max angle
      return ShooterConstants.PITCH_MAX_ANGLE;
    }

    double sqrtDiscriminant = Math.sqrt(discriminant);
    double tanTheta = (v2 + sqrtDiscriminant) / (g * x);
    double theta = Math.toDegrees(Math.atan(tanTheta));

    // Clamp to valid range
    return Math.max(
        ShooterConstants.PITCH_MIN_ANGLE, Math.min(ShooterConstants.PITCH_MAX_ANGLE, theta));
  }

  /** Result of vision-assisted shot calculation. */
  public record VisionAimedShot(
      double pitchAngle, // Calculated pitch angle in degrees
      boolean visionAssisted, // True if AprilTags were used
      int tagCount, // Number of AprilTags detected
      double ambiguity, // Vision ambiguity (0 = perfect, 1 = bad)
      double distanceToHub, // Calculated distance to hub in meters
      String confidenceDescription // Human-readable confidence level
      ) {
    /** Check if this shot has high confidence. */
    public boolean isHighConfidence() {
      return visionAssisted && tagCount >= 2 && ambiguity < 0.2;
    }

    /** Check if this shot should be taken automatically. */
    public boolean isSafeForAutoShot() {
      return visionAssisted && tagCount >= 1 && ambiguity < 0.3;
    }
  }

  // ================================================================
  // GETTERS
  // ================================================================

  /** Get current shooter state */
  public ShooterState getState() {
    return state;
  }

  /** Get target flywheel RPM */
  public double getTargetFlywheelRPM() {
    return targetFlywheelRPM;
  }

  // ================================================================
  // PERIODIC
  // ================================================================

  @Override
  public void periodic() {
    // Update state based on flywheel status
    if (state == ShooterState.SPINNING_UP && isFlywheelAtSpeed()) {
      state = ShooterState.READY;
    } else if (state == ShooterState.SPINNING_DOWN && getFlywheelRPM() < 50) {
      state = ShooterState.IDLE;
    }

    // Update SmartDashboard - Core shooter status
    SmartDashboard.putString("Shooter/State", state.toString());
    SmartDashboard.putNumber("Shooter/PitchAngle", targetPitchAngle);
    SmartDashboard.putNumber("Shooter/FlywheelRPM", getFlywheelRPM());
    SmartDashboard.putNumber("Shooter/TargetRPM", targetFlywheelRPM);
    SmartDashboard.putBoolean("Shooter/AtSpeed", isFlywheelAtSpeed());
    SmartDashboard.putBoolean("Shooter/ReadyToShoot", isReadyToShoot());

    // Show distance to hub if available (odometry-based)
    hubDistance().ifPresent(d -> SmartDashboard.putNumber("Shooter/HubDistance", d));

    // Vision targeting status
    SmartDashboard.putBoolean("Shooter/VisionAvailable", hasReliableVisionTarget());

    // If vision is available, show what the vision-aimed shot would look like
    if (hasReliableVisionTarget()) {
      VisionAimedShot visionShot = calculateOptimalPitchWithVision();
      SmartDashboard.putNumber("Shooter/Vision/RecommendedPitch", visionShot.pitchAngle());
      SmartDashboard.putNumber("Shooter/Vision/TagCount", visionShot.tagCount());
      SmartDashboard.putNumber("Shooter/Vision/Ambiguity", visionShot.ambiguity());
      SmartDashboard.putNumber("Shooter/Vision/Distance", visionShot.distanceToHub());
      SmartDashboard.putBoolean("Shooter/Vision/HighConfidence", visionShot.isHighConfidence());
    }
  }
}
