package frc.robot.subsystems;

/*
 * ========================================================================
 * VISION SUBSYSTEM - Camera Processing & Target Detection
 * ========================================================================
 *
 * ========================================================================
 * ORANGE PI HARDWARE SETUP & WIRING GUIDE
 * ========================================================================
 *
 * RECOMMENDED MODEL:
 * ------------------
 * Orange Pi 5 (4GB or 8GB RAM recommended)
 * - RK3588S processor with NPU for ML acceleration
 * - Gigabit Ethernet for reliable NetworkTables communication
 * - USB 3.0 ports for high-bandwidth camera connections
 *
 * Alternative: Orange Pi 5 Plus (if running multiple cameras or heavy ML)
 *
 * WIRING DIAGRAM:
 * ---------------
 *
 *   +-----------------------------------------------------------------+
 *   |                        ROBOT ELECTRICAL PANEL                   |
 *   |                                                                 |
 *   |  +---------+    +---------+    +-------------+                 |
 *   |  |   PDP   |    |  VRM    |    |   RADIO     |                 |
 *   |  |  /PDH   |--->| 12V/5V  |    |  (OpenMesh  |                 |
 *   |  +---------+    +----+----+    |   or new)   |                 |
 *   |                      |         +------+------+                 |
 *   |                      |                |                        |
 *   |              5V 3A   |        Ethernet|                        |
 *   |              (USB-C) |                |                        |
 *   |                      v                v                        |
 *   |               +-----------------------------+                  |
 *   |               |        ORANGE PI 5          |                  |
 *   |               |                             |                  |
 *   |               |  USB-C <-- Power (5V/3A)    |                  |
 *   |               |  ETH   <-- Robot Network    |                  |
 *   |               |  USB3  <-- Camera 1         |                  |
 *   |               |  USB3  <-- Camera 2         |                  |
 *   |               |                             |                  |
 *   |               +-----------------------------+                  |
 *   |                                                                 |
 *   +-----------------------------------------------------------------+
 *
 * POWER WIRING:
 * -------------
 * Option 1 (Recommended): USB-C Power from VRM
 *   - VRM 5V/2A output -> USB-C cable -> Orange Pi USB-C power port
 *   - Use a quality USB-C cable rated for 3A
 *   - Note: VRM 5V/2A may be marginal; consider Option 2 for stability
 *
 * Option 2 (More Reliable): Dedicated 5V Regulator
 *   - PDP/PDH -> 5V/5A buck converter -> USB-C breakout -> Orange Pi
 *   - Recommended: Pololu 5V 5A Step-Down (D24V50F5)
 *   - Add 1000uF capacitor on output for stability
 *
 * Option 3: GPIO Header Power (Advanced)
 *   - 5V regulated supply -> Pin 2 & 4 (5V), Pin 6 (GND)
 *   - CAUTION: No reverse polarity protection on GPIO!
 *   - Must be exactly 5V (4.8V-5.2V tolerance)
 *
 * ETHERNET WIRING:
 * ----------------
 * - Use CAT5e or CAT6 Ethernet cable (keep under 3 meters on robot)
 * - Connect to robot radio's extra Ethernet port
 * - Or use an Ethernet switch if radio port is occupied:
 *     Radio --> 5-port Gigabit Switch --> Orange Pi
 *                                     +--> roboRIO
 *
 * CAMERA CONNECTIONS:
 * -------------------
 * - USB 3.0 cameras: Connect to blue USB 3.0 ports
 * - Recommended cameras:
 *     - Arducam OV9281 (global shutter, great for AprilTags)
 *     - Logitech C920/C922 (good general purpose)
 *     - See5CAM_CU135 (high resolution, wide FOV)
 * - Use short USB cables (<1m) or use powered USB hub
 * - Secure cables with zip ties to prevent disconnection
 *
 * ========================================================================
 * ORANGE PI SOFTWARE CONFIGURATION
 * ========================================================================
 *
 * STEP 1: Install Operating System
 * ---------------------------------
 * 1. Download Orange Pi OS (Debian-based) from orangepi.org
 *    - Use "Orangepi5_x.x.x_debian_bookworm_server_linux6.x.x.img"
 * 2. Flash to microSD card using balenaEtcher or Raspberry Pi Imager
 * 3. Insert microSD and power on Orange Pi
 * 4. Default login: orangepi / orangepi
 *
 * STEP 2: Configure Static IP Address
 * ------------------------------------
 * The Orange Pi MUST have a static IP in the 10.TE.AM.x range.
 * For team 3164, use 10.31.64.11 (or any .11-.19 address)
 *
 * Edit /etc/network/interfaces:
 * ```
 * auto eth0
 * iface eth0 inet static
 *     address 10.31.64.11
 *     netmask 255.255.255.0
 *     gateway 10.31.64.1
 * ```
 *
 * Or using nmcli:
 * ```
 * sudo nmcli con mod "Wired connection 1" ipv4.addresses 10.31.64.11/24
 * sudo nmcli con mod "Wired connection 1" ipv4.gateway 10.31.64.1
 * sudo nmcli con mod "Wired connection 1" ipv4.method manual
 * sudo nmcli con up "Wired connection 1"
 * ```
 *
 * STEP 3: Install PhotonVision
 * ----------------------------
 * ```bash
 * # Update system
 * sudo apt update && sudo apt upgrade -y
 *
 * # Install Java (required for PhotonVision)
 * sudo apt install openjdk-17-jdk -y
 *
 * # Download and install PhotonVision
 * wget https://github.com/PhotonVision/photonvision/releases/latest/download/photonvision-linux_arm64.jar
 * sudo mkdir -p /opt/photonvision
 * sudo mv photonvision-linux_arm64.jar /opt/photonvision/photonvision.jar
 *
 * # Create systemd service for auto-start
 * sudo tee /etc/systemd/system/photonvision.service << EOF
 * [Unit]
 * Description=PhotonVision
 * After=network.target
 *
 * [Service]
 * ExecStart=/usr/bin/java -jar /opt/photonvision/photonvision.jar
 * WorkingDirectory=/opt/photonvision
 * Restart=always
 * User=root
 *
 * [Install]
 * WantedBy=multi-user.target
 * EOF
 *
 * sudo systemctl enable photonvision
 * sudo systemctl start photonvision
 * ```
 *
 * STEP 4: Configure PhotonVision
 * ------------------------------
 * 1. Connect laptop to robot network
 * 2. Open browser to http://10.31.64.11:5800
 * 3. Add cameras and configure pipelines
 * 4. Set camera names to match VisionConstants.CAMERA_NAMES
 *    - Default: "example_cam_1", "example_cam_2"
 * 5. Configure AprilTag pipeline:
 *    - Select "AprilTag" pipeline type
 *    - Set tag family to "36h11" (2024+ FRC standard)
 *    - Adjust exposure for your lighting conditions
 *
 * STEP 5: Verify Connection
 * -------------------------
 * 1. On driver station laptop, open OutlineViewer or Shuffleboard
 * 2. Look for "photonvision" table in NetworkTables
 * 3. Check for camera data updating in real-time
 *
 * TROUBLESHOOTING:
 * ----------------
 * - No NetworkTables connection:
 *     - Verify static IP is set correctly (ping 10.31.64.11)
 *     - Check Ethernet cable connection
 *     - Ensure robot radio is configured for your team number
 *
 * - Cameras not detected:
 *     - Run `lsusb` to check if camera appears
 *     - Try different USB port (use USB 3.0 ports)
 *     - Check camera is compatible with Linux/V4L2
 *
 * - High latency / dropped frames:
 *     - Lower camera resolution (640x480 recommended for AprilTags)
 *     - Reduce exposure time
 *     - Use USB 3.0 instead of USB 2.0
 *     - Check CPU temperature: `cat /sys/class/thermal/thermal_zone0/temp`
 *
 * - Orange Pi overheating:
 *     - Add heatsink to RK3588S chip (included with most kits)
 *     - Add small 5V fan if in enclosed space
 *     - Reduce camera resolution/framerate
 *
 * ========================================================================
 * ARCHITECTURE OVERVIEW
 * ========================================================================
 *
 * The vision system runs on an Orange Pi coprocessor, which handles
 * computationally expensive tasks like neural network inference and
 * AprilTag detection. The Orange Pi publishes results to NetworkTables,
 * which this subsystem reads every 20ms.
 *
 *   +-----------------+              +-----------------+
 *   |    ORANGE PI    |              |     roboRIO     |
 *   |                 | NetworkTables|                 |
 *   |  Cameras -->    | ==========>  |  Vision.java    |
 *   |  Processing     |   ~30-60Hz   |  reads data     |
 *   |                 |              |  every 20ms     |
 *   +-----------------+              +-----------------+
 *
 * THREE TYPES OF VISION DATA:
 * ---------------------------
 *
 * 1. APRILTAG DETECTION (Field Positioning)
 *    - Detects AprilTags on the field
 *    - Calculates robot's position on the field
 *    - Used for autonomous navigation and pose correction
 *
 * 2. ROBOT DETECTION (Defense & Avoidance)
 *    - Detects other robots using ML model
 *    - Provides position and velocity for collision avoidance
 *    - Useful for defense strategies
 *
 * 3. FUEL DETECTION (Game Piece Tracking)
 *    - Detects FUEL game pieces on the field
 *    - Provides position for auto-intake
 *    - Can track multiple FUEL simultaneously
 *
 * ========================================================================
 * ORANGE PI -> ROBORIO DATA CONTRACTS (NetworkTables)
 * ========================================================================
 *
 * The Orange Pi will publish data to these NetworkTables paths.
 * This documents what we EXPECT to receive.
 *
 * +------------------------------------------------------------------------+
 * | 1. APRILTAG DATA                                                       |
 * |    NetworkTables Path: "Vision/AprilTag/"                              |
 * +------------------------------------------------------------------------+
 * |                                                                        |
 * |  KEY                    TYPE        DESCRIPTION                        |
 * |  ---                    ----        -----------                        |
 * |  hasTarget              boolean     True if any AprilTag detected      |
 * |  tagCount               int         Number of tags currently visible   |
 * |  primaryTagId           int         ID of best/closest tag             |
 * |  timestamp              double      FPGA timestamp of measurement      |
 * |                                                                        |
 * |  // Robot pose calculated from AprilTags                               |
 * |  robotPoseX             double      Robot X position (meters)          |
 * |  robotPoseY             double      Robot Y position (meters)          |
 * |  robotPoseTheta         double      Robot heading (radians)            |
 * |                                                                        |
 * |  // Quality metrics                                                    |
 * |  avgAmbiguity           double      0-1, lower is better               |
 * |  avgDistanceToTags      double      Avg distance to visible tags (m)   |
 * |  poseConfidence         double      0-1, confidence in pose estimate   |
 * |                                                                        |
 * |  // Per-tag data (arrays, indexed by detection order)                  |
 * |  tagIds                 int[]       IDs of all visible tags            |
 * |  tagDistances           double[]    Distance to each tag (meters)      |
 * |  tagAmbiguities         double[]    Ambiguity of each detection        |
 * |                                                                        |
 * |  EXAMPLE VALUES:                                                       |
 * |  ---------------                                                       |
 * |  hasTarget = true                                                      |
 * |  tagCount = 2                                                          |
 * |  robotPoseX = 5.2        // 5.2 meters from origin                     |
 * |  robotPoseY = 3.1        // 3.1 meters from origin                     |
 * |  robotPoseTheta = 1.57   // ~90 degrees (pi/2 radians)                  |
 * |  avgAmbiguity = 0.15     // Good quality detection                     |
 * |  timestamp = 45.123      // FPGA time when image was captured          |
 * |                                                                        |
 * +------------------------------------------------------------------------+
 *
 * +------------------------------------------------------------------------+
 * | 2. ROBOT DETECTION DATA                                                |
 * |    NetworkTables Path: "Vision/Robots/"                                |
 * +------------------------------------------------------------------------+
 * |                                                                        |
 * |  KEY                    TYPE        DESCRIPTION                        |
 * |  ---                    ----        -----------                        |
 * |  robotCount             int         Number of robots detected          |
 * |  timestamp              double      FPGA timestamp of detection        |
 * |                                                                        |
 * |  // Closest robot data (most relevant for collision avoidance)         |
 * |  closestRobotX          double      X position relative to us (m)      |
 * |  closestRobotY          double      Y position relative to us (m)      |
 * |  closestRobotDistance   double      Distance to closest robot (m)      |
 * |  closestRobotAngle      double      Angle to robot (radians, 0=front)  |
 * |  closestRobotVelX       double      Estimated X velocity (m/s)         |
 * |  closestRobotVelY       double      Estimated Y velocity (m/s)         |
 * |  closestRobotAlliance   string      "red", "blue", or "unknown"        |
 * |                                                                        |
 * |  // Detection confidence                                               |
 * |  closestConfidence      double      0-1, ML model confidence           |
 * |  closestBoundingBox     double[]    [x, y, width, height] in pixels    |
 * |                                                                        |
 * |  // All detected robots (arrays, up to 5 robots)                       |
 * |  robotXs                double[]    X positions of all detected        |
 * |  robotYs                double[]    Y positions of all detected        |
 * |  robotDistances         double[]    Distances to each robot            |
 * |  robotConfidences       double[]    Confidence for each detection      |
 * |                                                                        |
 * |  EXAMPLE VALUES:                                                       |
 * |  ---------------                                                       |
 * |  robotCount = 2                                                        |
 * |  closestRobotX = 1.5     // 1.5m in front of us                        |
 * |  closestRobotY = 0.3     // 0.3m to our left                           |
 * |  closestRobotDistance = 1.53  // sqrt(1.5^2 + 0.3^2)                        |
 * |  closestRobotAngle = 0.197    // atan2(0.3, 1.5) ~ 11.3 deg               |
 * |  closestRobotVelX = -0.5      // Moving toward us at 0.5 m/s           |
 * |  closestConfidence = 0.87     // 87% confident it's a robot            |
 * |                                                                        |
 * |  COORDINATE SYSTEM:                                                    |
 * |  ------------------                                                    |
 * |  Positions are ROBOT-RELATIVE (not field-relative)                     |
 * |                                                                        |
 * |              +X (front of robot)                                       |
 * |                    ^                                                   |
 * |                    |                                                   |
 * |       +Y (left) <---+---> -Y (right)                                     |
 * |                    |                                                   |
 * |                    v                                                   |
 * |              -X (back)                                                 |
 * |                                                                        |
 * +------------------------------------------------------------------------+
 *
 * +------------------------------------------------------------------------+
 * | 3. FUEL DETECTION DATA                                                 |
 * |    NetworkTables Path: "Vision/Fuel/"                                  |
 * +------------------------------------------------------------------------+
 * |                                                                        |
 * |  KEY                    TYPE        DESCRIPTION                        |
 * |  ---                    ----        -----------                        |
 * |  fuelCount              int         Number of FUEL detected            |
 * |  hasFuel                boolean     True if any FUEL visible           |
 * |  timestamp              double      FPGA timestamp of detection        |
 * |                                                                        |
 * |  // Best FUEL target (closest or most confident)                       |
 * |  bestFuelX              double      X position relative to robot (m)   |
 * |  bestFuelY              double      Y position relative to robot (m)   |
 * |  bestFuelDistance       double      Distance to FUEL (meters)          |
 * |  bestFuelAngle          double      Angle to FUEL (radians)            |
 * |  bestFuelConfidence     double      0-1, detection confidence          |
 * |  bestFuelInIntakeRange  boolean     True if close enough to intake     |
 * |                                                                        |
 * |  // Targeting assistance                                               |
 * |  angleToIntake          double      Angle robot should turn (rad)      |
 * |  distanceToIntake       double      Distance to drive (meters)         |
 * |  intakeReady            boolean     True if aligned for intake         |
 * |                                                                        |
 * |  // All detected FUEL (arrays, up to 10 FUEL)                          |
 * |  fuelXs                 double[]    X positions of all FUEL            |
 * |  fuelYs                 double[]    Y positions of all FUEL            |
 * |  fuelDistances          double[]    Distances to each FUEL             |
 * |  fuelConfidences        double[]    Confidence for each detection      |
 * |                                                                        |
 * |  // Field-relative positions (if AprilTags visible)                    |
 * |  fuelFieldXs            double[]    Field X positions (meters)         |
 * |  fuelFieldYs            double[]    Field Y positions (meters)         |
 * |  hasFieldPosition       boolean     True if field coords available     |
 * |                                                                        |
 * |  EXAMPLE VALUES:                                                       |
 * |  ---------------                                                       |
 * |  fuelCount = 3                                                         |
 * |  hasFuel = true                                                        |
 * |  bestFuelX = 0.8         // 0.8m in front of robot                     |
 * |  bestFuelY = -0.1        // 0.1m to the right                          |
 * |  bestFuelDistance = 0.806                                              |
 * |  bestFuelAngle = -0.124  // Turn right ~7 deg to align                    |
 * |  bestFuelConfidence = 0.94                                             |
 * |  bestFuelInIntakeRange = true  // Within 1m, ready to intake           |
 * |  angleToIntake = -0.124  // Same as bestFuelAngle                      |
 * |  intakeReady = false     // Need to turn 7 deg first                      |
 * |                                                                        |
 * |  FUEL DETECTION NOTES:                                                 |
 * |  ---------------------                                                 |
 * |  - FUEL is a yellow ball, ~7" diameter                                 |
 * |  - Detection uses color filtering + ML for robustness                  |
 * |  - Floor FUEL vs held FUEL distinguished by height                     |
 * |  - "InIntakeRange" threshold: ~1.0 meter                               |
 * |  - "intakeReady" requires: distance < 0.5m AND |angle| < 5 deg            |
 * |                                                                        |
 * +------------------------------------------------------------------------+
 *
 * +------------------------------------------------------------------------+
 * | LATENCY COMPENSATION                                                   |
 * +------------------------------------------------------------------------+
 * |                                                                        |
 * |  All vision data includes a TIMESTAMP from when the image was          |
 * |  captured. This is crucial because:                                    |
 * |                                                                        |
 * |  1. Camera capture: 0ms                                                |
 * |  2. Image transfer to Orange Pi: ~5ms                                     |
 * |  3. Processing (AprilTag/ML): ~20-40ms                                 |
 * |  4. NetworkTables publish: ~5ms                                        |
 * |  5. roboRIO reads data: ~0-20ms (depends on timing)                    |
 * |                                                                        |
 * |  TOTAL LATENCY: 30-70ms                                                |
 * |                                                                        |
 * |  For pose estimation, we use the timestamp to "rewind" the robot's     |
 * |  position to where it WAS when the image was taken:                    |
 * |                                                                        |
 * |    poseEstimator.addVisionMeasurement(pose, timestamp);                |
 * |                                                                        |
 * |  For FUEL/robot tracking, we can extrapolate current position:         |
 * |                                                                        |
 * |    double age = Timer.getFPGATimestamp() - timestamp;                  |
 * |    double currentX = detectedX + (velocityX * age);                    |
 * |                                                                        |
 * +------------------------------------------------------------------------+
 *
 * ========================================================================
 */

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Vision subsystem for camera-based detection and positioning.
 *
 * <p>Currently implements AprilTag detection via PhotonVision. FUEL and Robot detection will be
 * added when Orange Pi integration is complete.
 */
public class Vision extends SubsystemBase {

  // ========================================================================
  // DATA RECORDS - Structures for vision data
  // ========================================================================

  /** Configuration for a single camera. */
  public record CameraConfig(String name, Transform3d robotToCamera, PoseStrategy strategy) {}

  /** AprilTag-based pose update from vision system. */
  public record VisionUpdate(
      Pose3d pose3d,
      Pose2d pose2d,
      double timestampSeconds,
      int tagCount,
      double avgDistanceMeters,
      double avgAmbiguity) {}

  // ========================================================================
  // STUB RECORDS - Data structures for Orange Pi integration (TODO: implement)
  // ========================================================================

  /**
   * STUB: Detected robot data from Orange Pi ML model.
   *
   * <p>[ORANGE PI WILL PROVIDE] - Position relative to our robot (robot-centric coordinates) -
   * Estimated velocity for prediction - Alliance color if determinable - Confidence score from ML
   * model
   *
   * @param x X position relative to robot (meters, positive = front)
   * @param y Y position relative to robot (meters, positive = left)
   * @param distance Direct distance to robot (meters)
   * @param angle Angle to robot (radians, 0 = directly ahead)
   * @param velocityX Estimated X velocity (m/s)
   * @param velocityY Estimated Y velocity (m/s)
   * @param confidence ML detection confidence (0-1)
   * @param alliance Detected alliance ("red", "blue", "unknown")
   * @param timestamp FPGA timestamp of detection
   */
  public record DetectedRobot(
      double x,
      double y,
      double distance,
      double angle,
      double velocityX,
      double velocityY,
      double confidence,
      String alliance,
      double timestamp) {

    /**
     * Check if this robot is on a collision course with us.
     *
     * @param timeHorizon How far ahead to predict (seconds)
     * @param safeDistance Minimum safe distance (meters)
     * @return True if predicted to be within safeDistance
     */
    public boolean isCollisionThreat(double timeHorizon, double safeDistance) {
      // Predict where robot will be
      double predictedX = x + velocityX * timeHorizon;
      double predictedY = y + velocityY * timeHorizon;
      double predictedDistance = Math.sqrt(predictedX * predictedX + predictedY * predictedY);
      return predictedDistance < safeDistance;
    }
  }

  /**
   * STUB: Detected FUEL (game piece) data from Orange Pi.
   *
   * <p>[ORANGE PI WILL PROVIDE] - Position relative to robot (for driving to it) - Position on
   * field (if AprilTags visible) - Intake alignment assistance
   *
   * @param x X position relative to robot (meters)
   * @param y Y position relative to robot (meters)
   * @param distance Distance to FUEL (meters)
   * @param angle Angle to FUEL (radians)
   * @param confidence Detection confidence (0-1)
   * @param fieldX Field-relative X (meters), NaN if unknown
   * @param fieldY Field-relative Y (meters), NaN if unknown
   * @param timestamp FPGA timestamp of detection
   */
  public record DetectedFuel(
      double x,
      double y,
      double distance,
      double angle,
      double confidence,
      double fieldX,
      double fieldY,
      double timestamp) {

    /** Distance threshold for "ready to intake" */
    public static final double INTAKE_RANGE = 1.0; // meters

    /** Angle threshold for "aligned with intake" */
    public static final double INTAKE_ANGLE_TOLERANCE = Math.toRadians(5); // 5 degrees

    /** Check if FUEL is close enough to intake. */
    public boolean inIntakeRange() {
      return distance < INTAKE_RANGE;
    }

    /** Check if robot is aligned to intake this FUEL. */
    public boolean isAligned() {
      return Math.abs(angle) < INTAKE_ANGLE_TOLERANCE;
    }

    /** Check if field position is known (AprilTags were visible). */
    public boolean hasFieldPosition() {
      return !Double.isNaN(fieldX) && !Double.isNaN(fieldY);
    }
  }

  /**
   * STUB: Complete vision frame from Orange Pi containing all detections.
   *
   * <p>[ORANGE PI PUBLISHES ONE OF THESE EACH FRAME] Contains all detections from a single camera
   * frame.
   *
   * @param timestamp When the image was captured (FPGA time)
   * @param robots List of detected robots (may be empty)
   * @param fuels List of detected FUEL (may be empty)
   * @param aprilTagPose Robot pose from AprilTags (empty if no tags)
   * @param processingTimeMs How long Orange Pi took to process (for monitoring)
   */
  public record VisionFrame(
      double timestamp,
      List<DetectedRobot> robots,
      List<DetectedFuel> fuels,
      Optional<VisionUpdate> aprilTagPose,
      double processingTimeMs) {

    /** Get the closest detected robot, if any. */
    public Optional<DetectedRobot> getClosestRobot() {
      return robots.stream().min((a, b) -> Double.compare(a.distance(), b.distance()));
    }

    /** Get the best FUEL to target (closest with high confidence). */
    public Optional<DetectedFuel> getBestFuel() {
      return fuels.stream()
          .filter(f -> f.confidence() > 0.5) // Minimum confidence
          .min((a, b) -> Double.compare(a.distance(), b.distance()));
    }

    /** Get all robots that are collision threats. */
    public List<DetectedRobot> getCollisionThreats(double timeHorizon, double safeDistance) {
      return robots.stream().filter(r -> r.isCollisionThreat(timeHorizon, safeDistance)).toList();
    }
  }

  // ========================================================================
  // NETWORKTABLES KEYS - Where Orange Pi publishes data (for reference)
  // ========================================================================

  /**
   * NetworkTables paths where Orange Pi will publish data. These are constants for consistency
   * between Orange Pi and roboRIO code.
   */
  public static final class NTKeys {
    // AprilTag data
    public static final String APRILTAG_TABLE = "Vision/AprilTag";
    public static final String APRILTAG_HAS_TARGET = "hasTarget";
    public static final String APRILTAG_TAG_COUNT = "tagCount";
    public static final String APRILTAG_ROBOT_POSE_X = "robotPoseX";
    public static final String APRILTAG_ROBOT_POSE_Y = "robotPoseY";
    public static final String APRILTAG_ROBOT_POSE_THETA = "robotPoseTheta";
    public static final String APRILTAG_TIMESTAMP = "timestamp";
    public static final String APRILTAG_CONFIDENCE = "poseConfidence";

    // Robot detection data
    public static final String ROBOT_TABLE = "Vision/Robots";
    public static final String ROBOT_COUNT = "robotCount";
    public static final String ROBOT_CLOSEST_X = "closestRobotX";
    public static final String ROBOT_CLOSEST_Y = "closestRobotY";
    public static final String ROBOT_CLOSEST_DISTANCE = "closestRobotDistance";
    public static final String ROBOT_CLOSEST_VEL_X = "closestRobotVelX";
    public static final String ROBOT_CLOSEST_VEL_Y = "closestRobotVelY";
    public static final String ROBOT_CLOSEST_CONFIDENCE = "closestConfidence";
    public static final String ROBOT_TIMESTAMP = "timestamp";

    // FUEL detection data
    public static final String FUEL_TABLE = "Vision/Fuel";
    public static final String FUEL_COUNT = "fuelCount";
    public static final String FUEL_HAS_FUEL = "hasFuel";
    public static final String FUEL_BEST_X = "bestFuelX";
    public static final String FUEL_BEST_Y = "bestFuelY";
    public static final String FUEL_BEST_DISTANCE = "bestFuelDistance";
    public static final String FUEL_BEST_ANGLE = "bestFuelAngle";
    public static final String FUEL_BEST_CONFIDENCE = "bestFuelConfidence";
    public static final String FUEL_IN_INTAKE_RANGE = "bestFuelInIntakeRange";
    public static final String FUEL_INTAKE_READY = "intakeReady";
    public static final String FUEL_TIMESTAMP = "timestamp";
  }

  // ========================================================================
  // PHOTONVISION APRILTAG IMPLEMENTATION
  // ========================================================================
  //
  // This section handles AprilTag detection using PhotonVision.
  // PhotonVision runs on a coprocessor (Raspberry Pi or Orange Pi)
  // and handles the image processing. Results come via NetworkTables.
  //

  /** AprilTag positions on the field (loaded from JSON) */
  private final AprilTagFieldLayout fieldLayout;

  /** PhotonVision camera connections */
  private final List<PhotonCamera> cameras = new ArrayList<>();

  /** Pose estimators for each camera */
  private final List<PhotonPoseEstimator> estimators = new ArrayList<>();

  /**
   * Camera configurations - add more cameras by adding to this list.
   *
   * <p>Each camera needs: - name: Must match the camera name in PhotonVision - robotToCamera:
   * Transform from robot center to camera position - strategy: How to calculate pose (MULTI_TAG is
   * best)
   */
  private final List<CameraConfig> cameraConfigs =
      List.of(
          new CameraConfig(
              "example_cam_1",
              VisionConstants.EXAMPLE_CAMERA_TRANSFORM_1,
              PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR),
          new CameraConfig(
              "example_cam_2",
              VisionConstants.EXAMPLE_CAMERA_TRANSFORM_2,
              PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR));

  // ========================================================================
  // TODO: ORANGE PI INTEGRATION FIELDS
  // ========================================================================
  //
  // When Orange Pi integration is implemented, add these fields:
  //
  // private final NetworkTable robotDetectionTable;
  // private final NetworkTable fuelDetectionTable;
  // private DetectedRobot closestRobot;
  // private DetectedFuel bestFuel;
  // private List<DetectedRobot> allRobots = new ArrayList<>();
  // private List<DetectedFuel> allFuels = new ArrayList<>();
  //

  // ========================================================================
  // CONSTRUCTOR
  // ========================================================================

  public Vision() {
    try {
      fieldLayout =
          new AprilTagFieldLayout(
              Filesystem.getDeployDirectory().toPath().resolve("2026-rebuilt-welded.json"));
    } catch (IOException e) {
      throw new RuntimeException("Failed to load AprilTag layout", e);
    }

    // Initialize cameras & estimators w/ config data
    for (CameraConfig cfg : cameraConfigs) {
      PhotonCamera cam = new PhotonCamera(cfg.name());
      PhotonPoseEstimator estimator =
          new PhotonPoseEstimator(fieldLayout, cfg.strategy(), cfg.robotToCamera());

      cameras.add(cam);
      estimators.add(estimator);
    }
  }

  private Optional<VisionUpdate> getSingleCameraUpdate(
      PhotonCamera camera, PhotonPoseEstimator estimator, Pose2d robotPose) {

    PhotonPipelineResult result = camera.getLatestResult();

    /* FILTERS */

    // Is this result stale?
    double age = Timer.getFPGATimestamp() - result.getTimestampSeconds();
    if (age > VisionConstants.MAX_FRAME_AGE) {
      return Optional.empty();
    }

    // Are there any tags?
    if (!result.hasTargets()) {
      return Optional.empty();
    }

    List<PhotonTrackedTarget> targets = result.getTargets();
    int tagCount = targets.size();
    double avgAmbiguity =
        targets.stream().mapToDouble(PhotonTrackedTarget::getPoseAmbiguity).average().orElse(1.0);

    // Is this tag legal?
    for (PhotonTrackedTarget t : targets) {
      if (fieldLayout.getTagPose(t.getFiducialId()).isEmpty()) {
        return Optional.empty();
      }
    }

    // Is this tag close enough for our liking?
    if (targets.stream().anyMatch(t -> t.getArea() < VisionConstants.MIN_AREA)) {
      return Optional.empty();
    }

    // Are there enough tags for us to make a good guess?
    if (tagCount < VisionConstants.MIN_TAG_COUNT) return Optional.empty();

    // Does data meet our custom, personal standards?
    if (avgAmbiguity > VisionConstants.AMBIGUITY_THRESHOLD) {
      return Optional.empty();
    }

    estimator.setReferencePose(robotPose);

    Optional<EstimatedRobotPose> estOpt = estimator.update(result);

    // Did the estimator get a result?
    if (estOpt.isEmpty()) {
      return Optional.empty();
    }

    EstimatedRobotPose est = estOpt.get();
    Pose2d pose2d = est.estimatedPose.toPose2d();

    // Did we do a crazy change from our last position?
    if (pose2d.getTranslation().getDistance(robotPose.getTranslation())
        > VisionConstants.MAX_POSE_DIFFERENCE) {
      return Optional.empty();
    }

    double avgDistance =
        targets.stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .average()
            .orElse(5.0);
    if (avgDistance > VisionConstants.MAX_TAG_DISTANCE) return Optional.empty();

    SmartDashboard.putNumber("Vision/" + camera.getName() + "/TagCount", tagCount);
    SmartDashboard.putNumber("Vision/" + camera.getName() + "/AvgAmbiguity", avgAmbiguity);
    SmartDashboard.putNumber("Vision/" + camera.getName() + "/AvgDistance", avgDistance);
    SmartDashboard.putNumberArray(
        "Vision/" + camera.getName() + "/Pose2d",
        new double[] {pose2d.getX(), pose2d.getY(), pose2d.getRotation().getDegrees()});

    return Optional.of(
        new VisionUpdate(
            est.estimatedPose,
            est.estimatedPose.toPose2d(),
            est.timestampSeconds,
            tagCount,
            avgDistance,
            avgAmbiguity));
  }

  public Optional<VisionUpdate> getBestVisionUpdate(Pose2d robotPose) {
    VisionUpdate best = null;
    double bestScore = Double.NEGATIVE_INFINITY;

    // Rank estiamations from each camera
    for (int i = 0; i < estimators.size(); i++) {
      PhotonCamera cam = cameras.get(i);
      PhotonPoseEstimator est = estimators.get(i);

      Optional<VisionUpdate> updateOpt = getSingleCameraUpdate(cam, est, robotPose);
      if (updateOpt.isEmpty()) {
        continue;
      }

      VisionUpdate update = updateOpt.get();

      double score = 0.0;
      score += 2.0 * update.tagCount(); // more tags is good
      score += 1.5 * (1.0 / (update.avgDistanceMeters() + 0.1)); // less distance is good
      score += 1.0 * (1.0 - Math.min(update.avgAmbiguity(), 1.0)); // certainty is good

      double odomDistance =
          update.pose2d().getTranslation().getDistance(robotPose.getTranslation());
      score -= odomDistance / 2; // Difference when compared to 'official' odometry results

      if (score > bestScore) {
        bestScore = score;
        best = update;
      }
    }

    return Optional.ofNullable(best);
  }

  // ========================================================================
  // PUBLIC INTERFACE - AprilTag Methods
  // ========================================================================

  /**
   * Get the robot's 2D pose from AprilTag detection.
   *
   * @param robotPose Current odometry pose (for filtering bad detections)
   * @return Robot pose if valid detection available, empty otherwise
   */
  public Optional<Pose2d> getPose2d(Pose2d robotPose) {
    return getBestVisionUpdate(robotPose).map(VisionUpdate::pose2d);
  }

  /**
   * Get the robot's 3D pose from AprilTag detection.
   *
   * @param robotPose Current odometry pose (for filtering)
   * @return 3D pose if available
   */
  public Optional<Pose3d> getPose3d(Pose2d robotPose) {
    return getBestVisionUpdate(robotPose).map(update -> update.pose3d());
  }

  /**
   * Get the raw vision update with all metadata.
   *
   * @param robotPose Current odometry pose
   * @return Complete vision update with confidence metrics
   */
  public Optional<VisionUpdate> getBestVisionUpdateRaw(Pose2d robotPose) {
    return getBestVisionUpdate(robotPose);
  }

  // ========================================================================
  // TODO: ORANGE PI INTEGRATION - Robot Detection Methods
  // ========================================================================
  //
  // These methods will read robot detection data from NetworkTables.
  // Uncomment and implement when Orange Pi integration is ready.
  //
  // /**
  //  * Get the closest detected robot.
  //  *
  //  * [USE CASES]
  //  * - Collision avoidance in autonomous
  //  * - Defense strategy (track opponent)
  //  * - Avoid blocking alliance partners
  //  *
  //  * @return Closest robot if detected, empty otherwise
  //  */
  // public Optional<DetectedRobot> getClosestRobot() {
  //     // Read from NetworkTables
  //     var table = NetworkTableInstance.getDefault().getTable(NTKeys.ROBOT_TABLE);
  //     int count = (int) table.getEntry(NTKeys.ROBOT_COUNT).getInteger(0);
  //
  //     if (count == 0) {
  //         return Optional.empty();
  //     }
  //
  //     return Optional.of(new DetectedRobot(
  //         table.getEntry(NTKeys.ROBOT_CLOSEST_X).getDouble(0),
  //         table.getEntry(NTKeys.ROBOT_CLOSEST_Y).getDouble(0),
  //         table.getEntry(NTKeys.ROBOT_CLOSEST_DISTANCE).getDouble(0),
  //         Math.atan2(
  //             table.getEntry(NTKeys.ROBOT_CLOSEST_Y).getDouble(0),
  //             table.getEntry(NTKeys.ROBOT_CLOSEST_X).getDouble(0)),
  //         table.getEntry(NTKeys.ROBOT_CLOSEST_VEL_X).getDouble(0),
  //         table.getEntry(NTKeys.ROBOT_CLOSEST_VEL_Y).getDouble(0),
  //         table.getEntry(NTKeys.ROBOT_CLOSEST_CONFIDENCE).getDouble(0),
  //         "unknown",
  //         table.getEntry(NTKeys.ROBOT_TIMESTAMP).getDouble(0)
  //     ));
  // }
  //
  // /**
  //  * Check if there's a collision threat within the time horizon.
  //  *
  //  * @param timeHorizon How far ahead to predict (seconds)
  //  * @param safeDistance Minimum distance to maintain (meters)
  //  * @return True if evasive action recommended
  //  */
  // public boolean hasCollisionThreat(double timeHorizon, double safeDistance) {
  //     return getClosestRobot()
  //         .map(r -> r.isCollisionThreat(timeHorizon, safeDistance))
  //         .orElse(false);
  // }
  //
  // /**
  //  * Get avoidance vector to steer away from detected robot.
  //  *
  //  * @return [steerX, steerY] normalized vector pointing away from threat
  //  */
  // public double[] getAvoidanceVector() {
  //     return getClosestRobot().map(robot -> {
  //         // Point away from the robot
  //         double magnitude = Math.sqrt(robot.x() * robot.x() + robot.y() * robot.y());
  //         if (magnitude < 0.01) return new double[] {0, 0};
  //         return new double[] {-robot.x() / magnitude, -robot.y() / magnitude};
  //     }).orElse(new double[] {0, 0});
  // }

  // ========================================================================
  // TODO: ORANGE PI INTEGRATION - FUEL Detection Methods
  // ========================================================================
  //
  // /**
  //  * Get the best FUEL to target for intake.
  //  *
  //  * [SELECTION CRITERIA]
  //  * - Closest FUEL with confidence > 0.5
  //  * - Prefers FUEL in intake range
  //  * - Ignores FUEL that are likely held by other robots
  //  *
  //  * @return Best FUEL target if detected, empty otherwise
  //  */
  // public Optional<DetectedFuel> getBestFuel() {
  //     var table = NetworkTableInstance.getDefault().getTable(NTKeys.FUEL_TABLE);
  //     boolean hasFuel = table.getEntry(NTKeys.FUEL_HAS_FUEL).getBoolean(false);
  //
  //     if (!hasFuel) {
  //         return Optional.empty();
  //     }
  //
  //     return Optional.of(new DetectedFuel(
  //         table.getEntry(NTKeys.FUEL_BEST_X).getDouble(0),
  //         table.getEntry(NTKeys.FUEL_BEST_Y).getDouble(0),
  //         table.getEntry(NTKeys.FUEL_BEST_DISTANCE).getDouble(0),
  //         table.getEntry(NTKeys.FUEL_BEST_ANGLE).getDouble(0),
  //         table.getEntry(NTKeys.FUEL_BEST_CONFIDENCE).getDouble(0),
  //         Double.NaN,  // Field position not always available
  //         Double.NaN,
  //         table.getEntry(NTKeys.FUEL_TIMESTAMP).getDouble(0)
  //     ));
  // }
  //
  // /**
  //  * Check if there's a FUEL ready to intake (close and aligned).
  //  *
  //  * @return True if robot can intake immediately
  //  */
  // public boolean isFuelIntakeReady() {
  //     var table = NetworkTableInstance.getDefault().getTable(NTKeys.FUEL_TABLE);
  //     return table.getEntry(NTKeys.FUEL_INTAKE_READY).getBoolean(false);
  // }
  //
  // /**
  //  * Get the angle to turn to align with the best FUEL.
  //  *
  //  * @return Angle in radians (positive = turn left), 0 if no FUEL
  //  */
  // public double getAngleToFuel() {
  //     return getBestFuel().map(DetectedFuel::angle).orElse(0.0);
  // }
  //
  // /**
  //  * Get the distance to drive to reach the best FUEL.
  //  *
  //  * @return Distance in meters, 0 if no FUEL
  //  */
  // public double getDistanceToFuel() {
  //     return getBestFuel().map(DetectedFuel::distance).orElse(0.0);
  // }
  //
  // /**
  //  * Check if any FUEL is visible.
  //  *
  //  * @return True if at least one FUEL detected
  //  */
  // public boolean hasFuelTarget() {
  //     var table = NetworkTableInstance.getDefault().getTable(NTKeys.FUEL_TABLE);
  //     return table.getEntry(NTKeys.FUEL_HAS_FUEL).getBoolean(false);
  // }
  //
  // /**
  //  * Get count of visible FUEL.
  //  *
  //  * @return Number of FUEL currently detected
  //  */
  // public int getFuelCount() {
  //     var table = NetworkTableInstance.getDefault().getTable(NTKeys.FUEL_TABLE);
  //     return (int) table.getEntry(NTKeys.FUEL_COUNT).getInteger(0);
  // }

  // ========================================================================
  // TODO: PERIODIC - Read Orange Pi data each loop
  // ========================================================================
  //
  // @Override
  // public void periodic() {
  //     // Read AprilTag data (already handled by PhotonVision)
  //
  //     // Read robot detection data from Orange Pi
  //     // updateRobotDetections();
  //
  //     // Read FUEL detection data from Orange Pi
  //     // updateFuelDetections();
  //
  //     // Log to SmartDashboard for debugging
  //     // logOrangePiData();
  // }
  //
  // private void updateRobotDetections() {
  //     var table = NetworkTableInstance.getDefault().getTable(NTKeys.ROBOT_TABLE);
  //     int count = (int) table.getEntry(NTKeys.ROBOT_COUNT).getInteger(0);
  //
  //     SmartDashboard.putNumber("Vision/Robots/Count", count);
  //     if (count > 0) {
  //         SmartDashboard.putNumber("Vision/Robots/ClosestDistance",
  //             table.getEntry(NTKeys.ROBOT_CLOSEST_DISTANCE).getDouble(0));
  //     }
  // }
  //
  // private void updateFuelDetections() {
  //     var table = NetworkTableInstance.getDefault().getTable(NTKeys.FUEL_TABLE);
  //     int count = (int) table.getEntry(NTKeys.FUEL_COUNT).getInteger(0);
  //     boolean intakeReady = table.getEntry(NTKeys.FUEL_INTAKE_READY).getBoolean(false);
  //
  //     SmartDashboard.putNumber("Vision/Fuel/Count", count);
  //     SmartDashboard.putBoolean("Vision/Fuel/IntakeReady", intakeReady);
  //     if (count > 0) {
  //         SmartDashboard.putNumber("Vision/Fuel/BestDistance",
  //             table.getEntry(NTKeys.FUEL_BEST_DISTANCE).getDouble(0));
  //         SmartDashboard.putNumber("Vision/Fuel/BestAngle",
  //             Math.toDegrees(table.getEntry(NTKeys.FUEL_BEST_ANGLE).getDouble(0)));
  //     }
  // }

} // End of Vision class
