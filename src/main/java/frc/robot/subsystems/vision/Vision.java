package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  // Records
  public record CameraConfig(
      String name,
      Transform3d robotToCamera,
      PoseStrategy strategy) {
  }

  public record VisionUpdate(
      Pose3d pose3d,
      Pose2d pose2d,
      double timestampSeconds,
      int tagCount,
      double avgDistanceMeters,
      double avgAmbiguity) {
  }

  /* HIDDEN PROCESSES */
  private final AprilTagFieldLayout fieldLayout;
  private final List<PhotonCamera> cameras = new ArrayList<>();
  private final List<PhotonPoseEstimator> estimators = new ArrayList<>();

  // Scaleable camera list (just add more w/ new configs)
  private final List<CameraConfig> cameraConfigs = List.of(
      new CameraConfig(
          "example_cam_1",
          VisionConstants.EXAMPLE_CAMERA_TRANSFORM_1, // RobotToCamera
          PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR),

      new CameraConfig(
          "example_cam_2",
          VisionConstants.EXAMPLE_CAMERA_TRANSFORM_2, // RobotToCamera
          PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR));

  // Constructor
  public Vision() {
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    // Initialize cameras & estimators w/ config data
    for (CameraConfig cfg : cameraConfigs) {
      PhotonCamera cam = new PhotonCamera(cfg.name());
      PhotonPoseEstimator estimator = new PhotonPoseEstimator(
          fieldLayout,
          cfg.strategy(),
          cfg.robotToCamera());

      cameras.add(cam);
      estimators.add(estimator);
    }
  }

  private Optional<VisionUpdate> getSingleCameraUpdate(
      PhotonCamera camera,
      PhotonPoseEstimator estimator,
      Pose2d robotPose) {

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
    double avgAmbiguity = targets.stream()
        .mapToDouble(PhotonTrackedTarget::getPoseAmbiguity)
        .average()
        .orElse(1.0);

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
    if (tagCount < VisionConstants.MIN_TAG_COUNT)
      return Optional.empty();

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
    if (pose2d.getTranslation().getDistance(robotPose.getTranslation()) > VisionConstants.MAX_POSE_DIFFERENCE) {
      return Optional.empty();
    }

    double avgDistance = targets.stream().mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
        .average().orElse(5.0);
    if (avgDistance > VisionConstants.MAX_TAG_DISTANCE)
      return Optional.empty();

    SmartDashboard.putNumber("Vision/" + camera.getName() + "/TagCount", tagCount);
    SmartDashboard.putNumber("Vision/" + camera.getName() + "/AvgAmbiguity", avgAmbiguity);
    SmartDashboard.putNumber("Vision/" + camera.getName() + "/AvgDistance", avgDistance);
    SmartDashboard.putNumberArray("Vision/" + camera.getName() + "/Pose2d",
        new double[] { pose2d.getX(), pose2d.getY(), pose2d.getRotation().getDegrees() });

    return Optional.of(new VisionUpdate(
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

      double odomDistance = update.pose2d()
          .getTranslation()
          .getDistance(robotPose.getTranslation());
      score -= odomDistance / 2; // Difference when compared to 'official' odometry results

      if (score > bestScore) {
        bestScore = score;
        best = update;
      }
    }

    return Optional.ofNullable(best);
  }

  /* PUBLIC INTERFACE */
  public Optional<Pose2d> getPose2d(Pose2d robotPose) {
    return getBestVisionUpdate(robotPose).map(VisionUpdate::pose2d);
  }

  public Optional<Pose3d> getPose3d(Pose2d robotPose) {
    return getBestVisionUpdate(robotPose)
        .map(update -> update.pose3d());
  }

  /**
   * @param robotPose
   * @return raw data, as opposed to a Pose
   */
  public Optional<VisionUpdate> getBestVisionUpdateRaw(Pose2d robotPose) {
    return getBestVisionUpdate(robotPose);
  }

}