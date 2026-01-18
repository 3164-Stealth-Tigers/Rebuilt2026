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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  // Records
  public record CameraConfig(
      String name,
      Transform3d robotToCamera,
      PoseStrategy strategy) {
  }

  public record VisionUpdate(
      Pose2d pose,
      double timestampSeconds,
      int tagCount,
      double avgDistanceMeters,
      double avgAmbiguity) {
  }

  /* BACKGROUND PROCESSES */
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
    if (!result.hasTargets()) {
      return Optional.empty();
    }

    List<PhotonTrackedTarget> targets = result.getTargets();
    if (targets.isEmpty()) {
      return Optional.empty();
    }

    int tagCount = targets.size();
    double avgAmbiguity = targets.stream()
        .mapToDouble(PhotonTrackedTarget::getPoseAmbiguity)
        .average()
        .orElse(1.0);

    if (avgAmbiguity > Constants.VisionConstants.AMBIGUITY_THRESHOLD) {
      return Optional.empty();
    }

    estimator.setReferencePose(robotPose);

    Optional<EstimatedRobotPose> estOpt = estimator.update(result);
    if (estOpt.isEmpty()) {
      return Optional.empty();
    }

    EstimatedRobotPose est = estOpt.get();
    Pose2d pose2d = est.estimatedPose.toPose2d();

    return Optional.of(new VisionUpdate(
        pose2d,
        est.timestampSeconds,
        result.getTargets().size(),
        /* avgDistance */ 0.0,
        /* avgAmbiguity */ 0.0));
  }

  /* PUBLIC INTERFACE */
  private static double x, y, z;
  private static Rotation2d angle;

  public static double getX() {
    return x;
  } // determine x coordinate of the robot, where x axis runs along long axis of
    // field, in meters

  public static double getY() {
    return y;
  } // determine y coordinate of the robot, where y axis runs along short axis of
    // field, in meters

  public static double getZ() {
    return z;
  } // determine z coordinate of the robot, where z axis is vertical, in meters

  public static Rotation2d getAngle() {
    return angle;
  } // determine angle that the robot is facing where 0 is facing away from the
    // driver station, in degrees
}