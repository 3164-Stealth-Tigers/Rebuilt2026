package frc.robot.subsystems.vision;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Transform3d;

public record CameraConfig(
    String name,
    Transform3d robotToCamera,
    PoseStrategy strategy
) {}
