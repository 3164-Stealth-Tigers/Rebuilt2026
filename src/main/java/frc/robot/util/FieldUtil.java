package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.FieldConstants;

// MIGHT use this. It's probably not ready at the moment
public final class FieldUtil {
  private FieldUtil() {
    // Utility class - prevent instantiation
  }

  // ========================================================================
  // ALLIANCE FLIPPING
  // ========================================================================
  //
  // [WHY WE FLIP]
  // All our autonomous paths and positions are defined for Blue alliance.
  // When we're on Red alliance, we flip everything to the mirrored position.
  //
  // [THE MATH]
  // X: Flip across center of field -> newX = FIELD_LENGTH - oldX
  // Y: Flip across center of field -> newY = FIELD_WIDTH - oldY
  // Rotation: Flip direction -> rotate 180 deg (negate cos and sin)
  //
  // ========================================================================

  /**
   * Flip a translation (X, Y position) for Red alliance.
   *
   * <p>[WHEN TO USE] Call this on any position that's defined for Blue alliance before using it. If
   * we're Blue, it returns unchanged.
   *
   * @param translation The position (Blue alliance origin)
   * @return The flipped position if Red, original if Blue
   */
  public static Translation2d flipAlliance(Translation2d translation) {
    // Check our alliance color (default to Blue if unknown)
    boolean shouldFlip =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Red;

    if (shouldFlip) {
      // Mirror position to opposite side of field
      return new Translation2d(
          FieldConstants.FIELD_LENGTH - translation.getX(), // Flip X
          FieldConstants.FIELD_WIDTH - translation.getY() // Flip Y
          );
    }
    return translation; // Blue alliance - no change
  }

  /**
   * Flip a rotation for Red alliance.
   *
   * <p>[THE MATH] To rotate 180 deg, we negate both cos and sin components. This effectively points
   * the robot the opposite direction.
   *
   * @param rotation The rotation (Blue alliance reference)
   * @return The flipped rotation if Red, original if Blue
   */
  public static Rotation2d flipAlliance(Rotation2d rotation) {
    boolean shouldFlip =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Red;

    if (shouldFlip) {
      // Rotate 180 deg by negating cos and sin
      return new Rotation2d(-rotation.getCos(), -rotation.getSin());
    }
    return rotation;
  }

  /**
   * Flip a complete pose (position + rotation) for Red alliance.
   *
   * @param pose The pose (Blue alliance reference)
   * @return The flipped pose if Red, original if Blue
   */
  public static Pose2d flipAlliance(Pose2d pose) {
    boolean shouldFlip =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Red;

    if (shouldFlip) {
      return new Pose2d(
          // Flip position
          new Translation2d(
              FieldConstants.FIELD_LENGTH - pose.getX(), FieldConstants.FIELD_WIDTH - pose.getY()),
          // Flip rotation
          new Rotation2d(-pose.getRotation().getCos(), -pose.getRotation().getSin()));
    }
    return pose;
  }
}
