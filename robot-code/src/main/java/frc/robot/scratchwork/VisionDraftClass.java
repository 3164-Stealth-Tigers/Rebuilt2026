package frc.robot.scratchwork;

import edu.wpi.first.math.geometry.Rotation2d;

public class VisionDraftClass {
  private static double x, y, z;
  private static Rotation2d angle;

  public static double getX() {
    return x; 
  } // determine x coordinate of the robot, where x axis runs along long axis of field, in meters

  public static double getY() {
    return y;
  } // determine y coordinate of the robot, where y axis runs along short axis of field, in meters

  public static double getZ() {
    return z;
  } // determine z coordinate of the robot, where z axis is vertical, in meters

  public static Rotation2d getAngle() {
    return angle;
  } // determine angle that the robot is facing where 0 is facing away from the driver station, in degrees
}
