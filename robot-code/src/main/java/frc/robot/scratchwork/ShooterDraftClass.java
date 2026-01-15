package frc.robot.scratchwork;

public class ShooterDraftClass {
  private double x, y, hubx, huby;
  private VisionDraftClass vision = new VisionDraftClass();

  double hubAngle() {
    x = vision.visionx(); // find x coordinate of robot
    y = vision.visiony(); // find y coordinate of robot
    
    huby = 4.034663; // set universal y coordinate of hub
    if (x <= 4.625594) {
      hubx = 4.625594;
    } // determine if robot is in red alliance zone and set x coordinate of hub accordingly 
    else if (x >= 11.915394) {
      hubx = 11.915394;
    } // determine if robot is in blue alliance zone and set x coordinate of hub accordingly 
    else {
      return -180;
    } // determine if robot is in neutral zone and return invalid value, showing not to run
    return vision.visionAngle() - Math.atan((huby - y) / (hubx - x)) * 57.2957795131; // calculate hub angle, convert it to degrees, and subtract it from robot angle
  } // determine angle difference between robot angle and hub, in degrees
  
  double hubDistance() {
    x = vision.visionx(); // find x coordinate of robot
    y = vision.visiony(); // find y coordinate of robot
    huby = 4.034663; // set universal y coordinate of hub
    if (x <= 4.625594) {
      hubx = 4.625594;
    } // determine if robot is in red alliance zone and set x coordinate of hub accordingly 
    else if (x >= 11.915394) {
      hubx = 11.915394;
    } // determine if robot is in blue alliance zone and set x coordinate of hub accordingly 
    else {
      return 0;
    } // determine if robot is in neutral zone and return invalid value, showing not to run
    return Math.sqrt(Math.pow(hubx - x, 2) + Math.pow(huby - y, 2)); // calculate distance between robot and hub using pythagorean theorem
  } // determine distance between robot and hub, in meters
  
  double shootVelocity() {
    double deltax = hubDistance(); // find distance between robot and hub, in meters
    if (deltax == 0) {
      return 0;
    } // determine if robot is in neutral zone and return zero if so
    double theta = 45; // launch angle of fuel, in degrees
    double g = 9.8067; // acceleration due to gravity in meters per second squared
    double yInit = vision.visionz() + 0.5; // height of fuel at launch
    double yFinal = 1.8288; // height of the hub rim
    return deltax / Math.cos(theta * 0.01745329251) * Math.sqrt(g / 2 * (yInit - yFinal + deltax * Math.tan(theta * 0.01745329251))); // calculate velocity needed using kinematics
  } // determine velocity needed to shoot the ball into the hub, in meters per second squared
  
  public static void main(String args[]) {
    ShooterDraftClass shooter = new ShooterDraftClass();
    System.out.println("Velocity Needed: " + shooter.shootVelocity() + " meters per second.");
    System.out.println("Angle Change Needed: " + shooter.hubAngle() + " degrees.");
  }
}
