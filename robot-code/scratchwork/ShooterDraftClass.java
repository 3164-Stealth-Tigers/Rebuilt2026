public class ShooterDraftClass {
  static double visionx() {
    return 2;
  }
  static double visiony() {
    return 4.034663;
  }
  static double visionz() {
    return 0;
  }
  static double visionAngle() {
    return 0;
  }
  static double hubAngle() {
    double x, y, hubx, huby;
    x = visionx();
    y = visiony();
    huby = 4.034663;
    if (x < 4.625594) {
      hubx = 4.625594;
    }
    else if (x > 11.915394) {
      hubx = 11.915394;
    }
    else {
      return 0;
    }
    return Math.atan((y - huby) / (x - hubx));
  }
  static double hubDistance() {
    double x, y, hubx, huby;
    x = visionx();
    y = visiony();
    huby = 4.034663;
    if (x < 4.625594) {
      hubx = 4.625594;
    }
    else if (x > 11.915394) {
      hubx = 11.915394;
    }
    else {
      return 0;
    }
    return Math.sqrt(Math.pow(hubx - x, 2) + Math.pow(huby - y, 2));
  }
  static double shootVelocity() {
    double deltax = hubDistance();
    if (deltax == 0) {
      return 0;
    }
    double theta = 0.785398;
    double g = 9.8067;
    double yInit = visionz() + 0.5;
    double yFinal = 1.8288;
    return deltax / Math.cos(theta) * Math.sqrt(g / 2 * (yInit - yFinal + deltax * Math.tan(theta)));
  }
  public static void main(String args[]) {
    System.out.println("Velocity Needed: " + shootVelocity() + " meters per second.");
    System.out.println("Angle Change Needed: " + hubAngle() + " radians.");
  }
}
