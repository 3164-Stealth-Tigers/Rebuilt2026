package frc.robot.scratchwork;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class OtherShooterDraftClass {
    private double x, y;
    private Alliance alliance;
    private double[] hub = new double[2];

    // Constructor for init
    public OtherShooterDraftClass() {
        UpdateHubLocation();
    }

    @SuppressWarnings("unlikely-arg-type")
    private void UpdateHubLocation() {
        hub[1] = Constants.ShooterConstants.UNIV_Y;

        hub[0] = ((DriverStation.getAlliance().equals(Alliance.Blue)) ? ShooterConstants.BLUE_X // Hub X if on Blue
                : ((DriverStation.getAlliance().equals(Alliance.Red)) ? ShooterConstants.RED_X // Hub X if on Red
                        : -180.0)); // Indicator that something's gone wrong (not on an alliance yet)
    };

    private boolean canScore() {
        // Check if we're active
        String gameData = DriverStation.getGameSpecificMessage();
        char active = (gameData.length() > 0) ? gameData.charAt(0) : 'Z';

        // Check if the robot is in its own zone
        return ((active == 'B' && alliance.equals(Alliance.Blue) && x >= ShooterConstants.BLUE_X) // Blue active
                || (active == 'R' && alliance.equals(Alliance.Red) && x <= ShooterConstants.RED_X) // Red active
        );
    }

    private Rotation2d robotToShooter() {
        // Replace 0.0 w/ eqn
        return Rotation2d.fromDegrees(0.0);
    }

    private double hubAngle() {
        UpdateHubLocation();

        x = VisionDraftClass.getX();
        y = VisionDraftClass.getY();

        Rotation2d targetAngle = new Rotation2d(hub[0] - x, hub[1] - y); // dx, dy

        return VisionDraftClass.getAngle().minus(targetAngle).minus(robotToShooter()).getDegrees();
    } // determine angle difference between robot angle and hub, in degrees

    private Optional<Double> hubDistance() {
        UpdateHubLocation();

        if (!canScore()) {
            return null;
        }

        x = VisionDraftClass.getX(); // find x coordinate of robot
        y = VisionDraftClass.getY(); // find y coordinate of robot

        Translation2d a = new Translation2d(x, y);
        Translation2d b = new Translation2d(hub[0], hub[1]);

        return Optional.of(a.getDistance(b)); // dist btwn bot & hub
    }

    private double shootVelocity() {
        Optional<Double> dx_proxy = hubDistance(); // find distance between robot and hub, in meters

        if (dx_proxy.isEmpty()) {
            return 0.0;
        }

        double dx = dx_proxy.get();
        double dy = Constants.ShooterConstants.HUB_RIM_HEIGHT
                - (VisionDraftClass.getZ() + Constants.ShooterConstants.Z_OFFSET);
        double g = Constants.ShooterConstants.G_ACCEL; // acceleration due to gravity in meters per second squared
        double theta = Math.toRadians(Constants.ShooterConstants.LAUNCH_ANGLE);

        return dx / Math.cos(theta)
                * Math.sqrt(g / 2
                        * (-dy + dx * Math.tan(theta))); // calculate
                                                         // velocity
        // needed using
        // kinematics
    } // determine velocity needed to shoot the ball into the hub, in meters per
      // second

    public void shootHub() {
        if (shootVelocity() == 0) {
            return;
        }
        while (Math.abs(hubAngle()) > 10) {
            if (hubAngle() > 0) {
                // rotate shooter clockwise??
            } else {
                // rotate shooter counterclockwise?
            }
        } // rotate the shooter until it is pointed at the hub
          // run shootVelocity() and divide by gear ratio to get number, then run motor at
          // that speed and push fuel into it
    } // shoot the fuel at the hub

    public static void shoot() {
        OtherShooterDraftClass shooter = new OtherShooterDraftClass();
        System.out.println("Velocity Needed: " + shooter.shootVelocity() + " meters per second.");
        System.out.println("Angle Change Needed: " + shooter.hubAngle() + " degrees.");
    }
}
