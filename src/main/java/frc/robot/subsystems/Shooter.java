package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * <h2>Shooter Class</h2>
 * <hr>
 * The shooter subsystem
 * <hr>
 * <b> Methods </b>
 * 
 * @apiNote Shooter()
 * @apiNote UpdateHubLocation()
 * @apiNote canScore()
 * @apiNote robotToShooter()
 * @apiNote robotToHub()
 * @apiNote hubDistance()
 * @apiNote getVelocity()
 * @apiNote shootHub()
 * @apiNote shoot()
 */
public class Shooter extends SubsystemBase {
    private double x, y, z;
    private Rotation2d heading;
    private Alliance alliance;
    private double[] hub = new double[2];

    private final Vision vision;
    private final SwerveDrive swerve;

    /**
     * <b> Shooter Class Constructor </b>
     * <hr>
     * 
     * @apiNote Constructor for initialization
     */
    public Shooter(Vision vision, SwerveDrive swerve) {
        this.vision = vision;
        this.swerve = swerve;
        UpdateHubLocation();
    }

    /**
     * <b> Get Robot Pose Method </b>
     * <hr>
     * 
     * @return Robot's current position & heading from odometry
     */
    private Pose2d getRobotPose() {
        return swerve.getPose();
    }

    /**
     * <b> Update Hub Location Method </b>
     * <hr>
     * 
     * @apiNote Updates the x and y values of the hub depending on our alliance
     * @apiNote Sets X to -180.0° if we're not on an alliance, signaling catastrophe
     */
    @SuppressWarnings("unlikely-arg-type")
    private void UpdateHubLocation() {
        hub[1] = ShooterConstants.UNIV_Y;

        hub[0] = ((DriverStation.getAlliance().equals(Alliance.Blue)) ? ShooterConstants.BLUE_X // Hub X if on Blue
                : ((DriverStation.getAlliance().equals(Alliance.Red)) ? ShooterConstants.RED_X // Hub X if on Red
                        : -180.0)); // Indicator that something's gone wrong (not on an alliance yet)
    };

    /**
     * <b> Can-Score? Method </b>
     * <hr>
     * 
     * @param active   There is an active alliance according to provided game data
     * @param alliance Our alliance is the active alliance
     * @param position We are in our alliance's zone
     * 
     * @return Whether we are able to score right now, based on whether all
     *         conditions are met
     */
    private boolean canScore() {
        // Check if we're active
        String gameData = DriverStation.getGameSpecificMessage();
        char active = (gameData.length() > 0) ? gameData.charAt(0) : 'Z';

        // Check if the robot is in its own zone
        return ((active == 'B' && alliance.equals(Alliance.Blue) && x >= ShooterConstants.BLUE_X) // Blue active
                || (active == 'R' && alliance.equals(Alliance.Red) && x <= ShooterConstants.RED_X) // Red active
        );
    }

    /**
     * <b>Robot to Shooter Method</b>
     * <hr>
     * 
     * @return Current angle difference between the robot and the shooter
     */
    private Rotation2d robotToShooter() {
        // Replace 0.0 w/ eqn (placeholder TODO)
        return Rotation2d.fromDegrees(0.0);
    }

    /**
     * <b>Robot to Hub Method </b>
     * <hr>
     * 
     * @return Angle difference between the robot and the hub in degrees
     */
    private double robotToHub() {
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

        Rotation2d targetAngle = new Rotation2d(hub[0] - x, hub[1] - y); // dx, dy

        return heading.minus(targetAngle).minus(robotToShooter()).getDegrees();
    }

    /**
     * <b> Hub Distance Method </b>
     * <hr>
     * 
     * @return Distance between bot and hub, if and only if scoring is an option
     */
    private Optional<Double> hubDistance() {
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
     * <b> Get Velocity Method </b>
     * <hr>
     * 
     * @return Velocity necessary to shoot the ball into the center of the hub from
     *         the robot's position.
     */
    private double getVelocity() {
        Optional<Double> dx_proxy = hubDistance(); // find distance between robot and hub, in meters

        if (dx_proxy.isEmpty()) {
            return 0.0;
        }
        Optional<Pose3d> poseOpt = vision.getPose3d(getRobotPose());
        if (poseOpt.isEmpty()) {
            return 0.0;
        }

        Pose3d pose = poseOpt.get();
        x = pose.getX();
        y = pose.getY();
        z = pose.getZ();
        heading = pose.getRotation().toRotation2d();

        double dx = dx_proxy.get();
        double dy = ShooterConstants.HUB_RIM_HEIGHT
                - (z + ShooterConstants.Z_OFFSET);
        double g = ShooterConstants.G_ACCEL; // acceleration due to gravity in meters per second squared
        double theta = Math.toRadians(ShooterConstants.LAUNCH_ANGLE);

        return dx / Math.cos(theta)
                * Math.sqrt(g / 2
                        * (-dy + dx * Math.tan(theta)));
    }

    /**
     * <b> Shoot Hub Method </b>
     * <hr>
     * 
     * @apiNote Turns the shooter towards the hub and fires continuously
     */
    public void shootHub() {
        if (getVelocity() == 0) {
            return;
        }

        while (Math.abs(robotToHub()) > ShooterConstants.ANGLE_TOLERANCE) {
            if (robotToHub() > 0) {
                // rotate shooter clockwise??
            } else {
                // rotate shooter counterclockwise?
            }
        } // rotate the shooter until it is pointed at the hub
          // run shootVelocity() and divide by gear ratio to get number, then run motor at
          // that speed and push fuel into it
    } // shoot the fuel at the hub

    /**
     * <i> Shoot Method </i>
     * 
     * @deprecated Use superstructure instead
     *             <hr>
     * 
     * @apiNote A test method for calculating values off-handedly
     */
    protected static void shoot() {
        Shooter shooter = new Shooter(new Vision(), new SwerveDrive());
        System.out.println("Velocity Needed: " + shooter.getVelocity() + " meters per second.");
        System.out.println("Angle Change Needed: " + shooter.robotToHub() + " degrees.");
    }
}