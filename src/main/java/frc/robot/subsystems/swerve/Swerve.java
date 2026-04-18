package frc.robot.subsystems.swerve;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.TalonFXSimState;


import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Swerve extends SubsystemBase {
    // Dependencies
    // private final Vision vision;

    // Sub-subsystems
    private final Config config;
    private final SwerveDriveKinematics kinematics;
    private final Hardware hardware;
    private final Dashboard dashboard;
    private final SwerveDrivePoseEstimator estimator;

    public Swerve() {
    // public Swerve(Vision vision) {
        // this.vision = vision;

        // Sub-subsystems
        config = new Config();
        dashboard = new Dashboard();
        hardware = new Hardware(new Pigeon2(SwerveConstants.PIGEON_ID));
        kinematics = new SwerveDriveKinematics(hardware.getModuleLocations());
        estimator = new SwerveDrivePoseEstimator(
                kinematics,
                hardware.getYaw(),
                hardware.getPositions(),
                new Pose2d() // (0,0) & forward direction
        );
    }

    public void setFieldRelative(boolean b) {
        config.field_relative = b;
    }

    

    public void setOpenLoop(boolean b) {
        config.open_loop = b;
    }

    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        estimator.resetPosition(hardware.getYaw(), hardware.getPositions(), pose);
    }

    public void addVisionMeasurement(Pose2d pose, double timestampSeconds) {
        estimator.addVisionMeasurement(pose, timestampSeconds);
    }

    public boolean isFieldRelative() {
        return config.field_relative;
    }

    public void skiStop() {
        hardware.skiStop(config.open_loop);
    }

    @Override
    public void periodic() {
        // vision.getP().invalidateCache();

        // vision.getBestVisionUpdateRaw(getPose()).ifPresent((update) -> {
        //     // Standard deviation for distance error
        //     double xyStdDev = SwerveConstants.XY_BASE_STDDEV
        //             + (Math.pow(update.avgDistanceMeters(), 2) * SwerveConstants.XY_DIST_FACTOR);
        //     double thetaStdDev = SwerveConstants.HEADING_BASE_STDDEV
        //             + (Math.pow(update.avgDistanceMeters(), 2) * SwerveConstants.HEADING_DIST_FACTOR);

        //     estimator.addVisionMeasurement(
        //             update.pose2d(),
        //             update.timestampSeconds(),
        //             VecBuilder.fill(xyStdDev, xyStdDev, Units.degreesToRadians(thetaStdDev)));
        // });
        dashboard.updateLogs(getPose(), hardware.getStates());
    }
    private final DCMotorSim m_motorSimModel = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor., 0))

    @Override
    public void simulationPeriodic() {
        dashboard.updateLogs(getPose(), hardware.getStates());
        var talonFxSim =  hardware.getModule(0).GetDriveMotor().getSimState();
        talonFxSim.setSupplyVoltage(12);
        var motorVoltage = talonFxSim.getMotorVoltageMeasure();
        m_motorSimModel.setInputVoltage(motorVoltage.in(Volts));
        m_motorSimModel.update(0.020);
        
    }

    public void zeroHeading() {
        estimator.resetPosition(
                hardware.getYaw(),
                hardware.getPositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public void zeroGyro() {
        hardware.setYaw(new Rotation2d());
    }

    // Driving methods
    public Command teleopCommand(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn) {
        return new RunCommand(
                () -> { // WPIlib autoflips coord sys when alliance color changes
                    double vx = forward.getAsDouble() * SwerveConstants.MAX_SPEED;
                    double vy = strafe.getAsDouble() * SwerveConstants.MAX_SPEED;
                    double omega = turn.getAsDouble() * SwerveConstants.MAX_ANGULAR_VELOCITY;

                    drive(new Translation2d(vx, vy), omega);
                },
                this);
    }

    public void drive(Translation2d translation, double rotation) {
        ChassisSpeeds speeds;

        if (config.field_relative == true) { // Field-relative means forward direction is always @ far end of field
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation,
                    hardware.getYaw());
        } else { // Robo-relative means forward direction is always @ the front of the robot
            speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        }

        drive(speeds);
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(hardware.getStates());
    }

    public void drive(ChassisSpeeds speeds) {
        speeds = ChassisSpeeds.discretize(speeds, 0.02);

        var states = kinematics.toSwerveModuleStates(speeds);
        var modules = hardware.getModules();

        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_SPEED);

        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(states[i], config.open_loop);
        }
    }

    // Sub-subsystem getters
    public Hardware getH() {
        return hardware;
    }

    public SwerveDriveKinematics getK() {
        return kinematics;
    }
}
