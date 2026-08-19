package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Hardware {
    private final Pigeon2 gyro;
    private final SwerveModule[] modules;
    private final Translation2d[] locations;

    public Hardware(Pigeon2 new_gyro) {
        gyro = new_gyro;
        this.gyro.reset();

        locations = new Translation2d[] { // (+, +) is (Front, Left)
                new Translation2d(SwerveConstants.WHEEL_BASE / 2, SwerveConstants.TRACK_WIDTH / 2),
                new Translation2d(SwerveConstants.WHEEL_BASE / 2, -SwerveConstants.TRACK_WIDTH / 2),
                new Translation2d(-SwerveConstants.WHEEL_BASE / 2, SwerveConstants.TRACK_WIDTH / 2),
                new Translation2d(-SwerveConstants.WHEEL_BASE / 2, -SwerveConstants.TRACK_WIDTH / 2)
        };

        modules = new SwerveModule[] {
                new SwerveModule(
                        0,
                        SwerveConstants.FL_DRIVE_ID,
                        SwerveConstants.FL_AZIMUTH_ID,
                        SwerveConstants.FL_DRIVE_INVERT,
                        SwerveConstants.FL_AZIMUTH_INVERT),
                new SwerveModule(
                        1,
                        SwerveConstants.FR_DRIVE_ID,
                        SwerveConstants.FR_AZIMUTH_ID,
                        SwerveConstants.FR_DRIVE_INVERT,
                        SwerveConstants.FR_AZIMUTH_INVERT),
                new SwerveModule(
                        2,
                        SwerveConstants.RL_DRIVE_ID,
                        SwerveConstants.RL_AZIMUTH_ID,
                        SwerveConstants.BL_DRIVE_INVERT,
                        SwerveConstants.BL_AZIMUTH_INVERT),
                new SwerveModule(
                        3,
                        SwerveConstants.RR_DRIVE_ID,
                        SwerveConstants.RR_AZIMUTH_ID,
                        SwerveConstants.BR_DRIVE_INVERT,
                        SwerveConstants.BR_AZIMUTH_INVERT)
        };
    }

    public Translation2d[] getModuleLocations() {
        return locations;
    }

    public SwerveModule[] getModules() {
        return modules;
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getModulePosition();
        }
        return positions;
    }

    public SwerveModule getModule(int i) {
        try {
            return modules[i];
        } catch (Exception e) {
            System.out.println("\n Failed getting swerve module " + i);
            e.printStackTrace();
            System.out.println("\nReturning first swerve module");
            return modules[0];
        }
    }

    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public void setYaw(Rotation2d angle) {
        gyro.setYaw(angle.getDegrees());
    }

    public void skiStop(boolean open_loop) {
        modules[0].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), open_loop); // FL
        modules[1].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), open_loop); // FR
        modules[2].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), open_loop); // RL
        modules[3].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), open_loop); // RR
    }

    public void hardStop() {
        for (SwerveModule module : modules) {
            module.stop();
        }
    }

    /**
     * Re-zeros every module's azimuth relative encoder to "forward". Only call this while the
     * wheels are physically pointed forward (no CANcoder to correct a bad zero automatically).
     */
    public void rezeroAzimuth() {
        for (SwerveModule module : modules) {
            module.resetToForward();
        }
    }

    public void resetWheelsForward(boolean open_loop) {
        SwerveModuleState forward = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        for (SwerveModule module : modules) {
            module.setDesiredState(forward, open_loop);
        }
    }
}