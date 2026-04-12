package frc.robot.subsystems.turret;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.turret.TurretState.turret_state;
import frc.robot.subsystems.vision.Vision;

public class Turret extends SubsystemBase {
    private final Vision vision;

    // Sub-subsystems
    private final Flywheel flywheel;
    private final TurretState state;
    private final Physics physics;
    private final Pitch pitch;
    private final Yaw yaw;
    private final Kicker kicker;

    public Turret(Vision vision) {
        this.vision = vision;

        flywheel = new Flywheel(new TalonFX(TurretConstants.FLYWHEEL_MOTOR_ID));

        state = new TurretState();

        pitch = new Pitch(new SparkFlex(TurretConstants.HOOD_MOTOR_ID, MotorType.kBrushless));
        pitch.reset();

        yaw = new Yaw(new SparkMax(TurretConstants.TURN_MOTOR_ID, MotorType.kBrushless));

        physics = new Physics(this.vision);

        kicker = new Kicker(new SparkFlex(TurretConstants.KICKER_MOTOR_ID, MotorType.kBrushless));
    }

    // Start shooting
    public void startFlywheel() {
        flywheel.spinUp();
        kicker.run();
        state.set(turret_state.SPINNING_UP);
    }

    // Stop shooting
    public void stopFlywheel() {
        kicker.stop();
        flywheel.stop();
        state.set(turret_state.SPINNING_DOWN);
    }

    public boolean canShoot() {
        return (state.get() == turret_state.READY);
    }

    public double getYaw() {
        return yaw.getDegrees();
    }

    public void nudgeYaw(double degrees) {
        double target = yaw.getDegrees() + degrees;
        target = Math.max(TurretConstants.MIN_YAW, Math.min(TurretConstants.MAX_YAW, target));
        yaw.moveTo(target);
    }

    public double getPitch() {
        return pitch.getDegrees();
    }

    @Override
    public void periodic() {
        // Trust pitch & yaw to auto-update, since they change slightly every loop
        if (state.get() == turret_state.SPINNING_UP && flywheel.atTargetRPM()) {
            state.set(turret_state.READY);

        } else if (state.get() == turret_state.SPINNING_DOWN
                && flywheel.getRPM() < TurretConstants.FLYWHEEL_RPM_TOLERANCE) {
            state.set(turret_state.IDLE);

        }

        // Update angles & encoders here
        yaw.moveTo(physics.getYawError() + yaw.getDegrees());
        pitch.turnTo(physics.getPitchRequired(flywheel.getRPM()));
    }

    public Flywheel getFlywheel() {
        return flywheel;
    }

    public turret_state getState() {
        return state.get();
    }
}
