package frc.robot.subsystems.turret;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class Kicker {
    private final SparkFlex motor;

    public Kicker(SparkFlex motor) {
        this.motor = motor;

        SparkFlexConfig config = new SparkFlexConfig();
        config.disableFollowerMode();
        config.idleMode(TurretConstants.KICKER_COAST ?
            IdleMode.kCoast : IdleMode.kBrake);
        config.smartCurrentLimit(
            TurretConstants.KICKER_STATOR_CURRENT_LIMIT,
            TurretConstants.KICKER_SUPPLY_CURRENT_LIMIT);
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        this.motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void run() {
        int Invert = TurretConstants.KICKER_SPIN_REVERSE ? 
            -1 : 1;
        motor.set(Invert * (TurretConstants.KICKER_SPIN_SPEED / TurretConstants.KICKER_SPIN_SPEED_MAX)); 
    }

    public void stop() {
        motor.set(0);
    }
}
