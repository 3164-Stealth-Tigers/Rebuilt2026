
package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
// import edu.wpi.first.wpilibj.DigitalInput;

public class Deploy {
  private final SparkFlex motor;
  private final RelativeEncoder deployEncoder;
  private final SparkClosedLoopController deployController;
  // private final DigitalInput limitSwitch;
  private double targetPosition;

  public Deploy(SparkFlex motor) {
    this.motor = motor;
    deployEncoder = this.motor.getEncoder();
    deployController = this.motor.getClosedLoopController();

    targetPosition = IntakeConstants.STOW_POS;

    SparkFlexConfig config = new SparkFlexConfig();
      config.idleMode(IntakeConstants.DEPLOY_COAST ? 
        IdleMode.kCoast : IdleMode.kBrake)
        .smartCurrentLimit(
          IntakeConstants.DEPLOY_STATOR_CURRENT_LIMIT, 
          IntakeConstants.DEPLOY_SUPPLY_CURRENT_LIMIT);

      config.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .p(IntakeConstants.DEPLOY_kP).i(0).d(0);

    this.motor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    // limitSwitch = new DigitalInput(IntakeConstants.DEPLOY_LIMIT_SWITCH_DIO);

      deployEncoder.setPosition(0);
  }

  public void setTargetPosition(double newPos) {
    targetPosition = newPos;
    deployController.setSetpoint(targetPosition, SparkFlex.ControlType.kPosition);
  }

  public boolean isDeployed() {
    return Math
        .abs(deployEncoder.getPosition() - IntakeConstants.DEPLOY_POS) < IntakeConstants.TOLERANCE;
  }

  public boolean isStowed() {
    return Math.abs(deployEncoder.getPosition() - IntakeConstants.STOW_POS) < IntakeConstants.TOLERANCE;
  }

  public double getRotations() {
    return deployEncoder.getPosition();
  }
}