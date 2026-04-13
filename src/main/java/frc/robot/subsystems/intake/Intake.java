package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeState.intake_state;

public class Intake extends SubsystemBase {
  private final Deploy deploy;
  private final IntakeState state_machine;
  private final Roller roller;

  public Intake() {
    state_machine = new IntakeState();
    deploy = new Deploy(
        new SparkFlex(IntakeConstants.DEPLOY_MOTOR_ID, MotorType.kBrushless));
    roller =
        new Roller(
            new TalonFX(IntakeConstants.ROLLER_MOTOR_ID),
            state_machine,
            deploy);
  }

  public Deploy getD() {
    return deploy;
  }

  public IntakeState getS() { // Unused
    return state_machine;
  }

  public Roller getR() {
    return roller;
  }

  public void deployIntakeMechanism() {
    if (state_machine.get() == intake_state.DEPLOYING
        || state_machine.get() == intake_state.DEPLOYED) return;

    deploy.setTargetPosition(IntakeConstants.DEPLOY_POS);
    state_machine.set(intake_state.DEPLOYING);
  }

  public void retractIntakeMechanism() {
    if (state_machine.get() == intake_state.RETRACTING
        || state_machine.get() == intake_state.STOWED) return;
    roller.stop();
    deploy.setTargetPosition(IntakeConstants.STOW_POS);
    state_machine.set(intake_state.RETRACTING);
  }

  @Override
  public void periodic() {
    state_machine.update(deploy.isDeployed(), deploy.isStowed());
  }
}
