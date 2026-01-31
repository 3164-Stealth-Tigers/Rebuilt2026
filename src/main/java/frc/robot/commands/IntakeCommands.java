package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;

/**
 * Command factory for intake-related commands.
 *
 * Provides reusable command compositions for intake operations:
 * - Deploy and intake FUEL
 * - Retract to stowed position
 * - Outtake FUEL
 * - Feed FUEL to shooter
 * - Continuous intake sequences
 */
public final class IntakeCommands {

    private IntakeCommands() {
        // Utility class - prevent instantiation
    }

    // ================================================================
    // BASIC OPERATIONS
    // ================================================================

    /**
     * Deploy intake, run rollers until FUEL detected, then retract.
     *
     * @param intake The intake subsystem
     * @return Command that completes a full intake cycle
     */
    public static Command intakeFuelCommand(Intake intake) {
        return Commands.sequence(
            intake.intakeCommand(),
            intake.retractCommand()
        ).withName("Intake and Retract");
    }

    /**
     * Deploy intake and run continuously until cancelled.
     *
     * @param intake The intake subsystem
     * @return Command that runs until interrupted
     */
    public static Command continuousIntakeCommand(Intake intake) {
        return Commands.sequence(
            Commands.runOnce(intake::deploy, intake),
            Commands.waitUntil(intake::isDeployed),
            Commands.run(intake::runIntake, intake)
        ).finallyDo(interrupted -> {
            intake.stopRollers();
            intake.retract();
        }).withName("Continuous Intake");
    }

    /**
     * Outtake FUEL while button is held.
     *
     * @param intake The intake subsystem
     * @return Command that runs outtake until released
     */
    public static Command outtakeCommand(Intake intake) {
        return Commands.sequence(
            Commands.runOnce(intake::deploy, intake),
            Commands.waitUntil(intake::isDeployed),
            Commands.run(intake::runOuttake, intake)
        ).finallyDo(interrupted -> {
            intake.stopRollers();
            intake.retract();
        }).withName("Outtake");
    }

    // ================================================================
    // SHOOTER INTEGRATION
    // ================================================================

    /**
     * Feed FUEL to the shooter.
     *
     * @param intake The intake subsystem
     * @return Command that feeds FUEL
     */
    public static Command feedCommand(Intake intake) {
        return Commands.startEnd(
            intake::feedToShooter,
            intake::stopRollers,
            intake
        ).withName("Feed FUEL");
    }

    // ================================================================
    // UTILITY COMMANDS
    // ================================================================

    /**
     * Emergency stop: stop all intake motors immediately.
     *
     * @param intake The intake subsystem
     * @return Instant command that stops everything
     */
    public static Command emergencyStopCommand(Intake intake) {
        return Commands.runOnce(() -> {
            intake.stopRollers();
            intake.retract();
        }, intake).withName("Intake Emergency Stop");
    }

}
