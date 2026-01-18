package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

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
     * Feed a single FUEL to the shooter.
     *
     * @param intake The intake subsystem
     * @return Command that feeds one FUEL
     */
    public static Command feedOneCommand(Intake intake) {
        return Commands.sequence(
            Commands.runOnce(intake::feedToShooter, intake),
            // Wait for FUEL to clear the sensor
            Commands.waitUntil(intake::hasFuel),
            Commands.waitUntil(() -> !intake.hasFuel()),
            Commands.runOnce(() -> {
                intake.stopRollers();
                intake.decrementFuelCount();
            }, intake)
        ).withTimeout(1.0) // Safety timeout
         .withName("Feed One FUEL");
    }

    /**
     * Feed all stored FUEL to the shooter.
     *
     * @param intake The intake subsystem
     * @param shooter The shooter subsystem (for coordination)
     * @return Command that feeds all FUEL
     */
    public static Command feedAllCommand(Intake intake, Shooter shooter) {
        return Commands.sequence(
            // Run feeder until empty
            Commands.run(intake::feedToShooter, intake)
                .until(() -> intake.getFuelCount() <= 0)
                .withTimeout(10.0), // Safety timeout
            Commands.runOnce(intake::stopRollers, intake)
        ).withName("Feed All FUEL");
    }

    // ================================================================
    // COMBINED SEQUENCES
    // ================================================================

    /**
     * Full intake sequence: deploy, collect FUEL, retract, and prepare for shooting.
     *
     * @param intake The intake subsystem
     * @return Command for complete intake operation
     */
    public static Command fullIntakeSequence(Intake intake) {
        return Commands.sequence(
            // Deploy
            Commands.runOnce(intake::deploy, intake),
            Commands.waitUntil(intake::isDeployed),
            // Collect until FUEL detected
            Commands.run(intake::runIntake, intake)
                .until(intake::hasFuel),
            // Stop and count
            Commands.runOnce(() -> {
                intake.stopRollers();
                intake.incrementFuelCount();
            }, intake),
            // Retract
            Commands.runOnce(intake::retract, intake),
            Commands.waitUntil(intake::isStowed)
        ).withName("Full Intake Sequence");
    }

    /**
     * Collect multiple FUEL: intake until count reached or timeout.
     *
     * @param intake The intake subsystem
     * @param count Number of FUEL to collect
     * @param timeoutSeconds Maximum time to wait
     * @return Command that collects specified number of FUEL
     */
    public static Command collectMultipleCommand(Intake intake, int count, double timeoutSeconds) {
        int targetCount = intake.getFuelCount() + count;
        return Commands.sequence(
            Commands.runOnce(intake::deploy, intake),
            Commands.waitUntil(intake::isDeployed),
            Commands.run(intake::runIntake, intake)
                .until(() -> intake.getFuelCount() >= targetCount),
            Commands.runOnce(intake::stopRollers, intake),
            Commands.runOnce(intake::retract, intake),
            Commands.waitUntil(intake::isStowed)
        ).withTimeout(timeoutSeconds)
         .withName("Collect " + count + " FUEL");
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

    /**
     * Reset fuel count to zero.
     *
     * @param intake The intake subsystem
     * @return Instant command that resets count
     */
    public static Command resetFuelCountCommand(Intake intake) {
        return Commands.runOnce(intake::resetFuelCount, intake)
            .withName("Reset Fuel Count");
    }
}
