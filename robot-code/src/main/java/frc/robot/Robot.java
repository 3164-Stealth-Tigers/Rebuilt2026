package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        // Start recording data to log file
        // This saves NetworkTables values for post-match analysis
        // Logs are stored on the roboRIO and can be viewed with AdvantageScope
        DataLogManager.start();

        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        // Log useful data to SmartDashboard for debugging
        robotContainer.logData();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void autonomousInit() {
        // Get the autonomous command selected on the dashboard
        autonomousCommand = robotContainer.getAutonomousCommand();

        // If an auto was selected, start running it
        if (autonomousCommand != null) {
            autonomousCommand.schedule(); // Add to CommandScheduler's queue
        }
    }

    @Override
    public void autonomousPeriodic() {
        // CommandScheduler handles everything in robotPeriodic()
    }

    @Override
    public void teleopInit() {
        // Stop the autonomous command when teleop starts
        // This gives the driver full control immediately
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
        // CommandScheduler handles everything in robotPeriodic()
    }

    @Override
    public void testInit() {
        // Cancel all running commands for a clean test environment
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }

}
