package frc.robot;

/*
 * ========================================================================
 * ROBOT.JAVA - THE ENTRY POINT FOR YOUR FRC ROBOT
 * ========================================================================
 *
 * START HERE! This is where everything begins when your robot powers on.
 *
 * WHAT THIS FILE DOES:
 * --------------------
 * This file is the "main" of your robot code. It controls the overall
 * program flow and tells the robot what to do during each phase of a match:
 *
 *   DISABLED   ->  AUTONOMOUS  ->  TELEOP  ->  DISABLED
 *   (waiting)     (15 sec)       (2:15)     (match over)
 *
 * HOW FRC ROBOT CODE WORKS:
 * -------------------------
 * Unlike a normal program that runs once and exits, robot code runs in
 * LOOPS. The robot constantly cycles through your code, running specific
 * methods over and over.
 *
 *   +-------------------------------------------------------------+
 *   |                    THE 20ms LOOP                            |
 *   |                                                             |
 *   |   Every 20 milliseconds (50 times per second), the robot:   |
 *   |                                                             |
 *   |   1. Reads all sensors (gyro, encoders, cameras)            |
 *   |   2. Runs robotPeriodic()                                   |
 *   |   3. Runs the current mode's periodic (autonomousPeriodic,  |
 *   |      teleopPeriodic, etc.)                                  |
 *   |   4. Sends commands to motors                               |
 *   |   5. Repeats!                                               |
 *   |                                                             |
 *   +-------------------------------------------------------------+
 *
 * WHY TIMEDROBOT?
 * ---------------
 * TimedRobot is WPILib's base class that handles the timing for us.
 * It guarantees our code runs exactly every 20ms (configurable).
 * This is important because:
 *   - Physics calculations assume a fixed time step
 *   - PID controllers need consistent timing
 *   - Sensor readings need to be regular
 *
 * COMMAND-BASED PROGRAMMING:
 * --------------------------
 * We use "Command-Based" architecture, which means:
 *   - SUBSYSTEMS: Physical mechanisms (drivetrain, shooter, etc.)
 *   - COMMANDS: Actions that use subsystems (drive forward, shoot, etc.)
 *   - SCHEDULER: Decides which commands run when
 *
 * The CommandScheduler (called in robotPeriodic) is the "brain" that
 * manages all of this automatically. You just need to:
 *   1. Create subsystems in RobotContainer
 *   2. Write commands that use those subsystems
 *   3. Bind buttons to commands
 *
 * LIFECYCLE METHODS:
 * ------------------
 * Each robot mode has two methods:
 *   - xxxInit(): Called ONCE when entering that mode
 *   - xxxPeriodic(): Called repeatedly (every 20ms) while in that mode
 *
 *   Timeline during a match:
 *
 *   [Robot Powered On]
 *        |
 *        v
 *   robotInit() -------------------------------------------------------------
 *        |
 *        v
 *   disabledInit() -> disabledPeriodic() -> disabledPeriodic() -> ...
 *        |                                                     |
 *        | (Match starts - AUTO period)                        |
 *        v                                                     |
 *   autonomousInit() -> autonomousPeriodic() -> autonomousPeriodic() -> ...
 *        |                                                     |
 *        | (AUTO ends - TELEOP period)                         |
 *        v                                                     |
 *   teleopInit() -> teleopPeriodic() -> teleopPeriodic() -> ...   |
 *        |                                                     |
 *        | (Match ends)                                        |
 *        v                                                     |
 *   disabledInit() -> disabledPeriodic() ...                    |
 *                                                              |
 *   robotPeriodic() runs ALWAYS in ALL modes ------------------+
 *
 * ========================================================================
 */

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * ======================================================================== ROBOT CLASS - The Main
 * Robot Program ========================================================================
 *
 * <p>This class extends TimedRobot, which provides the periodic execution framework. The FRC Driver
 * Station software calls these methods automatically based on the robot's current mode.
 *
 * <p>[KEY PRINCIPLE] This class should be THIN - it just delegates to RobotContainer. All the
 * interesting stuff (subsystems, commands, button bindings) lives in RobotContainer.
 */
public class Robot extends TimedRobot {

  // ================================================================
  // INSTANCE VARIABLES
  // ================================================================

  /**
   * The command that runs during autonomous.
   *
   * <p>[WHY STORE IT?] We need to cancel it when teleop starts. If we didn't store it, we couldn't
   * cancel it later.
   */
  private Command autonomousCommand;

  /**
   * The container that holds all subsystems and commands.
   *
   * <p>[WHAT IS ROBOTCONTAINER?] Think of it as the "organizer" - it creates all subsystems, sets
   * up button bindings, and knows what auto to run.
   *
   * <p>We keep Robot.java simple by putting all that logic there.
   */
  private RobotContainer robotContainer;

  // ================================================================
  // INITIALIZATION (Runs once when robot powers on)
  // ================================================================

  /**
   * Called when the robot first starts up.
   *
   * <p>[WHAT HAPPENS HERE] 1. Start data logging (records everything for later analysis) 2. Create
   * the RobotContainer (which creates all subsystems)
   *
   * <p>[DATA LOGGING] DataLogManager records robot data to a file on the roboRIO. After a match,
   * you can analyze it with AdvantageScope or WPILib's data log tool. Super helpful for debugging!
   *
   * <p>[ORDER MATTERS] RobotContainer's constructor creates all subsystems. If anything fails here
   * (like a motor not responding), you'll see it immediately.
   */
  @Override
  public void robotInit() {
    // Start logging all data - saved to /home/lvuser/logs on roboRIO
    DataLogManager.start();

    // Create the container - this sets up ALL subsystems and commands
    robotContainer = new RobotContainer();
  }

  // ================================================================
  // PERIODIC (Runs every 20ms in ALL modes)
  // ================================================================

  /**
   * Called every 20ms regardless of robot mode.
   *
   * <p>[THIS IS THE HEART OF COMMAND-BASED PROGRAMMING]
   *
   * <p>CommandScheduler.getInstance().run() does ALL of this:
   *
   * <p>1. POLL BUTTONS - Check if any buttons were pressed/released - Schedule commands that are
   * bound to those buttons
   *
   * <p>2. RUN ACTIVE COMMANDS - Call execute() on every running command - Check isFinished() to see
   * if any should stop - Call end() on finished commands
   *
   * <p>3. UPDATE SUBSYSTEM PERIODIC METHODS - Call periodic() on every registered subsystem - This
   * is where sensors get read and data gets logged
   *
   * <p>[WHY HERE AND NOT IN teleopPeriodic/autonomousPeriodic?] Commands need to run in ALL modes:
   * - In TELEOP: button-triggered commands - In AUTO: the autonomous command - In DISABLED:
   * subsystem periodic still runs for logging
   *
   * <p>By putting it here, we don't have to duplicate code.
   */
  @Override
  public void robotPeriodic() {
    // THE BIG ONE - this runs the entire command scheduler
    // Every subsystem's periodic() method
    // Every command's execute() method
    // All button checking
    CommandScheduler.getInstance().run();

    // Log telemetry data to SmartDashboard/NetworkTables
    robotContainer.logData();
  }

  // ================================================================
  // DISABLED MODE (Robot is connected but not enabled)
  // ================================================================

  /**
   * Called once when the robot enters disabled mode.
   *
   * <p>[WHEN DOES THIS HAPPEN?] - When robot first powers on - When you disable from Driver Station
   * - After autonomous ends (brief moment) - After teleop ends (match over)
   *
   * <p>[WHAT TO DO HERE] Reset any state that needs to be clean for the next enable. Currently
   * empty because we don't need any special handling.
   */
  @Override
  public void disabledInit() {
    // Nothing special needed when entering disabled
    // Commands will be cancelled automatically
  }

  /**
   * Called every 20ms while disabled.
   *
   * <p>[CAN I DO STUFF WHILE DISABLED?] Yes, but with restrictions: - CAN'T move motors - CAN read
   * sensors - CAN update dashboard - CAN do calculations
   *
   * <p>This is a good place to display diagnostic info!
   */
  @Override
  public void disabledPeriodic() {
    // Nothing special here
    // Subsystem periodic methods still run via robotPeriodic()
  }

  // ================================================================
  // AUTONOMOUS MODE (First 15 seconds of a match)
  // ================================================================

  /**
   * Called once when autonomous starts.
   *
   * <p>[WHAT HAPPENS] 1. Get the selected auto command from RobotContainer 2. Schedule it to run
   *
   * <p>[HOW AUTO SELECTION WORKS] RobotContainer has an auto chooser (either DIP switch or
   * SmartDashboard). getAutonomousCommand() returns whichever auto routine was selected.
   *
   * <p>[WHAT DOES SCHEDULE() DO?] It adds the command to the CommandScheduler's queue. The
   * scheduler will start running it on the next loop.
   */
  @Override
  public void autonomousInit() {
    // Get the auto command that was selected before the match
    autonomousCommand = robotContainer.getAutonomousCommand();

    // If an auto was selected, start running it
    if (autonomousCommand != null) {
      // schedule() tells the CommandScheduler to start running this command
      // The command will run during robotPeriodic() via the scheduler
      autonomousCommand.schedule();
    }
  }

  /**
   * Called every 20ms during autonomous.
   *
   * <p>[WHY IS THIS EMPTY?] The CommandScheduler (in robotPeriodic) handles everything! It runs the
   * auto command's execute() method and checks isFinished().
   *
   * <p>We don't need any code here because all auto logic is in the auto commands themselves.
   */
  @Override
  public void autonomousPeriodic() {
    // CommandScheduler handles everything in robotPeriodic()
    // The auto command runs automatically until it finishes
  }

  // ================================================================
  // TELEOP MODE (Driver controlled - about 2 minutes 15 seconds)
  // ================================================================

  /**
   * Called once when teleop starts.
   *
   * <p>[WHAT HAPPENS] Cancel the autonomous command so it doesn't interfere with driver control.
   *
   * <p>[WHY CANCEL?] The auto command might still be running when teleop starts. If we don't cancel
   * it, it could fight with the driver's inputs!
   *
   * <p>Example: Auto is driving forward, driver wants to go backward. Without cancelling, they'd
   * fight each other.
   */
  @Override
  public void teleopInit() {
    // Stop the autonomous command when teleop starts
    // This gives the driver full control immediately
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /**
   * Called every 20ms during teleop.
   *
   * <p>[WHY IS THIS EMPTY?] All teleop control is handled by: 1. DEFAULT COMMANDS on subsystems
   * (like the drive command) 2. BUTTON BINDINGS that trigger commands
   *
   * <p>These are all set up in RobotContainer and run automatically via the CommandScheduler in
   * robotPeriodic().
   */
  @Override
  public void teleopPeriodic() {
    // CommandScheduler handles everything in robotPeriodic()
    // Driver commands run automatically based on button bindings
  }

  // ================================================================
  // TEST MODE (For testing individual mechanisms)
  // ================================================================

  /**
   * Called once when entering test mode.
   *
   * <p>[WHAT IS TEST MODE?] A special mode for testing mechanisms without the constraints of normal
   * operation. You can enable it from the Driver Station.
   *
   * <p>[WHY CANCEL ALL?] We want a clean slate for testing. Any running commands could interfere
   * with our tests.
   */
  @Override
  public void testInit() {
    // Cancel all running commands for a clean test environment
    CommandScheduler.getInstance().cancelAll();
  }

  /** Called every 20ms during test mode. */
  @Override
  public void testPeriodic() {
    // Add test code here if needed
  }

  // ================================================================
  // SIMULATION MODE (For testing without a real robot)
  // ================================================================

  /**
   * Called once when simulation starts.
   *
   * <p>[WHAT IS SIMULATION?] WPILib can simulate robot code on your computer. This is great for
   * testing code without a physical robot!
   */
  @Override
  public void simulationInit() {
    // Add simulation-specific initialization here
  }

  /** Called every 20ms during simulation. */
  @Override
  public void simulationPeriodic() {
    // Add simulation-specific updates here
  }
} // End of Robot class
