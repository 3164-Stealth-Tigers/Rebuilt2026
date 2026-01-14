package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.OI.DriverActionSet;
import frc.robot.OI.XboxDriver;
import frc.robot.commands.SwerveCommands;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swerve.SwerveDrive;

/*
 * ============================================================================
 * ROBOTCONTAINER.JAVA - The Robot's Command Center
 * ============================================================================
 *
 * WHAT THIS FILE DOES:
 * This is where everything comes together! RobotContainer:
 * 1. Creates all the subsystems (elevator, arm, drivetrain, etc.)
 * 2. Maps controller buttons to robot actions
 * 3. Sets up autonomous routines
 *
 * Think of it like the main control panel that wires everything together.
 *
 * ============================================================================
 * QUICK REFERENCE - Common Tasks
 * ============================================================================
 *
 * ADD A NEW BUTTON BINDING:
 *   → Go to configureButtonBindings() method
 *   → Add: joystick.buttonName().onTrue(yourCommand);
 *
 * CHANGE WHICH CONTROLLER IS USED:
 *   → Find the controller initialization in the constructor
 *   → Change XboxDriver to PS4Driver, etc.
 *
 * ADD A NEW AUTONOMOUS ROUTINE:
 *   → Create a new build___Auto() method
 *   → Add it to the autoChooser
 *
 * CHANGE LEVEL HEIGHTS/ANGLES:
 *   → Don't change here! Go to Constants.java > ScoringConstants
 *
 * ============================================================================
 * HOW BUTTON BINDINGS WORK
 * ============================================================================
 *
 * Button bindings connect controller inputs to robot commands:
 *
 *   button.onTrue(command)    → Run command ONCE when button is pressed
 *   button.whileTrue(command) → Run command WHILE button is held
 *   button.toggleOnTrue()     → Toggle command on/off each press
 *
 * Example:
 *   driverJoystick.resetGyro().onTrue(swerve.resetGyroCommand());
 *   ↑ controller ↑ button     ↑ action      ↑ what happens
 *
 * ============================================================================
 */
public class RobotContainer {

    // ========================================================================
    // CONTROLLERS
    // ========================================================================
    // These interfaces define what inputs we need from each controller.
    // The actual controller type (Xbox, PS4, etc.) is set in the constructor.
    // ========================================================================
    private final DriverActionSet driverJoystick;       // Port 0: Moves the robot

    // ========================================================================
    // SUBSYSTEMS - The physical parts of the robot
    // ========================================================================
    // Each subsystem controls one mechanism. They're created here and used
    // throughout this file to make commands and button bindings.
    // ========================================================================
    private final SwerveDrive swerve;      // Drivetrain - moves the robot around
    private final Vision vision;           // Camera system for tracking targets

    // ========================================================================
    // SUPERSTRUCTURE - Coordinates multiple subsystems
    // ========================================================================
    private final Superstructure superstructure;

    // ========================================================================
    // AUTONOMOUS CHOOSER
    // ========================================================================
    // Lets drivers select which auto routine to run from SmartDashboard
    private final SendableChooser<Command> autoChooser;

    // ========================================================================
    // STATE VARIABLES
    // ========================================================================
    private boolean useAutomation = true;  // Auto-score when ready?
    private int speedExponent = 2;         // Joystick curve (1=linear, 2=squared)

    // ========================================================================
    // CONSTRUCTOR - Sets up everything when robot starts
    // ========================================================================

    /**
     * Creates the RobotContainer. This runs once when the robot powers on.
     *
     * ORDER MATTERS! We create things in this order:
     * 1. Silence warnings (so driver station doesn't spam errors)
     * 2. Create controllers
     * 3. Create auto chooser
     * 4. Create subsystems
     * 5. Set up driving command
     * 6. Build auto routines
     * 7. Configure button bindings
     */
    public RobotContainer() {
        // Prevents "joystick not connected" warnings during setup
        DriverStation.silenceJoystickConnectionWarning(true);

        // --------------------------------------------------------------------
        // STEP 1: CREATE CONTROLLERS
        // --------------------------------------------------------------------
        // Change these to use different controller types:
        // - XboxDriver, PS4Driver, T16000MDriver (for driver)
        // - XboxOperator (for operator)
        // - ArcadeScoringPositions, PS4ScoringPositions, KeyboardScoringPositions
        // --------------------------------------------------------------------
        driverJoystick = new XboxDriver(0);           // Driver uses Xbox on port 0

        // --------------------------------------------------------------------
        // STEP 2: CREATE AUTO CHOOSER
        // --------------------------------------------------------------------
        // This dropdown appears in SmartDashboard for selecting auto modes
        autoChooser = new SendableChooser<>();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // --------------------------------------------------------------------
        // STEP 3: CREATE SUBSYSTEMS
        // --------------------------------------------------------------------
        // Each subsystem manages one part of the robot hardware
        vision = new Vision();
        swerve = new SwerveDrive();
        superstructure = new Superstructure(swerve);

        // --------------------------------------------------------------------
        // STEP 4: SET UP DRIVING (runs continuously in teleop)
        // --------------------------------------------------------------------
        // This command reads joystick inputs and drives the robot
        Command teleopDriveCommand = swerve.teleopCommand(
            () -> applySpeedCurve(driverJoystick.forward()),  // Forward/backward
            () -> applySpeedCurve(driverJoystick.strafe()),   // Left/right
            () -> applySpeedCurve(driverJoystick.turn())      // Rotation
        );
        swerve.setDefaultCommand(teleopDriveCommand);

        // Start facing away from driver station (180 degrees)
        swerve.resetYaw(Rotation2d.fromDegrees(180));

        // Put subsystems on SmartDashboard for debugging
        SmartDashboard.putData("TeleOp Command", teleopDriveCommand);

        // --------------------------------------------------------------------
        // STEP 5: BUILD AUTO ROUTINES
        // --------------------------------------------------------------------
        // buildForwardAuto();

        // --------------------------------------------------------------------
        // STEP 6: CONFIGURE BUTTON BINDINGS
        // --------------------------------------------------------------------
        configureButtonBindings();

        // --------------------------------------------------------------------
        // STEP 7: AUTOMATIC SCORING TRIGGER
        // --------------------------------------------------------------------
        // When everything is ready and automation is enabled, auto-score!
        /* new Trigger(() -> superstructure.readyToScore() && DriverStation.isTeleop() && useAutomation)
            .whileTrue(claw.outtakeCommand()); */
    }

    // ========================================================================
    // HELPER METHODS
    // ========================================================================

    /**
     * Applies a curve to joystick input for better control feel.
     *
     * WHY USE THIS?
     * Raw joystick input is linear: 50% stick = 50% speed.
     * With squared input: 50% stick = 25% speed (easier precision at low speeds)
     *
     * @param input Raw joystick value (-1 to 1)
     * @return Curved value (-1 to 1)
     */
    private double applySpeedCurve(double input) {
        // Math.pow gives us exponential curve
        // Math.signum keeps the positive/negative sign
        return Math.pow(Math.abs(input), speedExponent) * Math.signum(input);
    }

    private void configureButtonBindings() {

        // ====================================================================
        // DRIVER CONTROLS - Moving the robot
        // ====================================================================

        // Reset gyro to 180° (robot facing away from driver)
        // Press START button to recalibrate "forward" direction
        driverJoystick.resetGyro().onTrue(swerve.resetGyroCommand());

        // Toggle between field-relative and robot-relative driving
        // Field-relative: push forward = robot goes toward opposing alliance
        // Robot-relative: push forward = robot goes where it's facing
        driverJoystick.toggleFieldRelative().onTrue(
            new InstantCommand(swerve::toggleFieldRelative)
        );

        // Ski stop: lock wheels in X pattern to resist being pushed
        // Hold Y button to engage, releases when driver starts moving
        driverJoystick.skiStop().onTrue(
            SwerveCommands.skiStopCommand(swerve).until(driverJoystick::isMovementCommanded)
        );

        // Toggle between squared input (precise) and linear input (fast)
        // Press X to switch between modes
        driverJoystick.toggleSpeed().onTrue(
            new InstantCommand(() -> speedExponent = (speedExponent == 1) ? 2 : 1)
        );
     }

    // ========================================================================
    // AUTONOMOUS ROUTINES
    // ========================================================================
    // Each method creates an auto routine and adds it to the chooser.
    // Drivers select which one to run from SmartDashboard before the match.
    // ========================================================================

    /**
     * Simple auto that just drives forward for 2 seconds.
     * Good for testing or when you just need to leave the starting zone.
     */
    // private void buildForwardAuto() {
    //     Command auto = Commands.run(
    //         // Drive forward at 1.5 m/s, no strafe, no rotation
    //         () -> swerve.drive(new Translation2d(1.5, 0), 0, false, false),
    //         swerve
    //     ).andThen(
    //         // Stop after driving
    //         Commands.run(() -> swerve.drive(new Translation2d(), 0, false, false), swerve)
    //     ).withTimeout(2);  // Total runtime: 2 seconds

    //     // Add to the auto chooser
    //     autoChooser.setDefaultOption("Drive Forward", auto);
    // }

    // ========================================================================
    // PUBLIC ACCESSORS
    // ========================================================================
    // Methods that other classes can call to get information or subsystems
    // ========================================================================

    /**
     * Gets the autonomous command selected from the dashboard.
     * Called by Robot.java when autonomous period starts.
     *
     * @return The selected autonomous command
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * Logs telemetry data to SmartDashboard.
     * Called every loop by Robot.java.
     */
    public void logData() {
        SmartDashboard.putBoolean("Slow Speed", speedExponent == 2);
        SmartDashboard.putBoolean("Use Automation", useAutomation);
    }

    /**
     * Gets the vision subsystem (for testing).
     *
     * @return The vision subsystem
     */
    public Vision getVision() {
        return vision;
    }
}
