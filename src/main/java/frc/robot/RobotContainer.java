package frc.robot;

/*
 * ========================================================================
 * ROBOTCONTAINER.JAVA - THE ORGANIZER
 * ========================================================================
 *
 * READ THIS AFTER Robot.java!
 *
 * WHAT THIS FILE DOES:
 * --------------------
 * RobotContainer is like a "factory" that builds everything the robot needs:
 *
 *   1. CREATES all subsystems (drivetrain, shooter, intake, etc.)
 *   2. SETS UP button bindings (what happens when you press A, B, etc.)
 *   3. CONFIGURES autonomous mode selection
 *   4. PROVIDES the autonomous command to Robot.java
 *
 * WHY A SEPARATE FILE?
 * --------------------
 * We could put all this in Robot.java, but that would be messy!
 * Separation of concerns:
 *   - Robot.java: Handles the lifecycle (init, periodic, mode changes)
 *   - RobotContainer: Sets up what the robot CAN do
 *   - Commands: Define HOW to do specific actions
 *   - Subsystems: Control physical mechanisms
 *
 * COMMAND-BASED ARCHITECTURE:
 * ---------------------------
 *
 *   ┌─────────────────────────────────────────────────────────────────┐
 *   │                     THE BIG PICTURE                             │
 *   │                                                                 │
 *   │   ROBOTCONTAINER                                                │
 *   │        │                                                        │
 *   │        ├──► Creates SUBSYSTEMS (swerve, shooter, intake, etc.)  │
 *   │        │         │                                              │
 *   │        │         └──► Subsystems have DEFAULT COMMANDS          │
 *   │        │              that run when nothing else is using them  │
 *   │        │                                                        │
 *   │        └──► Sets up BUTTON BINDINGS                             │
 *   │             │                                                   │
 *   │             └──► Buttons trigger COMMANDS                       │
 *   │                  │                                              │
 *   │                  └──► Commands use SUBSYSTEMS                   │
 *   │                                                                 │
 *   └─────────────────────────────────────────────────────────────────┘
 *
 * SUBSYSTEMS vs COMMANDS:
 * -----------------------
 *
 *   SUBSYSTEM = A physical mechanism
 *   ─────────
 *   Examples: SwerveDrive, Shooter, Intake, Climber
 *   Contains: Motors, sensors, and methods to control them
 *   Rule: Only ONE command can use a subsystem at a time!
 *
 *   COMMAND = An action that uses subsystems
 *   ───────
 *   Examples: DriveForward, ShootBall, IntakeCargo
 *   Contains: Logic for what to do (initialize, execute, end)
 *   Rule: Must "require" any subsystems it uses
 *
 * DEFAULT COMMANDS:
 * -----------------
 * Each subsystem can have a DEFAULT command that runs when nothing else
 * is using that subsystem.
 *
 *   Example: SwerveDrive's default command is the teleop drive command
 *   - When teleop starts, nothing is using swerve
 *   - So the default command (teleop driving) runs automatically
 *   - If you press a button that uses swerve, default pauses
 *   - When that command ends, default resumes
 *
 * BUTTON BINDINGS:
 * ----------------
 *   onTrue(): Run command when button is pressed (rising edge)
 *   whileTrue(): Run command while button is held
 *   onFalse(): Run command when button is released (falling edge)
 *   toggleOnTrue(): Toggle command on/off when pressed
 *
 *   Example:
 *     driverJoystick.intake().whileTrue(intake.intakeCommand())
 *     │                      │         │
 *     The intake button      While     Run the intake command
 *                            held
 *
 * ========================================================================
 */

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auto.AutoRoutines;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.SwerveCommands;
import frc.robot.Superstructure;
import frc.robot.OI.DriverActionSet;
import frc.robot.OI.XboxDriver;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.DipSwitchSelector;

/**
 * ========================================================================
 * ROBOTCONTAINER CLASS - Creates and connects everything
 * ========================================================================
 *
 * This class is instantiated once in Robot.robotInit().
 *
 * [THE CONSTRUCTOR DOES ALL THE WORK]
 * When RobotContainer is created, it:
 *   1. Creates all subsystems
 *   2. Sets default commands
 *   3. Configures button bindings
 *   4. Registers autonomous routines
 *
 * After that, everything runs automatically via the CommandScheduler!
 */
public class RobotContainer {

    // ================================================================
    // DRIVER INPUT
    // ================================================================

    /**
     * Interface to the driver's controller (joystick/gamepad).
     *
     * [WHAT IS DriverActionSet?]
     * It's an interface that defines all the driver inputs we need:
     *   - forward(), strafe(), turn(): Movement commands
     *   - intake(), outtake(): Game piece control
     *   - resetGyro(), toggleFieldRelative(): Utility buttons
     *
     * XboxDriver implements this interface for an Xbox controller.
     * We could create other implementations for different controllers.
     */
    private final DriverActionSet driverJoystick;

    // ================================================================
    // SUBSYSTEMS - The physical mechanisms
    // ================================================================

    /**
     * Swerve drive subsystem - controls the drivetrain.
     *
     * [WHAT IS SWERVE DRIVE?]
     * A drivetrain where each wheel can spin AND rotate independently.
     * This allows the robot to move in any direction while facing any way!
     */
    private final SwerveDrive swerve;

    /**
     * Vision subsystem - processes camera data.
     *
     * [WHAT DOES VISION DO?]
     * Uses cameras (like Limelight or PhotonVision) to:
     *   - Detect AprilTags for field positioning
     *   - Track game pieces
     *   - Align to targets
     */
    private final Vision vision;

    /**
     * Shooter subsystem - launches FUEL into the hub.
     */
    private final Shooter shooter;

    /**
     * Intake subsystem - collects FUEL from the ground.
     */
    private final Intake intake;

    /**
     * Superstructure - coordinates multiple subsystems.
     *
     * [WHY HAVE A SUPERSTRUCTURE?]
     * Sometimes you need to coordinate multiple mechanisms.
     * For example: "Only shoot if intake is retracted and shooter is spun up"
     * The superstructure holds references to everything and can orchestrate.
     */
    private final Superstructure superstructure;

    // ================================================================
    // AUTONOMOUS SELECTION
    // ================================================================

    /**
     * SmartDashboard chooser for selecting autonomous mode.
     *
     * [HOW SENDABLECHOOSER WORKS]
     * It creates a dropdown menu on SmartDashboard/Shuffleboard.
     * You add options with addOption(), and the driver can select one.
     * getSelected() returns whichever option was chosen.
     *
     *   ┌─────────────────────────────┐
     *   │  Auto Chooser:       [▼]   │
     *   │  ─────────────────────     │
     *   │  > 0: Do Nothing           │
     *   │    1: Score & Collect      │
     *   │    2: Quick Climb          │
     *   │    ...                     │
     *   └─────────────────────────────┘
     */
    private final SendableChooser<Command> autoChooser;

    /**
     * DIP switch selector for hardware-based auto selection.
     *
     * [WHY DIP SWITCHES?]
     * SmartDashboard is great, but what if WiFi is flaky?
     * Physical DIP switches on the robot are more reliable.
     * Flip the switches to select auto mode 0-31.
     *
     *   ┌───────────────────────────────────┐
     *   │  DIP SWITCHES (5 bits = 0-31)    │
     *   │                                   │
     *   │  [1][2][4][8][16]                │
     *   │   ▼  ▼  ▲  ▲  ▼   = 4+8 = 12    │
     *   │   on off on on off               │
     *   └───────────────────────────────────┘
     */
    private final DipSwitchSelector dipSwitchSelector;

    /**
     * Whether to use DIP switch (true) or SmartDashboard chooser (false).
     * Set to true for competition, false for development.
     */
    private static final boolean USE_DIP_SWITCH = true;

    // ================================================================
    // DRIVER SETTINGS
    // ================================================================

    /**
     * Exponent for speed curve (1 = linear, 2 = squared, 3 = cubic).
     *
     * [WHY A SPEED CURVE?]
     * Raw joystick values (-1 to 1) can feel twitchy.
     * Squaring the input makes small movements more precise
     * while still allowing full speed at full deflection.
     *
     *   LINEAR (exponent=1):     SQUARED (exponent=2):
     *   Speed                    Speed
     *     │     ╱                  │      ___╱
     *     │   ╱                    │    ╱
     *     │ ╱                      │  ╱
     *     └───────── Stick        └───────── Stick
     *
     * Squared feels better for most drivers - small movements
     * are more precise, but you can still go full speed.
     */
    private int speedExponent = 2;

    // ================================================================
    // CONSTRUCTOR - Builds everything when robot starts
    // ================================================================

    /**
     * Creates the RobotContainer.
     *
     * This is called ONCE when the robot powers on (from Robot.robotInit()).
     * By the time this constructor finishes, the robot is ready to go!
     *
     * [ORDER OF OPERATIONS]
     * 1. Set up driver input
     * 2. Set up auto selection
     * 3. Create all subsystems
     * 4. Set default commands
     * 5. Configure button bindings
     * 6. Register autonomous routines
     */
    public RobotContainer() {
        // ================================================================
        // STEP 1: DRIVER INPUT SETUP
        // ================================================================
        // Silence the annoying "Joystick Not Connected" warning
        // during development when no controller is plugged in
        DriverStation.silenceJoystickConnectionWarning(true);

        // Create the Xbox controller on the specified USB port
        // Constants.DrivingConstants.CONTROLLER_PORT is typically 0
        driverJoystick = new XboxDriver(Constants.DrivingConstants.CONTROLLER_PORT);

        // ================================================================
        // STEP 2: AUTONOMOUS SELECTION SETUP
        // ================================================================
        // Create the chooser for SmartDashboard auto selection
        autoChooser = new SendableChooser<>();
        SmartDashboard.putData("Auto Chooser", autoChooser);  // Send to dashboard

        // Initialize DIP switch selector for hardware-based auto selection
        dipSwitchSelector = new DipSwitchSelector();

        // ================================================================
        // STEP 3: CREATE ALL SUBSYSTEMS
        // ================================================================
        // Order matters here! Some subsystems depend on others.
        // Vision and Swerve are needed by Shooter for targeting.
        swerve = new SwerveDrive();
        vision = new Vision();
        shooter = new Shooter(vision, swerve);  // Shooter needs vision for targeting
        // intake = new Intake();  // DISABLED - Spark Max ID 9 not connected
        intake = null;

        // Superstructure holds references to all subsystems for coordination
        superstructure = new Superstructure(swerve, vision, shooter, null);

        // ================================================================
        // STEP 4: SET DEFAULT COMMANDS
        // ================================================================
        //
        // [WHAT IS A DEFAULT COMMAND?]
        // A command that runs when NO other command is using that subsystem.
        // For the drivetrain, we want teleop driving to be the default.
        //
        // [HOW THIS WORKS]
        // 1. Robot enters teleop mode
        // 2. Nothing is actively using swerve
        // 3. CommandScheduler sees swerve has a default command
        // 4. CommandScheduler runs the default command automatically
        // 5. Driver can now drive!
        //
        // [SUPPLIER FUNCTIONS]
        // The () -> syntax creates a "supplier" - a function that returns a value.
        // We pass suppliers (not values) so the command can read CURRENT joystick
        // values on each loop, not just the values at construction time.
        //
        //   WRONG: swerve.teleopCommand(driverJoystick.forward(), ...)
        //          This would capture the joystick value ONCE (probably 0)
        //
        //   RIGHT: swerve.teleopCommand(() -> driverJoystick.forward(), ...)
        //          This captures HOW to get the value, called every loop

        Command teleopDriveCommand = swerve.teleopCommand(
                () -> applySpeedCurve(driverJoystick.forward()),  // Forward/back input
                () -> applySpeedCurve(driverJoystick.strafe()),   // Left/right input
                () -> applySpeedCurve(driverJoystick.turn()));    // Rotation input
        swerve.setDefaultCommand(teleopDriveCommand);

        // Set initial gyro heading
        // 180° because robot typically starts facing YOUR driver station
        // (so "forward" on joystick pushes robot toward opposite alliance)
        swerve.resetYaw(Rotation2d.fromDegrees(180));

        // Put the teleop command on SmartDashboard for debugging
        SmartDashboard.putData("TeleOp Command", teleopDriveCommand);

        // ================================================================
        // STEP 5 & 6: BUTTON BINDINGS AND AUTO ROUTINES
        // ================================================================
        configureButtonBindings();  // Set up what buttons do
        registerAutoRoutines();     // Register all autonomous modes
    }

    // ================================================================
    // SPEED CURVE - Makes driving feel better
    // ================================================================

    /**
     * Apply a speed curve to joystick input for better control feel.
     *
     * [THE MATH]
     * output = sign(input) * |input|^exponent
     *
     *   exponent = 1: Linear (raw joystick values)
     *   exponent = 2: Squared (more precise at low speeds)
     *   exponent = 3: Cubed (even more precise)
     *
     * [WHY THIS HELPS]
     * Joysticks are most sensitive in the middle of their range.
     * By squaring the input, small movements stay small, but you
     * can still reach full speed at full deflection.
     *
     *   Input:  0.5 (half joystick)
     *   Linear: 0.5 (half speed)
     *   Squared: 0.25 (quarter speed - more precise!)
     *
     *   Input:  1.0 (full joystick)
     *   Linear: 1.0 (full speed)
     *   Squared: 1.0 (still full speed!)
     *
     * @param input The raw joystick value (-1 to 1)
     * @return The curved output value (-1 to 1)
     */
    private double applySpeedCurve(double input) {
        // Special case: zero input returns zero (avoid floating point weirdness)
        if (input == 0.0) {
            return 0.0;
        }

        // Apply the curve while preserving sign (direction)
        // Math.pow only works with positive numbers, so we:
        // 1. Take absolute value for the power calculation
        // 2. Multiply by the original sign to preserve direction
        double magnitude = Math.pow(Math.abs(input), speedExponent);
        double sign = (input > 0) ? 1.0 : -1.0;
        return magnitude * sign;
    }

    // ================================================================
    // BUTTON BINDINGS - What each button does
    // ================================================================

    /**
     * Configure all button bindings for the driver controller.
     *
     * [TRIGGER TYPES]
     *
     *   onTrue(command)    - Run command once when button is PRESSED
     *   │                    (rising edge - button goes from up to down)
     *   │
     *   whileTrue(command) - Run command while button is HELD
     *   │                    Command ends when button is released
     *   │
     *   onFalse(command)   - Run command once when button is RELEASED
     *   │                    (falling edge - button goes from down to up)
     *   │
     *   toggleOnTrue()     - Toggle command on/off each press
     *
     * [VISUAL TIMELINE]
     *
     *   Button State:  ___▄▄▄▄▄▄▄▄▄▄___
     *                     ↑         ↑
     *                  PRESSED   RELEASED
     *                  (onTrue)  (onFalse)
     *                  │         │
     *   whileTrue:     ←─────────→  (runs entire time button is held)
     *
     * [INSTANT vs CONTINUOUS]
     *
     *   InstantCommand: Runs once and immediately finishes
     *   Example: Toggle a flag, reset a sensor
     *
     *   RunCommand: Runs repeatedly until cancelled
     *   Example: Drive while button held, spin a motor
     */
    private void configureButtonBindings() {

        // ----------------------------------------------------------------
        // SWERVE DRIVE CONTROLS
        // ----------------------------------------------------------------

        // Reset gyro to 180° (facing your driver station)
        // Use this at the start of a match when robot is aligned
        driverJoystick.resetGyro().onTrue(swerve.resetGyroCommand());

        // Toggle between field-relative and robot-relative driving
        // Field-relative: "forward" is always toward opponent's side
        // Robot-relative: "forward" is wherever robot is facing
        driverJoystick.toggleFieldRelative().onTrue(new InstantCommand(swerve::toggleFieldRelative));

        // Ski stop - lock wheels in X pattern to resist pushing
        // Runs until driver moves the joystick again
        // .until() adds a condition that ends the command
        driverJoystick.skiStop().onTrue(
                SwerveCommands.skiStopCommand(swerve).until(driverJoystick::isMovementCommanded));

        // Toggle between fast (exponent=1) and precise (exponent=2) driving
        // Precise mode squares the input for finer control at low speeds
        driverJoystick.toggleSpeed().onTrue(
                new InstantCommand(() -> speedExponent = (speedExponent == 1) ? 2 : 1));

        // ----------------------------------------------------------------
        // INTAKE CONTROLS - DISABLED (Spark Max ID 9 not connected)
        // ----------------------------------------------------------------

        // Intake FUEL while trigger is held
        // holdToIntakeCommand(): deploys intake, runs rollers, retracts when released
        // driverJoystick.intake().whileTrue(intake.holdToIntakeCommand());

        // Outtake (eject) FUEL while button is held
        // driverJoystick.outtake().whileTrue(intake.outtakeCommand());
    }

    // ================================================================
    // AUTONOMOUS REGISTRATION - All the auto modes we can run
    // ================================================================

    /**
     * Register all autonomous routines with the auto chooser.
     *
     * [WHAT THIS DOES]
     * Adds all available auto modes to the SmartDashboard dropdown.
     * During a match, the driver (or pit crew) selects one before enabling.
     *
     * [OPTIMIZATION NOTE]
     * These modes were optimized using simulator benchmarking with 1000+
     * simulated matches each. Mode 13 (Depot+Climb) is the optimal strategy.
     *
     * [SCORING REFERENCE]
     * - FUEL in hub: 1 point each
     * - L1 Climb in AUTO: 15 points
     * - Preload: 3 FUEL (3 pts)
     *
     * [STRATEGY CONSIDERATIONS]
     * - Climb modes are reliable and high-scoring (15+ pts)
     * - FUEL-only modes depend on collection success
     * - Strategic modes (Deny, Center Control) affect opponents
     *
     * [HOW TO ADD A NEW AUTO]
     * 1. Create the routine in AutoRoutines.java
     * 2. Add it here with autoChooser.addOption()
     * 3. Add a case in getAutoFromSelection() if using DIP switch
     */
    private void registerAutoRoutines() {
        // ================================================================
        // ORIGINAL MODES (0-9)
        // ================================================================
        autoChooser.setDefaultOption("0: Do Nothing", AutoRoutines.doNothing());
        // TODO: Re-enable when intake is fixed (Spark Max ID 9)
        // autoChooser.addOption("1: Score & Collect (8 pts)", AutoRoutines.scoreAndCollectAuto(swerve, intake, shooter));
        // autoChooser.addOption("2: Quick Climb (18 pts)", AutoRoutines.quickClimbAuto(swerve, climber, intake, shooter));
        // autoChooser.addOption("3: Score Then Climb (18 pts)", AutoRoutines.scoreThenClimbAuto(swerve, intake, shooter, climber));
        // autoChooser.addOption("4: Depot Raid (5 pts)", AutoRoutines.depotRaidAuto(swerve, intake, shooter));
        // autoChooser.addOption("5: Far Neutral (3-4 pts)", AutoRoutines.farNeutralAuto(swerve, intake, shooter));
        // autoChooser.addOption("6: Preload Only (3 pts)", AutoRoutines.preloadOnlyAuto(swerve, intake, shooter));
        // autoChooser.addOption("7: Max Cycles (8 pts)", AutoRoutines.maxCyclesAuto(swerve, intake, shooter));
        // autoChooser.addOption("8: Climb Support (3 pts)", AutoRoutines.climbSupportAuto(swerve, intake, shooter));
        // autoChooser.addOption("9: Win AUTO (4 pts)", AutoRoutines.winAutoAuto(swerve, intake, shooter));

        // ================================================================
        // STRATEGIC MODES (15-19)
        // ================================================================
        // TODO: Re-enable when intake is fixed (Spark Max ID 9)
        // autoChooser.addOption("16: Dual Cycle (6-8 pts)", AutoRoutines.dualCycleAuto(swerve, intake, shooter));
        // autoChooser.addOption("17: Deny FUEL (strategic)", AutoRoutines.denyFuelAuto(swerve, intake, shooter));
        // autoChooser.addOption("18: Center Control (strategic)", AutoRoutines.centerControlAuto(swerve, intake, shooter));
        // autoChooser.addOption("19: Alliance Support (strategic)", AutoRoutines.allianceSupportAuto(swerve, intake, shooter));

        // ================================================================
        // TEST ROUTINES (Not DIP selectable)
        // ================================================================
        autoChooser.addOption("TEST: Drive Forward", AutoRoutines.driveForwardAuto(swerve));
        autoChooser.addOption("TEST: Drive Backward", AutoRoutines.driveBackwardAuto(swerve));
        // TODO: Re-enable when intake is fixed (Spark Max ID 9)
        // autoChooser.addOption("TEST: Drive and Intake", AutoRoutines.driveAndIntakeAuto(swerve, intake));
        // autoChooser.addOption("TEST: Two FUEL Auto", AutoRoutines.twoFuelAuto(swerve, intake, shooter));
    }

    // ================================================================
    // AUTONOMOUS COMMAND GETTER - Called when auto starts
    // ================================================================

    /**
     * Get the autonomous command to run.
     *
     * Called by Robot.autonomousInit() when the autonomous period begins.
     * Returns whichever auto routine was selected (either by DIP switch
     * or SmartDashboard chooser).
     *
     * [DIP SWITCH vs SMARTDASHBOARD]
     *
     *   DIP SWITCH (USE_DIP_SWITCH = true):
     *   - Physical switches on the robot
     *   - More reliable (no WiFi needed)
     *   - Used in competition
     *   - Selection is "locked" at start of auto
     *
     *   SMARTDASHBOARD (USE_DIP_SWITCH = false):
     *   - Dropdown menu on computer
     *   - Easier to change
     *   - Used in development/testing
     *
     * [WHY LOCK THE SELECTION?]
     * Once autonomous starts, we don't want the selection to change
     * mid-routine if someone accidentally bumps the switches.
     * lockSelection() freezes the current reading.
     *
     * @return The selected autonomous command
     */
    public Command getAutonomousCommand() {
        if (USE_DIP_SWITCH) {
            // Lock the selection at the start of auto to prevent mid-match changes
            dipSwitchSelector.lockSelection();
            int selection = dipSwitchSelector.getSelection();
            return AutoRoutines.getAutoFromSelection(selection, swerve, intake, shooter);
        } else {
            return autoChooser.getSelected();
        }
    }

    /**
     * Called when robot is disabled.
     *
     * [WHAT THIS DOES]
     * Unlocks the DIP switch selection so it can be changed for the next match.
     * During a match, the selection was locked to prevent accidental changes.
     */
    public void onDisabled() {
        dipSwitchSelector.unlockSelection();
    }

    // ================================================================
    // TELEMETRY - Data logging for debugging
    // ================================================================

    /**
     * Log telemetry data to SmartDashboard.
     *
     * Called from Robot.robotPeriodic() every 20ms.
     *
     * [WHAT IS TELEMETRY?]
     * Data sent from the robot to the driver station for display.
     * Helps drivers and programmers see what the robot is doing.
     *
     * [SMARTDASHBOARD vs SHUFFLEBOARD]
     * Both display NetworkTables data. Shuffleboard is newer and
     * more customizable, but SmartDashboard is simpler.
     * This code works with both!
     */
    public void logData() {
        // Show current speed mode (true = precise/slow mode)
        SmartDashboard.putBoolean("Slow Speed", speedExponent == 2);

        // Show which auto selection method is active
        SmartDashboard.putBoolean("Auto/Using DIP Switch", USE_DIP_SWITCH);

        // Show current DIP switch reading (so pit crew can verify)
        // This updates every loop, letting drivers confirm selection before match
        dipSwitchSelector.updateDashboard();
    }

}  // End of RobotContainer class