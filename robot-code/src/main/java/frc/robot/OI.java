package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/*
 * ============================================================================
 * OI.JAVA - Operator Interface (Controller Definitions)
 * ============================================================================
 *
 * WHAT THIS FILE DOES:
 * This file defines HOW controllers work. It maps physical buttons to
 * logical actions like "forward", "strafe", "level1", etc.
 *
 * WHY IT'S ORGANIZED THIS WAY:
 * We use INTERFACES to define what a controller should do, then
 * IMPLEMENTATIONS to say which buttons do those things. This lets us
 * easily swap controllers without changing other code!
 *
 * ============================================================================
 * FILE STRUCTURE
 * ============================================================================
 *
 * 1. INTERFACES (What controls are needed)
 *    - DriverActionSet: Controls for driving the robot
 *    - OperatorActionSet: Controls for mechanisms
 *    - ScoringPositionsActionSet: Controls for reef position buttons
 *
 * 2. IMPLEMENTATIONS (Which buttons do what)
 *    - XboxDriver: Xbox controller for driver
 *    - XboxOperator: Xbox controller for operator
 *    - PS4Driver: PS4 controller for driver
 *    - T16000MDriver: Flight stick for driver
 *    - ArcadeScoringPositions: Button board for reef
 *    - PS4ScoringPositions: PS4 for reef
 *    - KeyboardScoringPositions: Xbox as keyboard for reef
 *
 * ============================================================================
 * HOW TO ADD A NEW BUTTON
 * ============================================================================
 *
 * 1. Add the method to the INTERFACE:
 *    Trigger myNewButton();
 *
 * 2. Add the implementation to EACH controller class:
 *    @Override
 *    public Trigger myNewButton() {
 *        return stick.a();  // Or whatever button
 *    }
 *
 * 3. Use it in RobotContainer.java:
 *    operatorJoystick.myNewButton().onTrue(myCommand);
 *
 * ============================================================================
 * CONTROLLER BUTTON MAPS
 * ============================================================================
 *
 * XBOX CONTROLLER:
 * ┌─────────────────────────────────────────┐
 * │    LB          [≡]  [☰]          RB    │
 * │    LT                            RT     │
 * │         [LS]              (Y)           │
 * │                       (X)   (B)         │
 * │              [D-PAD]     (A)            │
 * │                    [RS]                 │
 * └─────────────────────────────────────────┘
 *
 * LS = Left Stick (also button when pressed)
 * RS = Right Stick (also button when pressed)
 * LT/RT = Triggers (analog 0-1)
 * LB/RB = Bumpers (digital)
 * D-PAD = POV (up/down/left/right)
 * [≡] = Back button
 * [☰] = Start button
 *
 * PS4 CONTROLLER:
 * Similar layout but different names:
 * - L1/R1 = Bumpers
 * - L2/R2 = Triggers
 * - Options = Start
 * - Share = Back
 * - Triangle/Square/Cross/Circle = Y/X/A/B
 *
 * ============================================================================
 */
public final class OI {

    // Private constructor prevents instantiation
    // This class is just a container for interfaces and implementations
    private OI() {
    }

  
    // UTILITY METHODS
  

    /**
     * Apply deadband to joystick input.
     *
     * WHAT IS DEADBAND?
     * Joysticks rarely return exactly 0 when released. They might return
     * 0.02 or -0.01 due to small physical imperfections. Deadband treats
     * any value within a small range around 0 as actually being 0.
     *
     * @param value The raw joystick value (-1 to 1)
     * @param band The deadband threshold (typically 0.05 to 0.1)
     * @return 0 if within deadband, otherwise the original value
     *
     * EXAMPLE:
     *   deadband(0.03, 0.1) returns 0.0  (within deadband)
     *   deadband(0.5, 0.1) returns 0.5   (outside deadband)
     */
    public static double deadband(double value, double band) {
        return Math.abs(value) > band ? value : 0;
    }

  
  
    //
    //                        INTERFACE DEFINITIONS
    //
  
  
    // These interfaces define WHAT controls a controller needs.
    // Think of them as a "contract" - any controller that wants to be
    // a driver controller must provide all these methods.
  

    /**
     * DRIVER CONTROLLER INTERFACE
     *
     * The driver controls the robot's movement (drivetrain).
     *
     * REQUIRED CONTROLS:
     * - forward/strafe/turn: Analog axes for driving
     * - resetGyro: Re-zeros the "forward" direction
     * - toggleSpeed: Switch between fast and slow mode
     * - toggleFieldRelative: Switch between field and robot relative
     * - skiStop: Lock wheels to resist being pushed
     */
    public interface DriverActionSet {
        /**
         * Movement along the X axis (forward/backward).
         * @return Value from -1 (full backward) to 1 (full forward)
         */
        double forward();

        /**
         * Movement along the Y axis (left/right strafe).
         * @return Value from -1 (full left) to 1 (full right)
         */
        double strafe();

        /**
         * Rotation around the Z axis (turning).
         * @return Value from -1 (full clockwise) to 1 (full counter-clockwise)
         */
        double turn();

        /** Reset the gyroscope to know which way is "forward" */
        Trigger resetGyro();

        /** Toggle between fast and slow driving modes */
        Trigger toggleSpeed();

        /** Toggle between field-relative and robot-relative control */
        Trigger toggleFieldRelative();

        /** Lock wheels in X pattern to resist pushing (ski stop) */
        Trigger skiStop();

        /**
         * Check if the driver is commanding any movement.
         * Used to release ski stop when driver wants to move.
         * @return true if any movement axis is non-zero
         */
        boolean isMovementCommanded();
    }

  
  
    //
    //                    CONTROLLER IMPLEMENTATIONS
    //
  
  
    // These classes implement the interfaces above for specific controllers.
    // They map physical buttons to the logical actions.
  

  
    // XBOX DRIVER IMPLEMENTATION
  

    /**
     * Xbox controller setup for the DRIVER.
     *
     * BUTTON MAPPING:
     * ┌─────────────────────────────────────────────────────┐
     * │  Control          │  Button/Axis                   │
     * ├─────────────────────────────────────────────────────┤
     * │  Forward/Back     │  Left Stick Y                  │
     * │  Strafe L/R       │  Left Stick X                  │
     * │  Turn             │  Right Stick X (70% speed)     │
     * │  Reset Gyro       │  Start Button                  │
     * │  Toggle Speed     │  X Button                      │
     * │  Field Relative   │  Back Button                   │
     * │  Ski Stop         │  Y Button                      │
     * └─────────────────────────────────────────────────────┘
     */
    public static class XboxDriver implements DriverActionSet {
        private final CommandXboxController stick;

        public XboxDriver(int port) {
            this.stick = new CommandXboxController(port);
        }

        @Override
        public double forward() {
            // Negative because Y axis is inverted on controllers
            return deadband(-stick.getLeftY(), 0.08);
        }

        @Override
        public double strafe() {
            // Negative to match field coordinate system (left = positive Y)
            return deadband(-stick.getLeftX(), 0.08);
        }

        @Override
        public double turn() {
            // 70% speed multiplier for more controlled turning
            return deadband(-stick.getRightX(), 0.08) * 0.7;
        }

        @Override
        public Trigger toggleSpeed() {
            return stick.x();
        }

        @Override
        public Trigger resetGyro() {
            return stick.start();
        }

        @Override
        public Trigger toggleFieldRelative() {
            return stick.back();
        }

        @Override
        public Trigger skiStop() {
            return stick.y();
        }

        @Override
        public boolean isMovementCommanded() {
            return forward() + strafe() + turn() != 0;
        }
    }

  
    // XBOX OPERATOR IMPLEMENTATION
  

    /**
     * Xbox controller setup for the OPERATOR.
     *
     * BUTTON MAPPING:
     * ┌─────────────────────────────────────────────────────┐
     * │  Control          │  Button                        │
     * ├─────────────────────────────────────────────────────┤
     * │  Loading Level    │  Right Trigger                 │
     * │  Level 1          │  A Button                      │
     * │  Level 2          │  X Button                      │
     * │  Level 3          │  B Button                      │
     * │  Level 4          │  Y Button                      │
     * │  Climber Up       │  D-Pad Up                      │
     * │  Climber Down     │  D-Pad Down                    │
     * │  Algae Extend     │  D-Pad Left                    │
     * │  Intake           │  Left Trigger                  │
     * │  Outtake          │  Left Bumper                   │
     * │  Manual Elevator  │  Left Stick Y                  │
     * │  Manual Arm       │  Right Stick Y                 │
     * │  Home Elevator    │  Start Button                  │
     * │  Auto Toggle      │  Back Button                   │
     * └─────────────────────────────────────────────────────┘
     */

  
    // PS4 DRIVER IMPLEMENTATION
  

    /**
     * PS4 controller setup for the DRIVER.
     *
     * Similar to Xbox but with PS4 button names.
     */
    public static class PS4Driver implements DriverActionSet {
        private final CommandPS4Controller stick;

        public PS4Driver(int port) {
            this.stick = new CommandPS4Controller(port);
        }

        @Override
        public double forward() {
            return deadband(-stick.getLeftY(), 0.08);
        }

        @Override
        public double strafe() {
            return deadband(-stick.getLeftX(), 0.08);
        }

        @Override
        public double turn() {
            // 60% speed for PS4 (slightly slower than Xbox)
            return deadband(-stick.getRightX(), 0.08) * 0.6;
        }

        @Override
        public Trigger resetGyro() {
            return stick.options();  // "Start" on PS4
        }

        @Override
        public Trigger toggleSpeed() {
            // Not mapped on PS4
            return new Trigger(() -> false);
        }

        @Override
        public Trigger toggleFieldRelative() {
            return stick.share();  // "Back" on PS4
        }

        @Override
        public Trigger skiStop() {
            return stick.triangle();  // "Y" equivalent
        }

        @Override
        public boolean isMovementCommanded() {
            return forward() + strafe() + turn() != 0;
        }
    }

  
    // T16000M FLIGHT STICK DRIVER IMPLEMENTATION
  

    /**
     * Thrustmaster T.16000M flight stick setup for the DRIVER.
     *
     * Some drivers prefer flight sticks for precise control.
     */
    public static class T16000MDriver implements DriverActionSet {
        private final CommandJoystick stick;

        public T16000MDriver(int port) {
            this.stick = new CommandJoystick(port);
        }

        @Override
        public double forward() {
            // Very small deadband for flight stick precision
            return deadband(-stick.getRawAxis(1), 0.001);
        }

        @Override
        public double strafe() {
            return deadband(-stick.getRawAxis(0), 0.001);
        }

        @Override
        public double turn() {
            // Twist axis for rotation
            return deadband(-stick.getRawAxis(2), 0.01) * 0.6;
        }

        @Override
        public Trigger resetGyro() {
            return stick.button(8);
        }

        @Override
        public Trigger toggleSpeed() {
            return new Trigger(() -> false);
        }

        @Override
        public Trigger toggleFieldRelative() {
            return stick.button(9);
        }

        @Override
        public Trigger skiStop() {
            return stick.trigger();  // Main trigger button
        }

        @Override
        public boolean isMovementCommanded() {
            return forward() + strafe() + turn() != 0;
        }
    }
}
