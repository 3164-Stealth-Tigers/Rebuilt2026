package frc.robot;

/*
 * ========================================================================
 * SUPERSTRUCTURE - Multi-Subsystem Coordinator
 * ========================================================================
 *
 * WHAT THIS FILE DOES:
 * --------------------
 * Coordinates multiple subsystems to perform complex actions that require
 * synchronized movement. Think of it as an "orchestra conductor" - it
 * doesn't play any instruments, but it tells all the musicians when and
 * how to play together.
 *
 * WHY DO WE NEED THIS?
 * --------------------
 * Some robot actions need multiple mechanisms to move in coordination:
 *
 *   Example: Scoring at Level 4 (highest level)
 *   1. Elevator needs to go to max height
 *   2. Arm needs to angle upward
 *   3. Robot needs to be at the correct position
 *   4. All this needs to happen smoothly together!
 *
 * [ANALOGY]
 * Imagine reaching for something on a high shelf:
 *   - Your legs (swerve) position you near the shelf
 *   - Your arm (elevator) extends upward
 *   - Your wrist (coral arm) angles to grab
 *   - Your hand (claw) grabs the object
 *
 * The Superstructure is like your brain coordinating all these movements.
 *
 * KINEMATICS:
 * -----------
 * The "end effector" is the claw/gripper at the end of the arm.
 * Its height depends on BOTH the elevator AND the arm angle:
 *
 *                      ╱ Arm (angled)
 *                     ╱
 *   ┌─────────────────╱──────────── End Effector Height
 *   │ Elevator       ●
 *   │    ↕          Claw
 *   │
 *   └───────────────────────────── Floor
 *
 *   End Effector Height = Carriage Height + (Arm Length × sin(angle))
 *
 * NOT A SUBSYSTEM:
 * ----------------
 * This is intentionally NOT a subsystem (doesn't extend SubsystemBase).
 * It's a utility/helper class that creates commands using other subsystems.
 * The individual subsystems own the hardware; Superstructure just coordinates.
 *
 * HOW TO MODIFY:
 * --------------
 * - Add new scoring positions: Create new command methods
 * - Change coordination logic: Modify setEndEffectorHeightCommand()
 * - Add new subsystems to coordinate: Add to constructor
 *
 * QUICK REFERENCE:
 * ----------------
 * → Check if ready to score: superstructure.readyToScore()
 * → Move to height: superstructure.setEndEffectorHeightCommand(height)
 * → Get subsystems: superstructure.getElevator(), getSwerve(), etc.
 *
 * ========================================================================
 */
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * ========================================================================
 * SUPERSTRUCTURE CLASS - Multi-Subsystem Coordinator
 * ========================================================================
 *
 * NOT a subsystem itself - this is a coordinator that creates commands
 * involving multiple subsystems working together.
 *
 * [WHY NOT A SUBSYSTEM?]
 * Subsystems own hardware. This class doesn't own any hardware; it just
 * creates commands that use multiple subsystems. Making it a subsystem
 * would cause issues with command requirements.
 */
public class Superstructure {

    // ========================================================================
    // SUBSYSTEM REFERENCES
    // ========================================================================

    /**
     * Reference to swerve drive (for checking robot velocity and position).
     */
    private final SwerveDrive swerve;

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================

    /**
     * Creates a new Superstructure coordinator.
     *
     * [NOTE]
     * This just stores references to subsystems. It doesn't create them.
     * The subsystems are created in RobotContainer and passed here.
     *
     * @param swerve The swerve drive subsystem
     * @param autoAlign The auto-align subsystem
     */
    public Superstructure(SwerveDrive swerve) {
        this.swerve = swerve;
    }

    // ================================================
    // ACCESSORS - Get references to subsystems
    // ========================================================================
    // These let other code access the subsystems through Superstructure

    public SwerveDrive getSwerve() {
        return swerve;
    }

}  // End of Superstructure class
