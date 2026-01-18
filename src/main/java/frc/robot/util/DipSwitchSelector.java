package frc.robot.util;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;

/**
 * Reads a physical DIP switch to select autonomous mode.
 *
 * Uses 2 DIO ports to read a 2-bit binary value (0-3):
 * - Switch Position 0 (00): Do Nothing
 * - Switch Position 1 (01): Score & Collect
 * - Switch Position 2 (10): Quick Climb
 * - Switch Position 3 (11): Score Then Climb
 *
 * DIP switches are read as active-low (switch ON = LOW/false = 1).
 * This is because DIP switches typically connect the pin to ground when ON.
 */
public class DipSwitchSelector {

    private final DigitalInput bit0;  // LSB
    private final DigitalInput bit1;  // MSB

    private int cachedSelection = -1;
    private boolean selectionLocked = false;

    /**
     * Create a new DIP switch selector using the configured DIO ports.
     */
    public DipSwitchSelector() {
        bit0 = new DigitalInput(AutoConstants.DIP_SWITCH_BIT_0_PORT);
        bit1 = new DigitalInput(AutoConstants.DIP_SWITCH_BIT_1_PORT);
    }

    /**
     * Read the current DIP switch position.
     *
     * DIP switches are typically active-low:
     * - Switch OFF (open) = HIGH (true) = 0
     * - Switch ON (closed to ground) = LOW (false) = 1
     *
     * @return Integer 0-3 representing the switch position
     */
    public int getSelection() {
        // If selection is locked (match started), return cached value
        if (selectionLocked && cachedSelection >= 0) {
            return cachedSelection;
        }

        // Read switches (inverted because active-low)
        int b0 = bit0.get() ? 0 : 1;  // LSB
        int b1 = bit1.get() ? 0 : 1;  // MSB

        // Combine bits: (MSB << 1) | LSB
        int selection = (b1 << 1) | b0;

        return selection;
    }

    /**
     * Lock the current selection (call at start of autonomous).
     * Prevents changes mid-match if someone bumps the switch.
     */
    public void lockSelection() {
        cachedSelection = getSelection();
        selectionLocked = true;
    }

    /**
     * Unlock selection (call when match ends or robot disabled).
     */
    public void unlockSelection() {
        selectionLocked = false;
        cachedSelection = -1;
    }

    /**
     * Check if selection is currently locked.
     */
    public boolean isLocked() {
        return selectionLocked;
    }

    /**
     * Get the name of the currently selected auto mode.
     *
     * @return Human-readable name of the auto mode
     */
    public String getSelectionName() {
        return getModeName(getSelection());
    }

    /**
     * Get the name for a specific mode number.
     *
     * @param mode The mode number (0-3)
     * @return Human-readable name
     */
    public static String getModeName(int mode) {
        switch (mode) {
            case AutoConstants.AUTO_DO_NOTHING:
                return "0: Do Nothing";
            case AutoConstants.AUTO_SCORE_AND_COLLECT:
                return "1: Score & Collect";
            case AutoConstants.AUTO_QUICK_CLIMB:
                return "2: Quick Climb";
            case AutoConstants.AUTO_SCORE_THEN_CLIMB:
                return "3: Score Then Climb";
            default:
                return "Unknown (" + mode + ")";
        }
    }

    /**
     * Update SmartDashboard with current DIP switch status.
     * Call this in robotPeriodic() or disabledPeriodic().
     */
    public void updateDashboard() {
        int selection = getSelection();
        SmartDashboard.putNumber("Auto/DIP Switch Value", selection);
        SmartDashboard.putString("Auto/Selected Mode", getSelectionName());
        SmartDashboard.putBoolean("Auto/Selection Locked", selectionLocked);

        // Show individual switch states for debugging
        SmartDashboard.putBoolean("Auto/DIP Bit 0 (LSB)", !bit0.get());
        SmartDashboard.putBoolean("Auto/DIP Bit 1 (MSB)", !bit1.get());
    }
}
