package frc.robot.util;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;

/**
 * Reads a physical DIP switch to select autonomous mode.
 *
 * Uses 5 DIO ports to read a 5-bit binary value (0-31):
 * - Supports up to 32 auto modes (we use 20)
 * - Switch Position 0 (00000): Do Nothing
 * - Switch Position 13 (01101): Depot+Climb (BEST - 20 pts)
 * - See AutoConstants for full list
 *
 * DIP switches are read as active-low (switch ON = LOW/false = 1).
 * This is because DIP switches typically connect the pin to ground when ON.
 */
public class DipSwitchSelector {

    private final DigitalInput bit0;  // LSB (1)
    private final DigitalInput bit1;  // (2)
    private final DigitalInput bit2;  // (4)
    private final DigitalInput bit3;  // (8)
    private final DigitalInput bit4;  // MSB (16)

    private int cachedSelection = -1;
    private boolean selectionLocked = false;

    /**
     * Create a new DIP switch selector using the configured DIO ports.
     */
    public DipSwitchSelector() {
        bit0 = new DigitalInput(AutoConstants.DIP_SWITCH_BIT_0_PORT);
        bit1 = new DigitalInput(AutoConstants.DIP_SWITCH_BIT_1_PORT);
        bit2 = new DigitalInput(AutoConstants.DIP_SWITCH_BIT_2_PORT);
        bit3 = new DigitalInput(AutoConstants.DIP_SWITCH_BIT_3_PORT);
        bit4 = new DigitalInput(AutoConstants.DIP_SWITCH_BIT_4_PORT);
    }

    /**
     * Read the current DIP switch position.
     *
     * DIP switches are typically active-low:
     * - Switch OFF (open) = HIGH (true) = 0
     * - Switch ON (closed to ground) = LOW (false) = 1
     *
     * @return Integer 0-31 representing the switch position (clamped to valid range)
     */
    public int getSelection() {
        // If selection is locked (match started), return cached value
        if (selectionLocked && cachedSelection >= 0) {
            return cachedSelection;
        }

        // Read switches (inverted because active-low)
        int b0 = bit0.get() ? 0 : 1;  // LSB (1)
        int b1 = bit1.get() ? 0 : 1;  // (2)
        int b2 = bit2.get() ? 0 : 1;  // (4)
        int b3 = bit3.get() ? 0 : 1;  // (8)
        int b4 = bit4.get() ? 0 : 1;  // MSB (16)

        // Combine bits: (b4 << 4) | (b3 << 3) | (b2 << 2) | (b1 << 1) | b0
        int selection = (b4 << 4) | (b3 << 3) | (b2 << 2) | (b1 << 1) | b0;

        // Clamp to valid range (0 to NUM_AUTO_MODES-1)
        if (selection >= AutoConstants.NUM_AUTO_MODES) {
            selection = 0;  // Default to Do Nothing if invalid
        }

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
     * @param mode The mode number (0-19)
     * @return Human-readable name
     */
    public static String getModeName(int mode) {
        if (mode >= 0 && mode < AutoConstants.AUTO_MODE_NAMES.length) {
            return AutoConstants.AUTO_MODE_NAMES[mode];
        }
        return "Unknown (" + mode + ")";
    }

    /**
     * Get the binary representation of current switch state.
     *
     * @return 5-character string like "01101" for mode 13
     */
    public String getBinaryString() {
        int selection = getSelection();
        return String.format("%5s", Integer.toBinaryString(selection)).replace(' ', '0');
    }

    /**
     * Update SmartDashboard with current DIP switch status.
     * Call this in robotPeriodic() or disabledPeriodic().
     */
    public void updateDashboard() {
        int selection = getSelection();
        SmartDashboard.putNumber("Auto/DIP Switch Value", selection);
        SmartDashboard.putString("Auto/Selected Mode", getSelectionName());
        SmartDashboard.putString("Auto/DIP Binary", getBinaryString());
        SmartDashboard.putBoolean("Auto/Selection Locked", selectionLocked);

        // Show individual switch states for debugging
        SmartDashboard.putBoolean("Auto/DIP Bit 0 (1)", !bit0.get());
        SmartDashboard.putBoolean("Auto/DIP Bit 1 (2)", !bit1.get());
        SmartDashboard.putBoolean("Auto/DIP Bit 2 (4)", !bit2.get());
        SmartDashboard.putBoolean("Auto/DIP Bit 3 (8)", !bit3.get());
        SmartDashboard.putBoolean("Auto/DIP Bit 4 (16)", !bit4.get());

        // Highlight if optimal mode is selected
        SmartDashboard.putBoolean("Auto/Optimal Mode Selected",
            selection == AutoConstants.AUTO_DEPOT_CLIMB);
    }
}
