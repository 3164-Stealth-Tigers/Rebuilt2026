# Autonomous Mode Selection Guide

This document describes the autonomous modes available for competition and how to select them using the physical DIP switch on the robot.

## DIP Switch Configuration

The robot uses a **2-bit DIP switch** connected to the roboRIO's Digital Input/Output (DIO) ports to select between 4 autonomous modes.

### Hardware Setup

| DIP Switch | DIO Port | Function |
|------------|----------|----------|
| Bit 0 (SW1) | DIO 1 | Least Significant Bit (LSB) |
| Bit 1 (SW2) | DIO 2 | Most Significant Bit (MSB) |

### Wiring

- Connect each DIP switch between the DIO signal pin and **ground (GND)**
- The roboRIO has internal pull-up resistors, so:
  - Switch **OFF** (open) = HIGH = 0
  - Switch **ON** (closed to ground) = LOW = 1

### Switch Positions

| Bit 1 (SW2) | Bit 0 (SW1) | Binary | Selection | Auto Mode |
|:-----------:|:-----------:|:------:|:---------:|-----------|
| OFF | OFF | 00 | 0 | Do Nothing |
| OFF | ON | 01 | 1 | Score & Collect |
| ON | OFF | 10 | 2 | Quick Climb |
| ON | ON | 11 | 3 | Score Then Climb |

---

## Autonomous Modes

### Mode 0: Do Nothing (Safety Default)
**DIP Switch: OFF-OFF (00)**

The robot does nothing during autonomous. Use this when:
- Testing other robot systems
- Uncertain about field position
- Alliance partner has a conflicting auto path
- Something is wrong with the robot

**Expected Points:** 0

---

### Mode 1: Score & Collect (Offensive)
**DIP Switch: OFF-ON (01)**

**Goal:** Maximize FUEL scored to win the AUTO phase and control hub shift timing.

#### Sequence
| Phase | Action | Time |
|-------|--------|------|
| 1 | Score all 8 preloaded FUEL | ~8 sec |
| 2 | Drive to neutral zone | ~4 sec |
| 3 | Intake additional FUEL | ~5 sec |
| 4 | Score collected FUEL (if time) | ~3 sec |

**Expected Points:** 8-12+ points from FUEL

**Risk Level:** Medium
- Depends on shooter accuracy
- May not complete all phases in 20 seconds

**Best When:**
- Shooter is reliable and tuned
- You want to win AUTO to control hub shift timing
- Alliance partners are handling climbing

---

### Mode 2: Quick Climb (Defensive/Guaranteed)
**DIP Switch: ON-OFF (10)**

**Goal:** Secure guaranteed 15 points by climbing to LEVEL 1.

#### Sequence
| Phase | Action | Time |
|-------|--------|------|
| 1 | Drive directly to TOWER | ~5 sec |
| 2 | Climb to LEVEL 1 | ~10 sec |
| 3 | Hold position | remaining |

**Expected Points:** 15 points (guaranteed)

**Risk Level:** Low
- No shooting required
- Simple, reliable path

**Best When:**
- Shooter is unreliable
- Climber has been tested and works well
- Alliance partner is scoring FUEL
- You need guaranteed points

**Important Notes:**
- Only **2 robots per alliance** can earn L1 points during AUTO
- Coordinate with alliance partners before the match!
- L1 climb is only worth points during AUTO (15 pts vs 10 pts in TELEOP)

---

### Mode 3: Score Then Climb (Maximum Points)
**DIP Switch: ON-ON (11)**

**Goal:** Maximize total AUTO points by scoring FUEL AND climbing.

#### Sequence
| Phase | Action | Time |
|-------|--------|------|
| 1 | Rapid-fire preloaded FUEL | ~6 sec |
| 2 | Drive to TOWER | ~4 sec |
| 3 | Climb to LEVEL 1 | ~10 sec |

**Expected Points:** 19-23 points (4-8 FUEL + 15 climb)

**Risk Level:** High
- Time-critical (20 seconds is tight)
- Both shooter AND climber must work
- Less FUEL scored than Mode 1

**Best When:**
- Robot is well-tuned and practiced
- Both shooter and climber are reliable
- Going for maximum single-robot AUTO contribution
- High-stakes match where every point matters

---

## Strategic Considerations

### Winning AUTO
The alliance that scores **more FUEL** during AUTO gets a strategic advantage:
- Their hub goes **inactive first** during SHIFT 1
- This affects the scoring rhythm for the entire teleop period

If both alliances score the same number of FUEL, the FMS randomly selects which alliance goes first.

### Point Values
| Action | AUTO Points | TELEOP Points |
|--------|-------------|---------------|
| FUEL scored | 1 | 1 |
| LEVEL 1 climb | **15** | 10 |
| LEVEL 2 climb | - | 20 |
| LEVEL 3 climb | - | 30 |

Note: L1 climb is worth **more** during AUTO (15 pts) than TELEOP (10 pts)!

### Ranking Points
For reference, here are the RP thresholds:
- **ENERGIZED RP:** 100+ FUEL scored
- **SUPERCHARGED RP:** 360+ FUEL scored
- **TRAVERSAL RP:** 50+ TOWER points

---

## Pre-Match Checklist

1. **Verify DIP switch position** matches intended auto mode
2. **Check SmartDashboard** - the selected mode is displayed under "Auto/Selected Mode"
3. **Confirm with alliance partners** - especially for Mode 2 (only 2 robots can climb in AUTO)
4. **Verify robot starting position** - ensure clear path to intended locations
5. **Preload FUEL** - up to 8 FUEL can be preloaded before match

---

## SmartDashboard Indicators

The following values are displayed on SmartDashboard:

| Key | Description |
|-----|-------------|
| `Auto/DIP Switch Value` | Current switch position (0-3) |
| `Auto/Selected Mode` | Name of selected auto mode |
| `Auto/Selection Locked` | True when auto is running |
| `Auto/DIP Bit 0 (LSB)` | State of switch 1 |
| `Auto/DIP Bit 1 (MSB)` | State of switch 2 |
| `Auto/Using DIP Switch` | True if DIP switch is enabled |

---

## Troubleshooting

### Auto mode not changing when I flip switches
- Verify wiring is correct (signal to switch, switch to ground)
- Check SmartDashboard for `Auto/DIP Bit X` values
- Ensure `USE_DIP_SWITCH` is `true` in `RobotContainer.java`

### Wrong auto mode running
- The selection is **locked** when autonomous starts
- Verify switch position **before** the match starts
- Check that `Auto/Selection Locked` is false before setting switches

### Want to use SmartDashboard chooser instead
- Set `USE_DIP_SWITCH = false` in `RobotContainer.java`
- Rebuild and redeploy code
- Use the "Auto Chooser" dropdown in SmartDashboard

---

## Code References

- Constants: `Constants.java` → `AutoConstants`
- DIP Switch Reader: `util/DipSwitchSelector.java`
- Auto Routines: `auto/AutoRoutines.java`
- Selection Logic: `RobotContainer.java` → `getAutonomousCommand()`
