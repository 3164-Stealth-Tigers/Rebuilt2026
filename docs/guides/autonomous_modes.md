# Autonomous Mode Selection Guide

This document describes the autonomous modes available for competition and how to select them using the physical DIP switch on the robot.

## DIP Switch Configuration

The robot uses a **4-bit DIP switch** connected to the roboRIO's Digital Input/Output (DIO) ports to select between 10 autonomous modes (expandable to 16).

### Hardware Setup

| DIP Switch | DIO Port | Function |
|------------|----------|----------|
| Bit 0 (SW1) | DIO 1 | Least Significant Bit (LSB) |
| Bit 1 (SW2) | DIO 2 | |
| Bit 2 (SW3) | DIO 3 | |
| Bit 3 (SW4) | DIO 4 | Most Significant Bit (MSB) |

### Wiring

- Connect each DIP switch between the DIO signal pin and **ground (GND)**
- The roboRIO has internal pull-up resistors, so:
  - Switch **OFF** (open) = HIGH = 0
  - Switch **ON** (closed to ground) = LOW = 1

### Switch Positions

| SW4 | SW3 | SW2 | SW1 | Binary | Selection | Auto Mode |
|:---:|:---:|:---:|:---:|:------:|:---------:|-----------|
| OFF | OFF | OFF | OFF | 0000 | 0 | Do Nothing |
| OFF | OFF | OFF | ON | 0001 | 1 | Score & Collect |
| OFF | OFF | ON | OFF | 0010 | 2 | Quick Climb |
| OFF | OFF | ON | ON | 0011 | 3 | Score Then Climb |
| OFF | ON | OFF | OFF | 0100 | 4 | Depot Raid |
| OFF | ON | OFF | ON | 0101 | 5 | Far Neutral |
| OFF | ON | ON | OFF | 0110 | 6 | Preload Only |
| OFF | ON | ON | ON | 0111 | 7 | Max Cycles |
| ON | OFF | OFF | OFF | 1000 | 8 | Climb Support |
| ON | OFF | OFF | ON | 1001 | 9 | Win AUTO |

---

## Autonomous Modes

### Mode 0: Do Nothing (Safety Default)
**DIP Switch: 0000**

The robot does nothing during autonomous. Use this when:
- Testing other robot systems
- Uncertain about field position
- Alliance partner has a conflicting auto path
- Something is wrong with the robot

**Expected Points:** 0

---

### Mode 1: Score & Collect (Offensive)
**DIP Switch: 0001**

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
**DIP Switch: 0010**

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
**DIP Switch: 0011**

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

### Mode 4: Depot Raid (Protected FUEL Collection)
**DIP Switch: 0100**

**Goal:** Collect FUEL from the safer alliance depot area, then score.

#### Sequence
| Phase | Action | Time |
|-------|--------|------|
| 1 | Drive to alliance depot | ~4 sec |
| 2 | Collect FUEL from depot | ~5 sec |
| 3 | Drive to scoring position | ~4 sec |
| 4 | Score collected FUEL | ~7 sec |

**Expected Points:** 4-8 points from FUEL

**Risk Level:** Low-Medium
- FUEL in depot is less contested
- Longer path means less time for scoring

**Best When:**
- Neutral zone will be congested
- Alliance partners are grabbing neutral FUEL
- You want reliable FUEL collection without defense

---

### Mode 5: Far Neutral (Territory Control)
**DIP Switch: 0101**

**Goal:** Drive to the far side of the neutral zone to collect FUEL and deny opponents.

#### Sequence
| Phase | Action | Time |
|-------|--------|------|
| 1 | Drive to far neutral zone | ~5 sec |
| 2 | Intake FUEL while traversing | ~4 sec |
| 3 | Return toward hub | ~4 sec |
| 4 | Score collected FUEL | ~7 sec |

**Expected Points:** 4-10 points from FUEL

**Risk Level:** Medium-High
- Long travel distance
- Risk of collision with opponents
- Time-critical

**Best When:**
- Alliance partners are handling near-side neutral
- You want to control more of the field
- Opponents are slow or passive

---

### Mode 6: Preload Only (Safe Scoring)
**DIP Switch: 0110**

**Goal:** Shoot only the preloaded FUEL, then hold position.

#### Sequence
| Phase | Action | Time |
|-------|--------|------|
| 1 | Shoot all preloaded FUEL | ~6-8 sec |
| 2 | Hold position | remaining |

**Expected Points:** 3-8 points from FUEL

**Risk Level:** Very Low
- No driving during AUTO
- Predictable and reliable
- No collision risk

**Best When:**
- Drivetrain issues
- Uncertain field obstacles
- Testing shooter in competition
- Alliance partners need you to stay out of the way

---

### Mode 7: Max Cycles (Pure Scoring)
**DIP Switch: 0111**

**Goal:** Continuous shoot-collect-shoot cycles to maximize FUEL scored.

#### Sequence
| Phase | Action | Time |
|-------|--------|------|
| 1 | Score preloaded FUEL | ~4 sec |
| 2 | Drive to neutral zone | ~3 sec |
| 3 | Quick intake | ~4 sec |
| 4 | Score collected FUEL | ~4 sec |
| 5 | Repeat if time allows | - |

**Expected Points:** 10-15+ points from FUEL

**Risk Level:** High
- Time pressure on every phase
- No climbing points
- Depends on shooter AND intake

**Best When:**
- Robot is fast and well-tuned
- Prioritizing FUEL RPs (100+ for ENERGIZED RP)
- Alliance partners are handling climbing
- Going all-in on winning AUTO

---

### Mode 8: Climb Support (Teleop Setup)
**DIP Switch: 1000**

**Goal:** Position near the tower for easy teleop climbing without blocking allies.

#### Sequence
| Phase | Action | Time |
|-------|--------|------|
| 1 | Drive to position near tower | ~6 sec |
| 2 | Hold position | remaining |

**Expected Points:** 0 (during AUTO)

**Risk Level:** Very Low
- No shooting
- Minimal movement
- Strategic positioning

**Best When:**
- Climber is unreliable in AUTO but works in TELEOP
- Alliance partner is already climbing in AUTO
- You want to be ready for immediate TELEOP climb
- Prioritizing TRAVERSAL RP

---

### Mode 9: Win AUTO (Aggressive Scoring)
**DIP Switch: 1001**

**Goal:** Score as much FUEL as possible as fast as possible to win the AUTO period.

#### Sequence
| Phase | Action | Time |
|-------|--------|------|
| 1 | Rapid-fire scoring | ~5 sec |
| 2 | Fast drive while intaking | ~5 sec |
| 3 | Quick intake | ~3 sec |
| 4 | Rapid-fire again | ~5 sec |
| 5 | Continue if time allows | - |

**Expected Points:** 8-16+ points from FUEL

**Risk Level:** Very High
- Maximum speed on all actions
- No time for corrections
- Depends on flawless execution

**Best When:**
- Robot is perfectly tuned
- Driver/auto has been extensively practiced
- Winning AUTO is critical for strategy
- Willing to accept higher failure risk

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

### Mode Selection Matrix

| Situation | Recommended Modes |
|-----------|-------------------|
| Shooter reliable, climber unreliable | Mode 1, 7, or 9 |
| Climber reliable, shooter unreliable | Mode 2 |
| Both systems reliable | Mode 3 |
| Testing in early quals | Mode 0 or 6 |
| Partner already climbing in AUTO | Mode 1, 7, or 9 |
| Contested neutral zone | Mode 4 |
| Positioning for teleop climb | Mode 8 |
| Critical match, need max points | Mode 3 or 9 |

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
| `Auto/DIP Switch Value` | Current switch position (0-9) |
| `Auto/Selected Mode` | Name of selected auto mode |
| `Auto/Selection Locked` | True when auto is running |
| `Auto/DIP Bit 0 (LSB)` | State of switch 1 |
| `Auto/DIP Bit 1` | State of switch 2 |
| `Auto/DIP Bit 2` | State of switch 3 |
| `Auto/DIP Bit 3 (MSB)` | State of switch 4 |
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

- Constants: `Constants.java` -> `AutoConstants`
- DIP Switch Reader: `util/DipSwitchSelector.java`
- Auto Routines: `auto/AutoRoutines.java`
- Selection Logic: `RobotContainer.java` -> `getAutonomousCommand()`
- Simulator Controller: `simulator/engine/AutonomousController.java`
