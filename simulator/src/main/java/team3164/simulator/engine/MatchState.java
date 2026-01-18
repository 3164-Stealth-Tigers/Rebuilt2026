package team3164.simulator.engine;

import team3164.simulator.Constants;

/**
 * Tracks the overall match state for REBUILT 2026.
 * Includes match phase, timing, HUB activation status, and alliance scores.
 */
public class MatchState {

    // ========================================================================
    // ENUMS
    // ========================================================================

    public enum MatchPhase {
        PRE_MATCH,      // Before match starts
        AUTO,           // 0-20s autonomous period
        TRANSITION,     // 20-30s transition period
        SHIFT_1,        // 30-55s first shift
        SHIFT_2,        // 55-80s second shift
        SHIFT_3,        // 80-105s third shift
        SHIFT_4,        // 105-130s fourth shift
        END_GAME,       // 130-160s end game
        POST_MATCH      // After match ends
    }

    public enum HubStatus {
        ACTIVE,         // HUB is accepting FUEL for points
        INACTIVE        // HUB is not scoring (opponent's turn)
    }

    public enum Alliance {
        RED,
        BLUE
    }

    // ========================================================================
    // TIMING
    // ========================================================================
    public double matchTime = 0;          // Elapsed time in seconds
    public MatchPhase currentPhase = MatchPhase.PRE_MATCH;
    public boolean matchStarted = false;
    public boolean matchEnded = false;

    // ========================================================================
    // HUB STATUS
    // ========================================================================
    public HubStatus redHubStatus = HubStatus.INACTIVE;
    public HubStatus blueHubStatus = HubStatus.INACTIVE;

    // AUTO winner determines first active HUB in TELEOP
    public Alliance autoWinner = null;    // null = tie, determines shift pattern
    public int redAutoScore = 0;
    public int blueAutoScore = 0;

    // ========================================================================
    // ALLIANCE SCORES
    // ========================================================================
    // FUEL scoring
    public int redFuelScored = 0;
    public int blueFuelScored = 0;
    public int redFuelInHub = 0;          // Currently in HUB (not yet fallen out)
    public int blueFuelInHub = 0;

    // TOWER climbing
    public int redTowerPoints = 0;
    public int blueTowerPoints = 0;
    public int[] redRobotClimbLevel = {0, 0, 0};  // Climb level for each robot (0-3)
    public int[] blueRobotClimbLevel = {0, 0, 0};

    // Total scores
    public int redTotalScore = 0;
    public int blueTotalScore = 0;

    // Ranking points
    public boolean redEnergized = false;  // 100 FUEL RP
    public boolean redTraversal = false;  // 50 TOWER points RP
    public boolean blueEnergized = false;
    public boolean blueTraversal = false;

    // ========================================================================
    // OUTPOST / HP STATUS
    // ========================================================================
    public int redChuteCount = Constants.Fuel.CHUTE_START_COUNT;
    public int blueChuteCount = Constants.Fuel.CHUTE_START_COUNT;
    public int redCorralCount = 0;
    public int blueCorralCount = 0;

    public boolean redChuteOpen = false;
    public boolean blueChuteOpen = false;

    // ========================================================================
    // METHODS
    // ========================================================================

    /**
     * Reset match state for a new match.
     */
    public void reset() {
        matchTime = 0;
        currentPhase = MatchPhase.PRE_MATCH;
        matchStarted = false;
        matchEnded = false;

        redHubStatus = HubStatus.INACTIVE;
        blueHubStatus = HubStatus.INACTIVE;

        autoWinner = null;
        redAutoScore = 0;
        blueAutoScore = 0;

        redFuelScored = 0;
        blueFuelScored = 0;
        redFuelInHub = 0;
        blueFuelInHub = 0;

        redTowerPoints = 0;
        blueTowerPoints = 0;
        for (int i = 0; i < 3; i++) {
            redRobotClimbLevel[i] = 0;
            blueRobotClimbLevel[i] = 0;
        }

        redTotalScore = 0;
        blueTotalScore = 0;

        redEnergized = false;
        redTraversal = false;
        blueEnergized = false;
        blueTraversal = false;

        redChuteCount = Constants.Fuel.CHUTE_START_COUNT;
        blueChuteCount = Constants.Fuel.CHUTE_START_COUNT;
        redCorralCount = 0;
        blueCorralCount = 0;

        redChuteOpen = false;
        blueChuteOpen = false;
    }

    /**
     * Start the match.
     */
    public void startMatch() {
        matchStarted = true;
        matchEnded = false;
        matchTime = 0;
        currentPhase = MatchPhase.AUTO;

        // Both HUBs active during AUTO
        redHubStatus = HubStatus.ACTIVE;
        blueHubStatus = HubStatus.ACTIVE;
    }

    /**
     * Update match phase based on elapsed time.
     */
    public void updatePhase() {
        if (!matchStarted || matchEnded) return;

        MatchPhase previousPhase = currentPhase;

        if (matchTime < Constants.Match.AUTO_END) {
            currentPhase = MatchPhase.AUTO;
        } else if (matchTime < Constants.Match.TRANSITION_END) {
            currentPhase = MatchPhase.TRANSITION;
            // During transition, determine auto winner and set HUB status
            if (previousPhase == MatchPhase.AUTO) {
                determineAutoWinner();
            }
        } else if (matchTime < Constants.Match.SHIFT_1_END) {
            currentPhase = MatchPhase.SHIFT_1;
        } else if (matchTime < Constants.Match.SHIFT_2_END) {
            currentPhase = MatchPhase.SHIFT_2;
        } else if (matchTime < Constants.Match.SHIFT_3_END) {
            currentPhase = MatchPhase.SHIFT_3;
        } else if (matchTime < Constants.Match.SHIFT_4_END) {
            currentPhase = MatchPhase.SHIFT_4;
        } else if (matchTime < Constants.Match.TOTAL_DURATION) {
            currentPhase = MatchPhase.END_GAME;
        } else {
            currentPhase = MatchPhase.POST_MATCH;
            matchEnded = true;
        }

        // Update HUB status on phase change
        if (currentPhase != previousPhase) {
            updateHubStatus();
        }
    }

    /**
     * Determine the AUTO winner based on scores.
     */
    private void determineAutoWinner() {
        if (redAutoScore > blueAutoScore) {
            autoWinner = Alliance.RED;
        } else if (blueAutoScore > redAutoScore) {
            autoWinner = Alliance.BLUE;
        } else {
            autoWinner = null;  // Tie - coin flip would happen in real match
        }
    }

    /**
     * Update HUB activation status based on current phase.
     * HUBs alternate being active during TELEOP shifts.
     */
    private void updateHubStatus() {
        switch (currentPhase) {
            case AUTO:
                // Both HUBs active during AUTO
                redHubStatus = HubStatus.ACTIVE;
                blueHubStatus = HubStatus.ACTIVE;
                break;

            case TRANSITION:
                // Both inactive during transition
                redHubStatus = HubStatus.INACTIVE;
                blueHubStatus = HubStatus.INACTIVE;
                break;

            case SHIFT_1:
            case SHIFT_3:
                // Odd shifts: AUTO winner's HUB active
                if (autoWinner == Alliance.RED || autoWinner == null) {
                    redHubStatus = HubStatus.ACTIVE;
                    blueHubStatus = HubStatus.INACTIVE;
                } else {
                    redHubStatus = HubStatus.INACTIVE;
                    blueHubStatus = HubStatus.ACTIVE;
                }
                break;

            case SHIFT_2:
            case SHIFT_4:
                // Even shifts: AUTO loser's HUB active
                if (autoWinner == Alliance.BLUE || autoWinner == null) {
                    redHubStatus = HubStatus.ACTIVE;
                    blueHubStatus = HubStatus.INACTIVE;
                } else {
                    redHubStatus = HubStatus.INACTIVE;
                    blueHubStatus = HubStatus.ACTIVE;
                }
                break;

            case END_GAME:
                // Both HUBs active during end game
                redHubStatus = HubStatus.ACTIVE;
                blueHubStatus = HubStatus.ACTIVE;
                break;

            case POST_MATCH:
            case PRE_MATCH:
                redHubStatus = HubStatus.INACTIVE;
                blueHubStatus = HubStatus.INACTIVE;
                break;
        }
    }

    /**
     * Score FUEL in a HUB.
     *
     * @param alliance Which alliance's HUB
     * @param count Number of FUEL scored
     * @return Points earned (0 if HUB is inactive)
     */
    public int scoreFuel(Alliance alliance, int count) {
        boolean isActive = (alliance == Alliance.RED)
            ? redHubStatus == HubStatus.ACTIVE
            : blueHubStatus == HubStatus.ACTIVE;

        int points = isActive ? count * Constants.Scoring.FUEL_ACTIVE_HUB : 0;

        if (alliance == Alliance.RED) {
            redFuelScored += count;
            if (isActive) {
                redTotalScore += points;
            }
        } else {
            blueFuelScored += count;
            if (isActive) {
                blueTotalScore += points;
            }
        }

        // Track AUTO score separately
        if (currentPhase == MatchPhase.AUTO) {
            if (alliance == Alliance.RED) {
                redAutoScore += points;
            } else {
                blueAutoScore += points;
            }
        }

        // Check ranking points
        updateRankingPoints();

        return points;
    }

    /**
     * Score TOWER climbing points.
     *
     * @param alliance Which alliance
     * @param robotIndex Which robot (0-2)
     * @param level Climb level (1-3)
     * @return Points earned
     */
    public int scoreTowerClimb(Alliance alliance, int robotIndex, int level) {
        if (robotIndex < 0 || robotIndex > 2 || level < 1 || level > 3) return 0;

        int points;
        if (currentPhase == MatchPhase.AUTO) {
            points = Constants.Scoring.TOWER_L1_AUTO;  // Only L1 in AUTO
        } else {
            switch (level) {
                case 1: points = Constants.Scoring.TOWER_L1_TELEOP; break;
                case 2: points = Constants.Scoring.TOWER_L2_TELEOP; break;
                case 3: points = Constants.Scoring.TOWER_L3_TELEOP; break;
                default: points = 0;
            }
        }

        if (alliance == Alliance.RED) {
            redRobotClimbLevel[robotIndex] = level;
            redTowerPoints += points;
            redTotalScore += points;
            if (currentPhase == MatchPhase.AUTO) {
                redAutoScore += points;
            }
        } else {
            blueRobotClimbLevel[robotIndex] = level;
            blueTowerPoints += points;
            blueTotalScore += points;
            if (currentPhase == MatchPhase.AUTO) {
                blueAutoScore += points;
            }
        }

        updateRankingPoints();
        return points;
    }

    /**
     * Update ranking point status.
     */
    private void updateRankingPoints() {
        redEnergized = redFuelScored >= Constants.Scoring.ENERGIZED_THRESHOLD;
        blueEnergized = blueFuelScored >= Constants.Scoring.ENERGIZED_THRESHOLD;

        redTraversal = redTowerPoints >= Constants.Scoring.TRAVERSAL_THRESHOLD;
        blueTraversal = blueTowerPoints >= Constants.Scoring.TRAVERSAL_THRESHOLD;
    }

    /**
     * Check if a HUB is currently active.
     */
    public boolean isHubActive(Alliance alliance) {
        return (alliance == Alliance.RED)
            ? redHubStatus == HubStatus.ACTIVE
            : blueHubStatus == HubStatus.ACTIVE;
    }

    /**
     * Get remaining match time.
     */
    public double getRemainingTime() {
        return Math.max(0, Constants.Match.TOTAL_DURATION - matchTime);
    }

    /**
     * Check if currently in end game.
     */
    public boolean isEndGame() {
        return currentPhase == MatchPhase.END_GAME;
    }

    /**
     * Check if currently in AUTO.
     */
    public boolean isAuto() {
        return currentPhase == MatchPhase.AUTO;
    }

    /**
     * Get formatted match time string (M:SS).
     */
    public String getFormattedTime() {
        double remaining = getRemainingTime();
        int mins = (int) (remaining / 60);
        int secs = (int) (remaining % 60);
        return String.format("%d:%02d", mins, secs);
    }

    /**
     * Get current phase name for display.
     */
    public String getPhaseName() {
        switch (currentPhase) {
            case PRE_MATCH: return "Pre-Match";
            case AUTO: return "AUTO";
            case TRANSITION: return "Transition";
            case SHIFT_1: return "Shift 1";
            case SHIFT_2: return "Shift 2";
            case SHIFT_3: return "Shift 3";
            case SHIFT_4: return "Shift 4";
            case END_GAME: return "END GAME";
            case POST_MATCH: return "Match Over";
            default: return "Unknown";
        }
    }
}
