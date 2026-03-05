package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

public class AllianceShifts {
    public enum AllianceShift {
        TransitionShift,
        Shift1,
        Shift2,
        Shift3,
        Shift4,
        Endgame
    }

    public static final int SHIFT1_END = 35;
    public static final int SHIFT2_END = 60;
    public static final int SHIFT3_END = 85;
    public static final int SHIFT4_END = 110;

    public static Alliance getAutoWinner() {
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData.length() > 0) {
            switch (gameData.charAt(0)) {
                case 'B':
                    return Alliance.Blue;
                case 'R':
                    return Alliance.Red;
                default:
                    return null;
            }
        } else {
            return null;
        }
    }

    public static boolean didWeWinAuto() {
        return getAutoWinner() == DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    public static double getRemainingShiftTime(Timer teleopTimer) {
        double matchTime = teleopTimer.get();
        return (switch (getCurrentShift(teleopTimer)) {
            case TransitionShift -> 10 - matchTime;
            case Shift1 -> SHIFT1_END - matchTime;
            case Shift2 -> SHIFT2_END - matchTime;
            case Shift3 -> SHIFT3_END - matchTime;
            case Shift4 -> SHIFT4_END - matchTime;
            case Endgame -> 140 - matchTime;
        });
    }

    public static AllianceShift getCurrentShift(Timer teleopTimer) {
        double matchTime = teleopTimer.get();

        if (matchTime < 10) {
            // Transition shift
            return AllianceShift.TransitionShift;
        } else if (matchTime < SHIFT1_END) {
            // Shift 1
            return AllianceShift.Shift1;
        } else if (matchTime < SHIFT2_END) {
            // Shift 2
            return AllianceShift.Shift2;
        } else if (matchTime < SHIFT3_END) {
            // Shift 3
            return AllianceShift.Shift3;
        } else if (matchTime < SHIFT4_END) {
            // Shift 4
            return AllianceShift.Shift4;
        }

        // End game, hub always active
        return AllianceShift.Endgame;
    }

    public static boolean areWeActive(Timer teleopTimer) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }

        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        Alliance autoWinner = getAutoWinner();
        boolean didBlueWinAuto = autoWinner == Alliance.Blue;
        boolean shift1Active = switch (alliance) {
            case Red -> didBlueWinAuto;
            case Blue -> !didBlueWinAuto;
        };

        return (switch (getCurrentShift(teleopTimer)) {
            case TransitionShift -> true;
            case Shift1 -> shift1Active;
            case Shift2 -> !shift1Active;
            case Shift3 -> shift1Active;
            case Shift4 -> !shift1Active;
            case Endgame -> true;
        });
    }
}
