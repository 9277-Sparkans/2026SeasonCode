package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

public class AllianceShifts {
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

        double matchTime = teleopTimer.get();

        if (matchTime < 10) {
            // Transition shift
            return true;
        } else if (matchTime < 35) {
            // Shift 1
            return shift1Active;
        } else if (matchTime < 60) {
            // Shift 2
            return !shift1Active;
        } else if (matchTime > 85) {
            // Shift 3
            return shift1Active;
        } else if (matchTime > 110) {
            // Shift 4
            return !shift1Active;
        }

        // End game, hub always active
        return true;
    }
}
