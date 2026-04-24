package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AllianceShifts;

public class DriverDashboard extends SubsystemBase {
    private Timer teleopTimer;

    public DriverDashboard() {
        teleopTimer = new Timer();
    }

    public void beginTeleop() {
        teleopTimer.start();
    }
    
    @Override
    public void periodic() {
        if (teleopTimer.get() >= 140) teleopTimer.stop();
        
        SmartDashboard.putNumber("Shifts/Match Time", teleopTimer.get());
        SmartDashboard.putString("Shifts/Auto Winner", AllianceShifts.getAutoWinnerAsString());
        SmartDashboard.putBoolean("Shifts/Received FMS Game Data", AllianceShifts.receivedFMSData());

        SmartDashboard.putString(
            "Shifts/Remaining Shift Time",
            String.format("%.1f",
                Math.max(AllianceShifts.getRemainingShiftTime(teleopTimer), 0.0)));
        
        SmartDashboard.putBoolean("Shifts/Shift Active", AllianceShifts.areWeActive(teleopTimer));
        
        SmartDashboard.putString(
            "Shifts/Game State", AllianceShifts.getCurrentShift(teleopTimer).toString());
    }
}
