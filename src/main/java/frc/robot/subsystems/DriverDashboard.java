package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AllianceShifts;

public class DriverDashboard extends SubsystemBase {
    private Timer teleopTimer;

    private DoubleSupplier shooterFudgeSupplier;
    private DoubleSupplier turretFudgeSupplier;

    public DriverDashboard(DoubleSupplier shooterFudgeSupplier, DoubleSupplier turretFudgeSupplier) {
        teleopTimer = new Timer();
        this.shooterFudgeSupplier = shooterFudgeSupplier;
        this.turretFudgeSupplier = turretFudgeSupplier;
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

        // SmartDashboard.putString(
        //     "Shifts/Remaining Shift Time",
        //     String.format("%.1f",
        //         Math.max(AllianceShifts.getRemainingShiftTime(teleopTimer), 0.0)));
        SmartDashboard.putNumber(
            "Shifts/Remaining Shift Time",
            AllianceShifts.getRemainingShiftTime(teleopTimer)
        );
        
        SmartDashboard.putBoolean("Shifts/Shift Active", AllianceShifts.areWeActive(teleopTimer));
        
        SmartDashboard.putString(
            "Shifts/Game State", AllianceShifts.getCurrentShift(teleopTimer).toString());

        SmartDashboard.putNumber("Shifts/Turret Fudge Factor", turretFudgeSupplier.getAsDouble());
        SmartDashboard.putNumber("Shifts/Shooter Fudge Factor", shooterFudgeSupplier.getAsDouble());
    }
}
