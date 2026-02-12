package frc.robot.commands;

import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.TransferConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Limelight;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Utils.Lookup;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoFire extends Command 
{
    Turret turret;
    Transfer transfer;
    Intake intake;
    Shooter shooter;
    Lookup lookup;
    Hood hood;
    double tgtRpm;
    double tgtAngle;

    public AutoFire(Turret turret, Transfer transfer, Shooter shooter, Hood hood, Intake intake, Lookup lookup)
    {
        this.turret = turret;
        this.transfer = transfer;
        this.shooter = shooter;
        this.hood = hood;

        addRequirements(shooter, hood, transfer, turret, intake);

        tgtRpm = 0;
        tgtAngle = 0.0;
    }

    
    @Override
    public void initialize(){
        tgtRpm = 0;
    }

    @Override
    public void execute()
    {
        double distance = Limelight.GetDistance();

        double[] output = lookup.FindOptimalVals(distance);
        
        tgtRpm = output[0];
        tgtAngle = output[1];

        shooter.setTgtRpm((int)(tgtRpm));
        hood.moveHoodToAngle(Angle.ofBaseUnits(tgtAngle, Degrees));
        transfer.activateTransfer();
        intake.intake();
    }

    @Override
    public void end(boolean interrupted)
    {
        // hood.stopHood();
        shooter.targetRPM = 0;
        shooter.fireAtRpm();
        transfer.stop();
        intake.stop();
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
