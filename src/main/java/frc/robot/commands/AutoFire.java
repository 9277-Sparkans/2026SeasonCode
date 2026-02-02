package frc.robot.commands;

import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hood;

import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoFire extends Command 
{
    Turret turret;
    Transfer transfer;
    Shooter shooter;
    Hood hood;
    int tgtRPM;
    double tgtAngle;

    public AutoFire(Turret turret, Transfer transfer, Shooter shooter, Hood hood)
    {
        this.turret = turret;
        this.transfer = transfer;
        this.shooter = shooter;
        this.hood = hood;

        addRequirements(shooter, hood);

        tgtRPM = 0;
        tgtAngle = 0.0;
        
    }

    
    @Override
    public void initialize(){
        tgtRPM = shooter.GetCorrectRPS();
        //tgtAngle = hood.GetTargetHoodAngle();
    }

    @Override
    public void execute()
    {
        shooter.fireAtRPM();
        // hood.moveHoodToAngle(tgtAngle);

        // add in turret

        if (Math.abs(shooter.GetCorrectRPS() - shooter.GetShooterVelocity()) < ShooterConstants.kRpmLenience)
        {
            transfer.activateTransfer();
        }
        else
        {
            transfer.stop();
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        // hood.stopHood();
        shooter.targetRPM = 0;
        shooter.fireAtRPM();
        transfer.stop();
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}