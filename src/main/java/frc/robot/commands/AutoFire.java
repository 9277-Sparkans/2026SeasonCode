package frc.robot.commands;

import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hood;

import frc.robot.Constants.TransferConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.HoodConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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
        tgtRPM = shooter.GetCorrectRPM();
        tgtAngle = hood.GetTargetHoodAngle();
    }

    @Override
    public void execute()
    {
        shooter.fireAtRPM(tgtRPM);
        hood.moveHoodToAngle(tgtAngle);

        // add in turret

        if (Math.abs(shooter.GetCorrectRPM() - shooter.GetShooterVelocity()) < ShooterConstants.kRpmLenience)
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
        hood.stopHood();
        shooter.fireAtRPM(0);
        transfer.stop();
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}