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
    int tgtRpm;
    double tgtAngle;

    public AutoFire(Turret turret, Transfer transfer, Shooter shooter, Hood hood)
    {
        this.turret = turret;
        this.transfer = transfer;
        this.shooter = shooter;
        this.hood = hood;

        addRequirements(shooter, hood);

        tgtRpm = 0;
        tgtAngle = 0.0;
        
    }

    
    @Override
    public void initialize(){
        tgtRpm = shooter.GetCorrectRps();
        //tgtAngle = hood.GetTargetHoodAngle();
    }

    @Override
    public void execute()
    {
        // get from lookup table
        shooter.fireAtRps();
        // hood.moveHoodToAngle(tgtAngle);

        // add in turret

        if (Math.abs(shooter.GetCorrectRps() - shooter.GetShooterVelocity()) < ShooterConstants.kRpsLenience)
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
        shooter.targetRpm = 0;
        shooter.fireAtRps();
        transfer.stop();
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}