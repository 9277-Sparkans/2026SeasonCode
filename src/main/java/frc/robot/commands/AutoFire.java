package frc.robot.commands;

import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.ShooterSubsystem;

import frc.robot.constants.TransferConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.constants.ShooterConstants;

public class AutoFire extends Command 
{
    Turret turret;
    Transfer transfer = new Transfer();
    ShooterSubsystem shooter = new ShooterSubsystem();

    public AutoFire(Turret turret, Transfer transfer, 
    ShooterSubsystem shooter)
    {
        this.turret = turret;
        this.transfer = trnasfer;
        this.shooter = shooter;
    }

    @Override
    public void execute()
    {
        shooter.autoFire();

        // add in hood
        // add in turret

        if (Math.abs(shooter.getCorrectRPM() - shooter.GetShooterVelocity()) < ShooterConstants.rpmLenience)
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

    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}