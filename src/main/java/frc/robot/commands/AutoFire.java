package frc.robot.commands;

import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.ShooterSubsystem;

import frc.robot.Constants.TransferConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoFire extends Command 
{
    Turret turret;
    Transfer transfer = new Transfer();
    ShooterSubsystem shooter = new ShooterSubsystem();

    public AutoFire(Turret turret, Transfer transfer, 
    ShooterSubsystem shooter)
    {
        this.turret = turret;
        this.transfer = transfer;
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