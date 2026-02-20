package frc.robot.commands;

import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.Constants;
import frc.robot.Limelight;
import frc.robot.Utils.Lookup;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoFire extends Command 
{
    Turret turret;
    Transfer transfer;
    Intake intake;
    Shooter shooter;
    Lookup lookup;
    Hood hood;
    double turretOffset;
    double tgtRPM;
    double tgtAngle;

    public AutoFire(Turret turret, Transfer transfer, Shooter shooter, Hood hood, Intake intake, Lookup lookup)
    {
        this.turret = turret;
        this.transfer = transfer;
        this.shooter = shooter;
        this.hood = hood;

        addRequirements(shooter, hood, transfer, turret, intake);

        turretOffset = 0.0;
        tgtRPM = 0.0;
        tgtAngle = 0.0;
    }

    
    @Override
    public void initialize(){
        tgtRPM = 0;
    }

    @Override
    public void execute()
    {
        double distance = Limelight.GetDistance();
        double velocityX = 0.0; // Velocity to or from the target (+/-)
        double velocityY = 0.0; // Velocity left or right from the target (+/-)
        double shooterRpm = shooter.GetShooterRPM();
        double hoodAngle = hood.GetHoodAngle();

        double[] optimal = lookup.FindOptimalVals(distance, velocityX, velocityY, shooterRpm, hoodAngle);
        double error = optimal[0];
        turretOffset = optimal[1];
        tgtRPM = optimal[2];
        tgtAngle = optimal[3];
        
        turret.setTurretToAngle(turretOffset);
        shooter.setShooterRPM((int)(tgtRPM));
        hood.setHoodToAngle(tgtAngle);

        if (error < Constants.ShooterConstants.maxShotError) {
            transfer.activateTransfer();
            intake.intake();
        } else {
            transfer.stop();
            intake.stop();
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        shooter.fireAtRPM();
        transfer.stop();
        intake.stop();
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
