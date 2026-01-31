package frc.robot.commands;

import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.TransferConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Limelight;
import frc.robot.Utils.Lookup;
import frc.robot.Constants.HoodConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoFire extends Command 
{
    Turret turret;
    Transfer transfer;
    Intake intake;
    Shooter shooter;
    Hood hood;
    Lookup lookup;
    int tgtRPM;
    double tgtAngle;

    public AutoFire(Turret turret, Transfer transfer, Shooter shooter, Hood hood, Intake intake, Lookup lookup)
    {
        this.turret = turret;
        this.transfer = transfer;
        this.shooter = shooter;
        this.hood = hood;
        this.lookup = lookup;

        addRequirements(shooter, hood);

        tgtRPM = 0;
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

        double[] optimal = lookup.FindOptimalVals(distance);
        tgtRPM = (int)(optimal[0]); // If possible RPM's should be doubles
        tgtAngle = optimal[1];;

        shooter.setShooterRPM(tgtRPM);
        hood.setHoodToAngle(tgtAngle);
        transfer.activateTransfer();
        intake.intake();
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