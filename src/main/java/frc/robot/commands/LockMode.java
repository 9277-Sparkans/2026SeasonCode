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
import frc.robot.Constants.LockModeConstants;
import frc.robot.Utils.Lookup;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;

public class LockMode extends Command 
{
    Turret turret;
    Shooter shooter;
    Hood hood;
    double tgtRpm;
    double tgtAngleHood;
    double tgtAngleTurret;
    lockState targetLockState;

    public LockMode(Turret turret, Shooter shooter, Hood hood)
    {
        this.turret = turret;
        this.shooter = shooter;
        this.hood = hood;

        addRequirements(shooter, hood, turret);

        // tgtRpm = 0;
        tgtAngleHood = 0.0;
        tgtAngleTurret = 0.0;
    }

    public enum lockState
    {
        LEFT,
        CENTER,
        RIGHT
    }

    public void setLockState(lockState input)
    {
        targetLockState = input;
    }
    
    @Override
    public void initialize()
    {
        // tgtRpm = LockModeConstants.kLockModeRPM;
        tgtAngleHood = 0.0;
        tgtAngleTurret = 0.0;
    }

    @Override
    public void execute()
    {
        switch (targetLockState)
        {
            case LEFT:
                tgtAngleHood = LockModeConstants.kHoodLeft;
                tgtAngleTurret = LockModeConstants.kTurretLeft;
                break;
            case CENTER:
                tgtAngleHood = LockModeConstants.kHoodCenter;
                tgtAngleTurret = LockModeConstants.kTurretCenter;
                break;
            case RIGHT:
                tgtAngleHood = LockModeConstants.kHoodRight;
                tgtAngleTurret = LockModeConstants.kTurretRight;
                break;
        }

        // shooter.setTgtRpm((int)(tgtRpm));
        // hood.moveHoodToAngle(Angle.ofBaseUnits(tgtAngleHood, Degrees));
        turret.turretMoveTgt(tgtAngleTurret);
    }

    @Override
    public void end(boolean interrupted)
    {
        // hood.stopHood();
        // shooter.targetRPM = 0;
        // shooter.fireAtRpm();
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
