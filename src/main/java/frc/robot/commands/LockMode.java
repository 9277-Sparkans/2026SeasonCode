package frc.robot.commands;

import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hood;
import frc.robot.Constants.LockModeConstants;
import edu.wpi.first.wpilibj2.command.Command;

public class LockMode extends Command {
    public enum LockState {
        LEFT,
        CENTER,
        RIGHT,
        TRENCHLEFT,
        TRENCHRIGHT,
        NEUTRAL
    }

    Turret turret;
    Shooter shooter;
    Hood hood;
    double tgtRpm;
    double tgtAngleHood;
    double tgtAngleTurret;
    LockState targetLockState;

    public LockMode(Turret turret, Shooter shooter, Hood hood) {
        this.turret = turret;
        this.shooter = shooter;
        this.hood = hood;

        addRequirements(shooter, hood, turret);

        tgtRpm = 0;
        tgtAngleHood = 0.0;
        tgtAngleTurret = 0.0;
        targetLockState = LockState.NEUTRAL;
    }

    public void setLockState(LockState input) {
        targetLockState = input;
    }
    
    @Override
    public void initialize() {
        tgtRpm = 0.0;
        tgtAngleHood = 0.0;
        tgtAngleTurret = 0.0;
    }

    @Override
    public void execute() {
        switch (targetLockState) {
            case LEFT:
                tgtAngleHood = LockModeConstants.kHoodLeft;
                tgtRpm = LockModeConstants.kRPMLeft;
                tgtAngleTurret = LockModeConstants.kTurretLeft;
                break;
            case CENTER:
                tgtAngleHood = LockModeConstants.kHoodCenter;
                tgtRpm = LockModeConstants.kRPMCenter;
                tgtAngleTurret = LockModeConstants.kTurretCenter;
                break;
            case RIGHT:
                tgtAngleHood = LockModeConstants.kHoodRight;
                tgtRpm = LockModeConstants.kRPMRight;
                tgtAngleTurret = LockModeConstants.kTurretRight;
                break;
            case TRENCHLEFT:
                tgtAngleHood = LockModeConstants.kHoodTrenchLeft;
                tgtRpm = LockModeConstants.kRPMTrenchLeft;
                tgtAngleTurret = LockModeConstants.kTurretTrenchLeft;
                break;
            case TRENCHRIGHT:
                tgtAngleHood = LockModeConstants.kHoodTrenchRight;
                tgtRpm = LockModeConstants.kRPMTrenchRight;
                tgtAngleTurret = LockModeConstants.kTurretTrenchRight;
                break;
            case NEUTRAL:
                tgtAngleHood = 0.0;
                tgtRpm = 0.0;
                tgtAngleTurret = 0.0;
                break;
        }

        shooter.targetVel = ((int)(tgtRpm));
        hood.targetHoodAngle = (tgtAngleHood);
        turret.target = (tgtAngleTurret);
        // System.out.println("lcokmodeworkysyipee");

    }

    @Override
    public void end(boolean interrupted) {
        // hood.stopHood();
        // shooter.targetVel = 0;
        // shooter.fireAtRpm();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
