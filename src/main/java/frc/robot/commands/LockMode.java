package frc.robot.commands;

import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hood;
import frc.robot.Constants.LockModeConstants;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
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
        NEUTRAL,
        LOCK
    }

    Turret turret;
    Shooter shooter;
    Hood hood;
    LockState lockState;

    public LockMode(Turret turret, Shooter shooter, Hood hood, LockState lockState) {
        this.turret = turret;
        this.shooter = shooter;
        this.hood = hood;

        addRequirements(shooter, hood, turret);

        this.lockState = lockState;
    }
    
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double tgtRpm = 0.0;
        double tgtAngleHood = 0.0;
        double tgtAngleTurret = 0.0;

        switch (lockState) {
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
            case LOCK:
                tgtAngleHood = LockModeConstants.kHoodLock;
                tgtRpm = LockModeConstants.kRPMLock;
                tgtAngleTurret = LockModeConstants.kTurretLock;
                break;
        }

        turret.target = tgtAngleTurret;
        turret.defaultCommand();
        shooter.targetVel = tgtRpm;
        hood.moveHoodToAngle(tgtAngleHood);

        // shooter.targetVel = ((int)(tgtRpm));
        // hood.targetHoodAngle = (tgtAngleHood);
        // turret.target = (tgtAngleTurret);
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

