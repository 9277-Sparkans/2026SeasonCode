package frc.robot.commands;

import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.Constants;
import frc.robot.Limelight;
import frc.robot.Utils;
import frc.robot.Utils.Lookup;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoFire extends Command 
{
    Intake intake;
    Transfer transfer;
    Turret turret;
    Shooter shooter;
    Hood hood;

    ChassisSpeeds speeds;
    Supplier<Pose3d[]> robotPosesSupplier;
    Supplier<Rotation2d> yawSupplier;
    Lookup lookup;

    double turretOffset;
    double tgtRPM;
    double tgtAngle;

    public AutoFire(Intake intake, Transfer transfer, Turret turret, Shooter shooter, Hood hood,
                    ChassisSpeeds speeds, Supplier<Pose3d[]> robotPosesSupplier, Supplier<Rotation2d> yawSupplier, Lookup lookup) {
        this.intake = intake;
        this.transfer = transfer;
        this.turret = turret;
        this.shooter = shooter;
        this.hood = hood;

        addRequirements(transfer, intake, turret, shooter, hood);

        this.speeds = speeds;
        this.robotPosesSupplier = robotPosesSupplier;
        this.yawSupplier = yawSupplier;
        this.lookup = lookup;

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
        // Get suppliers
        Pose3d[] robotPoses = robotPosesSupplier.get();
        Rotation2d yaw = yawSupplier.get();

        // Get values
        double posX = robotPoses[0].getX() + yaw.getCos() * Constants.HoodConstants.hoodOffset;
        double posY = robotPoses[0].getY() + yaw.getSin() * Constants.HoodConstants.hoodOffset;
        double velocityX = speeds.vxMetersPerSecond;
        double velocityY = speeds.vyMetersPerSecond;

        // Transform standard x-y velocity such that i^ is towards the shooter, j^ is 90 deg left from top-down
        double transformedVelocityX = velocityX * yaw.getCos() + velocityY * yaw.getSin();
        double transformedVelocityY = -velocityX * yaw.getSin() + velocityY * yaw.getCos();

        // Calculate target vector
        double offsetX = Constants.FieldConstants.HUB_X - posX;
        double offsetY = Constants.FieldConstants.HUB_Y - posY;

        double targetDirectionRad = Math.atan2(offsetY, offsetX);
        double targetDirectionDeg = targetDirectionRad * 180 / Math.PI;
        double targetDistance = Math.sqrt(offsetX * offsetX + offsetY * offsetY);

        double shooterRpm = shooter.GetShooterRPM();
        double hoodAngle = hood.GetHoodAngle();

        // Get optimal shot
        double[] optimal = lookup.FindOptimalVals(targetDistance, transformedVelocityX, transformedVelocityY, shooterRpm, hoodAngle);
        double error = optimal[0];
        turretOffset = targetDirectionDeg - yaw.getDegrees() - optimal[1];
        tgtRPM = optimal[2];
        tgtAngle = optimal[3];
        
        // Execute optimal shot
        turret.setTurretToAngle(turretOffset);
        shooter.setShooterRPM((int)(tgtRPM));
        hood.setHoodToAngle(tgtAngle);

        // Only start shooting if ready
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
