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
import edu.wpi.first.wpilibj2.command.Command;

public class AutoFire extends Command 
{
    Turret turret;
    Transfer transfer;
    Intake intake;
    Shooter shooter;
    Lookup lookup;
    Hood hood;

    double lastTime = 0.0;
    double lastX = 0.0;
    double lastY = 0.0;

    double turretOffset;
    double tgtRPM;
    double tgtAngle;

    public AutoFire(Turret turret, Transfer transfer, Shooter shooter, Hood hood, Intake intake,
                    Supplier<Pose3d[]> robotPosesSupplier, Supplier<Rotation2d> yawSupplier, Lookup lookup) {
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
        // tgtRpm = 0;
    }

    @Override
    public void execute()
    {
        // Get suppliers
        Pose3d[] robotPoses = robotPosesSupplier.get();
        Rotation2d yaw = yawSupplier.get();

        // Get values
        double time = System.currentTimeMillis();
        double posX = robotPoses[0].getX() + yaw.getCos() * 0.2;
        double posY = robotPoses[0].getY() + yaw.getSin() * 0.2;

        double dt = (lastTime - time) / 1000;
        double velocityX = (posX - lastX) / dt;
        double velocityY = (posY - lastY) / dt;

        // Update values
        lastTime = time;
        lastX = posX;
        lastY = posY;

        // Calculate target vector
        double offsetX = Constants.FieldConstants.HUB_X - posX;
        double offsetY = Constants.FieldConstants.HUB_Y - posY;

        double targetDirectionRad = Math.atan2(offsetY, offsetX);
        double targetDirectionDeg = targetDirectionRad * 180 / Math.PI;
        double targetDistance = Math.sqrt(offsetX * offsetX + offsetY * offsetY)

        // Transform standard x-y velocity such that i^ is towards the shooter, j^ is 90 deg left from top-down
        double transformedVelocityX = velocityX * yaw.getCos() + velocityY * yaw.getSin();
        double transformedVelocityY = -velocityX * yaw.getSin() + velocityY * yaw.getCos();

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
        // hood.stopHood();
        // shooter.targetRPM = 0;
        // shooter.fireAtRpm();
        transfer.stop();
        intake.stop();
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
