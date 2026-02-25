package frc.robot.commands;

import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.Utils.Lookup;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoFire extends Command 
{
    Intake intake;
    Indexer indexer;
    Turret turret;
    Shooter shooter;
    Hood hood;

    ChassisSpeeds speeds;
    Supplier<Pose2d> poseSupplier;
    Supplier<Rotation2d> rotationSupplier;
    Lookup lookup;

    double turretOffset;
    double tgtRPM;
    double tgtAngle;

    public AutoFire(Intake intake, Indexer indexer, Turret turret, Shooter shooter, Hood hood,
                    ChassisSpeeds speeds, Supplier<Pose2d> poseSupplier, Lookup lookup) {
        this.intake = intake;
        this.indexer = indexer;
        this.turret = turret;
        this.shooter = shooter;
        this.hood = hood;

        addRequirements(intake, indexer, turret, shooter, hood);

        this.speeds = speeds;
        this.poseSupplier = poseSupplier;
        //this.rotationSupplier = rotationSupplier;
        this.lookup = lookup;

        turretOffset = 0.0;
        tgtRPM = 0.0;
        tgtAngle = 0.0;
    }

    
    @Override
    public void initialize() {}

    @Override
    public void execute()
    {
        // Get poses
        Pose2d pose = poseSupplier.get();
        Rotation2d rotation = pose.getRotation();

        // Get values
        double posX = pose.getX() + rotation.getCos() * Constants.HoodConstants.hoodOffset;
        double posY = pose.getY() + rotation.getSin() * Constants.HoodConstants.hoodOffset;
        double velocityX = speeds.vxMetersPerSecond;
        double velocityY = speeds.vyMetersPerSecond;

        // Transform standard x-y velocity such that i^ is towards the shooter, j^ is 90 deg left from top-down
        double transformedVelocityX = velocityX * rotation.getCos() + velocityY * rotation.getSin();
        double transformedVelocityY = -velocityX * rotation.getSin() + velocityY * rotation.getCos();

        // Calculate target vector
        double offsetX = Constants.FieldConstants.HUB_X - posX;
        double offsetY = Constants.FieldConstants.HUB_Y - posY;

        double targetDirectionRad = Math.atan2(offsetY, offsetX);
        double targetDirectionDeg = targetDirectionRad * 180 / Math.PI;
        double targetDistance = Math.sqrt(offsetX * offsetX + offsetY * offsetY);

        double shooterRpm = shooter.getMotorRPM();
        double hoodAngle = hood.getPosition();

        // Get optimal shot
        double[] optimal = lookup.FindOptimalVals(targetDistance, transformedVelocityX, transformedVelocityY, shooterRpm, hoodAngle);
        double error = optimal[0];
        turretOffset = Utils.wrapAngle(targetDirectionDeg - rotation.getDegrees() - optimal[1]);
        tgtRPM = optimal[2];
        tgtAngle = optimal[3];
        
        System.out.println("Pos X: " + posX + ", Pos Y: " + posY + ", Rotation: " + rotation.getDegrees() + ", Vel X: " + transformedVelocityX + ", Vel Y: " + transformedVelocityY + ", TGT RPM: " + tgtRPM + ", TGT Angle: " + tgtAngle);

        // Execute optimal shot
        // turret.target = turretOffset;
        // turret.defaultCommand();
        // shooter.targetVel = tgtRPM;
        // shooter.setVel();
        // hood.moveHoodToAngle(tgtAngle);

        // Only start shooting if ready
        if (error < Constants.ShooterConstants.maxShotError) {
            // intake.intake();
            // indexer.spin();
        } else {
            // intake.stop();
            // indexer.stop();
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        // intake.stop();
        // indexer.stop();
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}