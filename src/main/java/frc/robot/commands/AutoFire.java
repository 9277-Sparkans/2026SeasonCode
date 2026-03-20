package frc.robot.commands;

import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.Utils.Lookup;
import frc.robot.generated.TunerConstants;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoFire extends Command 
{
    public enum TargetHub {
        BLUE_HUB,
        RED_HUB
    }

    Indexer indexer;
    Turret turret;
    Shooter shooter;
    Hood hood;

    Supplier<ChassisSpeeds> speedsSupplier;
    Supplier<Pose2d> poseSupplier;
    Supplier<Rotation2d> rotationSupplier;
    Lookup lookup;
    TargetHub targetHub;

    public AutoFire(Indexer indexer, Turret turret, Shooter shooter, Hood hood,
            Supplier<ChassisSpeeds> speedsSupplier, Supplier<Pose2d> poseSupplier, Lookup lookup, TargetHub targetHub) {
        this.indexer = indexer;
        this.turret = turret;
        this.shooter = shooter;
        this.hood = hood;

        addRequirements(turret, shooter, hood);

        this.speedsSupplier = speedsSupplier;
        this.poseSupplier = poseSupplier;
        this.lookup = lookup;
        this.targetHub = targetHub;
    }

    
    @Override
    public void initialize() {}

    public Translation2d SelectPosition(Pose2d robotPose)
    {
        Translation2d target;

        // right hand dump
        if (robotPose.getY() < frc.robot.Constants.FieldConstants.BLUE_HUB_Y &&
            robotPose.getX() > frc.robot.Constants.FieldConstants.BLUE_HUB_X)
        {
            target = targetHub == TargetHub.BLUE_HUB ? 
            new Translation2d(frc.robot.Constants.FieldConstants.BLUE_RIGHT_SIDE_X, 
            frc.robot.Constants.FieldConstants.BLUE_RIGHT_SIDE_Y) : 
            new Translation2d(frc.robot.Constants.FieldConstants.RED_RIGHT_SIDE_X, 
            frc.robot.Constants.FieldConstants.RED_RIGHT_SIDE_Y);
        }
        // left hand dump
        else if (robotPose.getY() > frc.robot.Constants.FieldConstants.BLUE_HUB_Y &&
                robotPose.getX() > frc.robot.Constants.FieldConstants.BLUE_HUB_X)
        {
            target = targetHub == TargetHub.BLUE_HUB ? 
            new Translation2d(frc.robot.Constants.FieldConstants.BLUE_LEFT_SIDE_X, 
            frc.robot.Constants.FieldConstants.BLUE_LEFT_SIDE_Y) : 
            new Translation2d(frc.robot.Constants.FieldConstants.RED_LEFT_SIDE_X, 
            frc.robot.Constants.FieldConstants.RED_LEFT_SIDE_Y);
        }
        // goal shots
        else if (robotPose.getX() > frc.robot.Constants.FieldConstants.RED_HUB_X
        && targetHub == TargetHub.RED_HUB)
        {
            target = new Translation2d(frc.robot.Constants.FieldConstants.RED_HUB_X, 
            frc.robot.Constants.FieldConstants.RED_HUB_Y); 
        }
        else 
        {
            target = new Translation2d(frc.robot.Constants.FieldConstants.BLUE_HUB_X, 
            frc.robot.Constants.FieldConstants.BLUE_HUB_Y); 
        }

        return target;
    }

    @Override
    public void execute()
    {
        // Get poses
        ChassisSpeeds speeds = speedsSupplier.get();
        Pose2d pose = poseSupplier.get();
        Rotation2d rotation = pose.getRotation();

        // Get values
        double posX = pose.getX() + rotation.getCos() * Constants.HoodConstants.hoodOffset;
        double posY = pose.getY() + rotation.getSin() * Constants.HoodConstants.hoodOffset;

        // Get coordinates to target
        // double hubX = targetHub == TargetHub.BLUE_HUB  ? Constants.FieldConstants.BLUE_HUB_X : Constants.FieldConstants.RED_HUB_X;
        // double hubY = targetHub == TargetHub.BLUE_HUB ? Constants.FieldConstants.BLUE_HUB_Y : Constants.FieldConstants.RED_HUB_Y;
        
        // idk if this works but it looks like it does
        Translation2d targetPos = SelectPosition(pose);

        double hubX = targetPos.getX();
        double hubY = targetPos.getY();

        // Calculate target vector
        double offsetX = hubX - posX;
        double offsetY = hubY - posY;

        double targetDistance = Math.sqrt(offsetX * offsetX + offsetY * offsetY);

        double shooterRPM = shooter.getMotorRPM();
        double hoodAngle = hood.getPosition();

        // get bot centric vel
        double transformedVelocityX = speeds.vxMetersPerSecond * rotation.getCos() + speeds.vyMetersPerSecond * rotation.getSin();
        double transformedVelocityY = -speeds.vxMetersPerSecond * rotation.getSin() + speeds.vyMetersPerSecond * rotation.getCos();

        // Get optimal static shot
        double[] optimal = lookup.FindOptimalVals(targetDistance, 0, 0, shooterRPM, hoodAngle);
        
        // find virtual goal
        double time = targetDistance / optimal[4];
        System.out.println(time);
        double virtualXOffset = transformedVelocityX * time;
        double virtualYOffset = transformedVelocityY * time;

        offsetX -= virtualXOffset;
        offsetY -= virtualYOffset;

        double targetDirectionRad = Math.atan2(offsetY, offsetX);
        double targetDirectionDeg = targetDirectionRad * 180 / Math.PI;
        targetDistance = Math.sqrt(offsetX * offsetX + offsetY * offsetY);

        double[] virtualOptimal = lookup.FindOptimalVals(targetDistance, 0, 0, shooterRPM, hoodAngle);
        double optimalTurretAngle = Utils.wrapAngle(rotation.getDegrees() - targetDirectionDeg);
        
        double stillOffset = Constants.ShooterConstants.rpmOffset * Math.pow(targetDistance, Constants.ShooterConstants.distancePower);
        double optimalShooterRPM = virtualOptimal[2] - stillOffset;
        double optimalHoodAngle = virtualOptimal[3];

        turret.target = optimalTurretAngle;
        shooter.targetVel = optimalShooterRPM;
        hood.targetHoodAngle = optimalHoodAngle;

        // Calculate Error
        // double shooterRPMRange = (double)(Constants.ShooterConstants.kMaxRPM - Constants.ShooterConstants.kMinOperationalRPM);
        // double hoodAngleRange = Constants.HoodConstants.kMaximumAngle - Constants.HoodConstants.kMinimumAngle;
        // double turretAngleRange = 180.0;

        // double normalizedCurrentShooterRPM = (shooterRPM - Constants.ShooterConstants.kMinRPM) / shooterRPMRange;
        // double normalizedCurrentHoodAngle = (hoodAngle - Constants.HoodConstants.kMinimumAngle) / hoodAngleRange;
        // double normalizedCurrentTurretAngle = turretAngle / turretAngleRange;

        // double normalizedShooterRPM = (optimalShooterRPM - Constants.ShooterConstants.kMinRPM) / shooterRPMRange;
        // double normalizedAngle = (optimalHoodAngle - Constants.HoodConstants.kMinimumAngle) / hoodAngleRange;
        // double normalizedTurretAngle = optimalTurretAngle / turretAngleRange;

        // double weight = (normalizedShooterRPM - normalizedCurrentShooterRPM) * (normalizedShooterRPM - normalizedCurrentShooterRPM)
        //               + (normalizedAngle - normalizedCurrentHoodAngle) * (normalizedAngle - normalizedCurrentHoodAngle)
        //               + (normalizedTurretAngle - normalizedCurrentTurretAngle) * (normalizedTurretAngle - normalizedCurrentTurretAngle);
        // double optimalError = weight / 3.0;

        double optimalError = optimal[0];

        SmartDashboard.putNumber("AutoFire/TargetDistance", targetDistance);
        SmartDashboard.putNumber("AutoFire/TargetDirection", targetDirectionDeg);
        SmartDashboard.putNumber("AutoFire/TransformedVelocityX", transformedVelocityX);
        SmartDashboard.putNumber("AutoFire/TransformedVelocityY", transformedVelocityY);
        SmartDashboard.putNumber("AutoFire/optimalError", optimalError);
        SmartDashboard.putNumber("AutoFire/OptimalTurretAngle", optimalTurretAngle);
        SmartDashboard.putNumber("AutoFire/OptimalShooterRPM", optimalShooterRPM);
        SmartDashboard.putNumber("AutoFire/OptimalHoodAngle", optimalHoodAngle);

        // Only start shooting if ready
        if (optimalError < Constants.ShooterConstants.maxShotError) {
            indexer.setVel();
        } else {
            indexer.stop();
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        indexer.stop();
        shooter.stop();
        turret.stop();
        hood.stopHoodCmd();
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}