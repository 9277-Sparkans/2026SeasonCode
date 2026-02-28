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

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoFire extends Command 
{
    Turret turret;
    Shooter shooter;
    Hood hood;

    Supplier<ChassisSpeeds> speedsSupplier;
    Supplier<Pose2d> poseSupplier;
    Supplier<Rotation2d> rotationSupplier;
    Lookup lookup;

    public AutoFire(Turret turret, Shooter shooter, Hood hood,
                    Supplier<ChassisSpeeds> speedsSupplier, Supplier<Pose2d> poseSupplier, Lookup lookup) {
        this.turret = turret;
        this.shooter = shooter;
        this.hood = hood;

        addRequirements(turret, shooter, hood);

        this.speedsSupplier = speedsSupplier;
        this.poseSupplier = poseSupplier;
        this.lookup = lookup;
    }

    
    @Override
    public void initialize() {}

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

        // Calculate target vector
        double offsetX = Constants.FieldConstants.HUB_X - posX;
        double offsetY = Constants.FieldConstants.HUB_Y - posY;

        double targetDirectionRad = Math.atan2(offsetY, offsetX);
        double targetDirectionDeg = targetDirectionRad * 180 / Math.PI;
        double targetDistance = Math.sqrt(offsetX * offsetX + offsetY * offsetY);

        double shooterRPM = shooter.getMotorRPM();
        double hoodAngle = hood.getPosition();

        // Transform standard x-y velocity such that i^ is towards the shooter, j^ is 90 deg left from top-down
        double transformedVelocityX = speeds.vxMetersPerSecond * Math.cos(targetDirectionRad) + speeds.vyMetersPerSecond * Math.sin(targetDirectionRad);
        double transformedVelocityY = -speeds.vxMetersPerSecond * Math.sin(targetDirectionRad) + speeds.vyMetersPerSecond * Math.cos(targetDirectionRad);

        // Get optimal shot
        double[] optimal = lookup.FindOptimalVals(targetDistance, transformedVelocityX, transformedVelocityY, shooterRPM, hoodAngle);
        double optimalError = optimal[0];
        double optimalTurretAngle = Utils.wrapAngle(rotation.getDegrees() - targetDirectionDeg + optimal[1]);
        double optimalShooterRPM = optimal[2];
        double optimalHoodAngle = optimal[3];

        turret.target = optimalTurretAngle;
        shooter.targetVel = optimalShooterRPM;
        hood.targetHoodAngle = optimalHoodAngle;

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
            // indexer.spin();
        } else {
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