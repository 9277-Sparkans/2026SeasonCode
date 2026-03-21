package frc.robot.commands;

import frc.robot.subsystems.Turret;
import frc.robot.util.FuelSim;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.Utils.Lookup;
import frc.robot.generated.TunerConstants;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.*;

public class AutoFire extends Command 
{
    public enum TargetHub {
        BLUE_HUB,
        RED_HUB
    }

    private final Indexer indexer;
    private final Turret turret;
    private final Shooter shooter;
    private final Hood hood;

    Supplier<ChassisSpeeds> speedsSupplier;
    Supplier<Pose2d> poseSupplier;
    Lookup lookup;
    TargetHub targetHub;

    private FuelSim fuelSim;
    private final Timer launchCooldown = new Timer();
    private static final double LAUNCH_COOLDOWN_SEC = 0.2;

    private final StructArrayPublisher<Pose3d> trajectoryPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("AutoFire/Trajectory", Pose3d.struct)
            .publish();

    public AutoFire(Indexer indexer, Turret turret, Shooter shooter, Hood hood,
            Supplier<ChassisSpeeds> speedsSupplier, Supplier<Pose2d> poseSupplier, Lookup lookup) {
        this.indexer = indexer;
        this.turret = turret;
        this.shooter = shooter;
        this.hood = hood;

        addRequirements(turret, shooter, hood, indexer);

        this.speedsSupplier = speedsSupplier;
        this.poseSupplier = poseSupplier;
        this.lookup = lookup;
    }

    public void setFuelSim(FuelSim fuelSim) {
        this.fuelSim = fuelSim;
    }

    @Override
    public void initialize() {
        launchCooldown.restart();
    }

    @Override
    public void execute()
    {
        this.targetHub =  DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                        ? TargetHub.RED_HUB
                        : TargetHub.BLUE_HUB;

        // Get poses
        ChassisSpeeds speeds = speedsSupplier.get();
        Pose2d pose = poseSupplier.get();
        Rotation2d rotation = pose.getRotation();

        // Get values
        double posX = pose.getX() + rotation.getCos() * Constants.HoodConstants.hoodOffset;
        double posY = pose.getY() + rotation.getSin() * Constants.HoodConstants.hoodOffset;

        // Get coordinates to target
        double hubX = targetHub == TargetHub.BLUE_HUB  ? Constants.FieldConstants.BLUE_HUB_X : Constants.FieldConstants.RED_HUB_X;
        double hubY = targetHub == TargetHub.BLUE_HUB ? Constants.FieldConstants.BLUE_HUB_Y : Constants.FieldConstants.RED_HUB_Y;
        
        // Calculate target vector
        double offsetX = hubX - posX;
        double offsetY = hubY - posY;

        double targetDirectionRad = Math.atan2(offsetY, offsetX);
        double targetDirectionDeg = targetDirectionRad * 180 / Math.PI;
        double targetDistance = Math.sqrt(offsetX * offsetX + offsetY * offsetY);

        double shooterRPM = shooter.getMotorRPM();
        double hoodAngle = hood.getPosition();
        double turretAngle = turret.getTurretAngle();

        if (RobotBase.isSimulation()) {
            // Check if motors are spun up/aligned enough for a realistic launch 
            // but we still use 'actual' values for the physics calculation.
            if (shooterRPM < 100) {
                shooterRPM = shooter.targetVel; // Fallback for initial frames
            }
        }

        // Transform standard x-y velocity
        double transformedVelocityX = speeds.vxMetersPerSecond * Math.cos(targetDirectionRad) + speeds.vyMetersPerSecond * Math.sin(targetDirectionRad);
        double transformedVelocityY = -speeds.vxMetersPerSecond * Math.sin(targetDirectionRad) + speeds.vyMetersPerSecond * Math.cos(targetDirectionRad);

        // Get optimal shot
        double[] optimal = lookup.FindOptimalVals(targetDistance, transformedVelocityX, transformedVelocityY, shooterRPM, hoodAngle);
        double optimalTurretAngle = Utils.wrapAngle(rotation.getDegrees() - targetDirectionDeg + optimal[1]);
        
        optimalTurretAngle = Utils.clamp(optimalTurretAngle, Constants.TurretConstants.kMinimumAngle, Constants.TurretConstants.kMaximumAngle);

        double stillOffset = Constants.ShooterConstants.rpmOffset * Math.pow(targetDistance, Constants.ShooterConstants.distancePower);
        double speedNormal = Math.sqrt(speeds.vxMetersPerSecond * speeds.vxMetersPerSecond + speeds.vyMetersPerSecond * speeds.vyMetersPerSecond);
        double speedOffset = Math.pow(speedNormal / TunerConstants.kSpeedAt12Volts.magnitude(), Constants.ShooterConstants.speedPower);
        double optimalShooterRPM = optimal[2] - stillOffset * (1.0 - speedOffset);
        
        double optimalHoodAngle = optimal[3];

        turret.target = optimalTurretAngle;
        shooter.targetVel = optimalShooterRPM;
        hood.targetHoodAngle = optimalHoodAngle;

        double optimalError = optimal[0];

        SmartDashboard.putNumber("AutoFire/TargetDistance", targetDistance);
        SmartDashboard.putNumber("AutoFire/optimalError", optimalError);
        SmartDashboard.putNumber("AutoFire/OptimalTurretAngle", optimalTurretAngle);
        SmartDashboard.putNumber("AutoFire/ActualTurretAngle", turretAngle);
        SmartDashboard.putNumber("AutoFire/OptimalShooterRPM", optimalShooterRPM);
        SmartDashboard.putNumber("AutoFire/ActualShooterRPM", shooterRPM);
        SmartDashboard.putNumber("AutoFire/OptimalHoodAngle", optimalHoodAngle);
        SmartDashboard.putNumber("AutoFire/ActualHoodAngle", hoodAngle);

        // Calculate physics-based velocity (matches 2026SeasonCode)
        double flywheelRPM = shooterRPM / Constants.ShooterConstants.kShooterGearRatio;
        double flywheelRadPerSec = flywheelRPM * 2.0 * Math.PI / 60.0;
        double surfaceVelocity = flywheelRadPerSec * Constants.ShooterConstants.kFlywheelRadius;
        double linearVelocity = surfaceVelocity * 0.85; 

        publishTrajectory(pose, speeds, linearVelocity, hoodAngle, turretAngle);

        boolean readyToShoot = optimalError < Constants.ShooterConstants.maxShotError;

        if (RobotBase.isSimulation()) {
            if (fuelSim != null && launchCooldown.hasElapsed(LAUNCH_COOLDOWN_SEC)) {
                // Unlimited fuel in sim: launch balls continuously for testing trajectory
                fuelSim.launchFuel(
                    MetersPerSecond.of(linearVelocity),
                    Degrees.of(45.0 - hoodAngle), // use actual hoodAngle
                    Degrees.of(-turretAngle),    // use actual turretAngle
                    Constants.TurretConstants.ROBOT_TO_TURRET_TRANSFORM
                );
                launchCooldown.restart();
            }
        }

        if (readyToShoot) {
            indexer.setVel();
        } else {
            indexer.stop();
        }
    }

    private void publishTrajectory(Pose2d robotPose, ChassisSpeeds fieldSpeeds, double linearVelocity, double optimalHoodAngle, double optimalTurretAngle) {
        Pose3d launchPose = new Pose3d(robotPose).plus(Constants.TurretConstants.ROBOT_TO_TURRET_TRANSFORM);
        
        double simHoodAngleRad = Math.toRadians(45.0 - optimalHoodAngle);
        double simTurretYawRad = Math.toRadians(-optimalTurretAngle);
        
        double horizontalVel = Math.cos(simHoodAngleRad) * linearVelocity;
        double verticalVel = Math.sin(simHoodAngleRad) * linearVelocity;
        
        double xVel = horizontalVel * Math.cos(simTurretYawRad + launchPose.getRotation().getZ());
        double yVel = horizontalVel * Math.sin(simTurretYawRad + launchPose.getRotation().getZ());
        
        xVel += fieldSpeeds.vxMetersPerSecond;
        yVel += fieldSpeeds.vyMetersPerSecond;
        
        Translation3d pos = launchPose.getTranslation();
        Translation3d vel = new Translation3d(xVel, yVel, verticalVel);
        
        double dt = 0.02; // 50 Hz prediction tick rate
        int maxSteps = (int)(2.0 / dt); 
        
        ArrayList<Pose3d> trajectory = new ArrayList<>();
        
        // Physics constants for drag (matches 2026SeasonCode)
        double FUEL_MASS = 0.448 * 0.45392;
        double FUEL_RADIUS = 0.075;
        double AIR_DENSITY = 1.2041;
        double DRAG_COF = 0.47;
        double FUEL_CROSS_AREA = Math.PI * FUEL_RADIUS * FUEL_RADIUS;
        double DRAG_FORCE_FACTOR = 0.5 * AIR_DENSITY * DRAG_COF * FUEL_CROSS_AREA;
        
        for (int i = 0; i < maxSteps; i++) {
            trajectory.add(new Pose3d(pos, new Rotation3d()));
            pos = pos.plus(vel.times(dt));
            
            if (pos.getZ() > FUEL_RADIUS) {
                Translation3d Fg = new Translation3d(0, 0, -9.81).times(FUEL_MASS);
                Translation3d Fd = new Translation3d();
                
                double speed = vel.getNorm();
                if (speed > 1e-6) {
                    Fd = vel.times(-DRAG_FORCE_FACTOR * speed);
                }
                
                Translation3d accel = Fg.plus(Fd).div(FUEL_MASS);
                vel = vel.plus(accel.times(dt));
            } else {
                break;
            }
        }
        
        trajectoryPublisher.set(trajectory.toArray(new Pose3d[0]));
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