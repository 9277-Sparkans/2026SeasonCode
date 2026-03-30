package frc.robot.commands;

import frc.robot.subsystems.Turret;
import frc.robot.util.FuelSim;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.util.ShotCalculator;
import frc.robot.util.ShotCalculator.CalculatedShot;
import frc.robot.Constants.ShooterConstants.ShotData;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.*;

/**
 * autofire but using an interpolating tree map and iterative lookahead for
 * moving shots
 */
public class AutoFireInterpolated extends Command {
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
    TargetHub targetHub;

    private FuelSim fuelSim;
    private final Timer launchCooldown = new Timer();
    private static final double LAUNCH_COOLDOWN_SEC = 0.2;

    private boolean isDumping = false;
    private boolean isLeftDump = false;

    private double goodDist = 0.0;

    public boolean finished = false;

    private final StructPublisher<Pose3d> targetPosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("AutoFire/TargetPose", Pose3d.struct)
            .publish();

    private final StructArrayPublisher<Pose3d> trajectoryPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("AutoFire/ActualTrajectory", Pose3d.struct)
            .publish();

    private final StructArrayPublisher<Pose3d> targetTrajectoryPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("AutoFire/TargetTrajectory", Pose3d.struct)
            .publish();

    public AutoFireInterpolated(Indexer indexer, Turret turret, Shooter shooter, Hood hood,
            Supplier<ChassisSpeeds> speedsSupplier, Supplier<Pose2d> poseSupplier) {
        this.indexer = indexer;
        this.turret = turret;
        this.shooter = shooter;
        this.hood = hood;

        // addRequirements(turret, shooter, hood, indexer);

        this.speedsSupplier = speedsSupplier;
        this.poseSupplier = poseSupplier;
    }

    public void setFuelSim(FuelSim fuelSim) {
        this.fuelSim = fuelSim;
    }

    @Override
    public void initialize() {
        launchCooldown.restart();
    }

    @Override
    public void execute() {
        Pose2d pose = poseSupplier.get();
        Rotation2d rotation = pose.getRotation();

        // update alliance
        this.targetHub = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? TargetHub.RED_HUB
                : TargetHub.BLUE_HUB;

        // get coordinates to target hub
        double hubX = targetHub == TargetHub.BLUE_HUB ? Constants.FieldConstants.BLUE_HUB_X
                : Constants.FieldConstants.RED_HUB_X;
        double hubY = targetHub == TargetHub.BLUE_HUB ? Constants.FieldConstants.BLUE_HUB_Y
                : Constants.FieldConstants.RED_HUB_Y;

        // should we dump?
        isDumping = targetHub == TargetHub.BLUE_HUB ? pose.getX() > hubX : pose.getX() < hubX;

        if (isDumping) {
            isLeftDump = pose.getY() > Constants.FieldConstants.BLUE_HUB_Y;

            if (targetHub == TargetHub.BLUE_HUB) {
                hubX = isLeftDump ? Constants.FieldConstants.BLUE_DUMP_LEFT_X
                        : Constants.FieldConstants.BLUE_DUMP_RIGHT_X;
                hubY = isLeftDump ? Constants.FieldConstants.BLUE_DUMP_LEFT_Y
                        : Constants.FieldConstants.BLUE_DUMP_RIGHT_Y;
            } else {
                hubX = isLeftDump ? Constants.FieldConstants.RED_DUMP_LEFT_X
                        : Constants.FieldConstants.RED_DUMP_RIGHT_X;
                hubY = isLeftDump ? Constants.FieldConstants.RED_DUMP_LEFT_Y
                        : Constants.FieldConstants.RED_DUMP_RIGHT_Y;
            }
        }

        // target pose
        double targetZ = isDumping ? 0.0
                : ((targetHub == TargetHub.BLUE_HUB) ? Constants.FieldConstants.HUB_BLUE.getZ()
                        : Constants.FieldConstants.HUB_RED.getZ());
        targetPosePublisher.set(new Pose3d(hubX, hubY, targetZ, new Rotation3d()));

        ChassisSpeeds speeds = speedsSupplier.get();
        double shooterRPM = shooter.getMotorRPM();
        double hoodAngle = hood.getPosition();
        double turretAngle = turret.getTurretAngle();

        if (RobotBase.isSimulation()) {
            if (shooterRPM < 100) {
                shooterRPM = shooter.targetVel;
            }
        }

        // get coordinates to current target
        Translation3d currentTarget = new Translation3d(hubX, hubY, targetZ);

        // calculate iterative shot with moving robot lookahead (5 iterations)
        CalculatedShot calculated = ShotCalculator.calculateIterativeShot(
                pose,
                speeds,
                currentTarget,
                5);

        ShotData optimalShot = calculated.shot();
        Translation3d predictedTarget = calculated.predictedTarget();

        // calculate aim to the predicted target
        Translation2d targetDirVector = predictedTarget.toTranslation2d().minus(pose.getTranslation());
        double targetDirectionRad = Math.atan2(targetDirVector.getY(), targetDirVector.getX());
        double targetDirectionDeg = Math.toDegrees(targetDirectionRad);

        double targetDistance = pose.getTranslation().getDistance(targetDirVector.plus(pose.getTranslation())); // predicted
                                                                                                                // distance

        double optimalTurretAngle = Utils.wrapAngle(rotation.getDegrees() - targetDirectionDeg);
        optimalTurretAngle = Utils.clamp(optimalTurretAngle, Constants.TurretConstants.kMinimumAngle,
                Constants.TurretConstants.kMaximumAngle);

        double optimalShooterRPM = optimalShot.rpm();
        double optimalHoodAngle = optimalShot.hoodAngle();

        turret.target = optimalTurretAngle;
        shooter.targetVel = optimalShooterRPM;
        hood.targetHoodAngle = optimalHoodAngle;

        double optimalError = 0.0;

        SmartDashboard.putNumber("AutoFire/TargetDistance", targetDistance);
        SmartDashboard.putNumber("AutoFire/optimalError", optimalError);
        SmartDashboard.putNumber("AutoFire/OptimalTurretAngle", optimalTurretAngle);
        SmartDashboard.putNumber("AutoFire/ActualTurretAngle", turretAngle);
        SmartDashboard.putNumber("AutoFire/OptimalShooterRPM", optimalShooterRPM);
        SmartDashboard.putNumber("AutoFire/ActualShooterRPM", shooterRPM);
        SmartDashboard.putNumber("AutoFire/OptimalHoodAngle", optimalHoodAngle);
        SmartDashboard.putNumber("AutoFire/ActualHoodAngle", hoodAngle);
        SmartDashboard.putNumber("AutoFire/GoodDist", goodDist);

        // calculate velocity and spin for trajectory
        double v_a = -4.1824e-7;
        double v_b = 4.7509e-3;
        double v_c = -5.1844;
        double linearVelocity = v_a * (shooterRPM * shooterRPM) + v_b * shooterRPM + v_c;

        double s_a = 7.2712e-8;
        double s_b = -6.1209e-3;
        double s_c = -3.6359;
        double spinFactor = s_a * (shooterRPM * shooterRPM) + s_b * shooterRPM + s_c;

        if (linearVelocity < 0)
            linearVelocity = 0;

        // actual trajectory
        publishTrajectory(trajectoryPublisher, pose, speeds, linearVelocity, spinFactor, (10 - hoodAngle), turretAngle);

        // target trajectory
        double targetLinearVelocity = v_a * (optimalShooterRPM * optimalShooterRPM) + v_b * optimalShooterRPM + v_c;
        double targetSpinFactor = s_a * (optimalShooterRPM * optimalShooterRPM) + s_b * optimalShooterRPM + s_c;
        if (targetLinearVelocity < 0)
            targetLinearVelocity = 0;

        publishTrajectory(targetTrajectoryPublisher, pose, speeds, targetLinearVelocity, targetSpinFactor,
                (10 - optimalHoodAngle), optimalTurretAngle);

        if (RobotBase.isSimulation()) {
            if (fuelSim != null && launchCooldown.hasElapsed(LAUNCH_COOLDOWN_SEC)) {
                fuelSim.launchFuel(
                        MetersPerSecond.of(linearVelocity),
                        Degrees.of(45.0 - hoodAngle),
                        Degrees.of(-turretAngle),
                        spinFactor,
                        Constants.TurretConstants.ROBOT_TO_TURRET_TRANSFORM);
                launchCooldown.restart();
            }
        }

        if (Math.abs(optimalShooterRPM - shooterRPM) < 150.0
                && Math.abs(turretAngle - optimalTurretAngle) < 4.0) {
            indexer.setVel();
        } else {
            indexer.stop();
        }
    }

    private void publishTrajectory(StructArrayPublisher<Pose3d> publisher, Pose2d robotPose, ChassisSpeeds fieldSpeeds,
            double linearVelocity, double spin, double hoodAngle, double turretAngle) {
        Pose3d launchPose = new Pose3d(robotPose).plus(Constants.TurretConstants.ROBOT_TO_TURRET_TRANSFORM);

        double simHoodAngleRad = Math.toRadians(45.0 - hoodAngle);
        double simTurretYawRad = Math.toRadians(-turretAngle);

        double horizontalVel = Math.cos(simHoodAngleRad) * linearVelocity;
        double verticalVel = Math.sin(simHoodAngleRad) * linearVelocity;

        double xVel = horizontalVel * Math.cos(simTurretYawRad + launchPose.getRotation().getZ());
        double yVel = horizontalVel * Math.sin(simTurretYawRad + launchPose.getRotation().getZ());

        xVel += fieldSpeeds.vxMetersPerSecond;
        yVel += fieldSpeeds.vyMetersPerSecond;

        Translation3d pos = launchPose.getTranslation();
        Translation3d vel = new Translation3d(xVel, yVel, verticalVel);

        double dt = 0.02;
        int maxSteps = (int) (2.0 / dt);

        ArrayList<Pose3d> trajectory = new ArrayList<>();

        // physics constants for fuelsim
        double FUEL_MASS = 0.448 * 0.45392;
        double FUEL_RADIUS = 0.075;
        double AIR_DENSITY = 1.14;
        double DRAG_COF = 0.5;
        double MAGNUS_K = 0.3;
        double FUEL_CROSS_AREA = Math.PI * FUEL_RADIUS * FUEL_RADIUS;
        double DRAG_FORCE_FACTOR = 0.5 * AIR_DENSITY * DRAG_COF * FUEL_CROSS_AREA;

        for (int i = 0; i < maxSteps; i++) {
            trajectory.add(new Pose3d(pos, new Rotation3d()));
            pos = pos.plus(vel.times(dt));

            if (pos.getZ() > FUEL_RADIUS) {
                Translation3d Fg = new Translation3d(0, 0, -9.81).times(FUEL_MASS);
                Translation3d Fd = new Translation3d();
                Translation3d Fm = new Translation3d();

                double speed = vel.getNorm();
                if (speed > 1e-6) {
                    Fd = vel.times(-DRAG_FORCE_FACTOR * speed);
                    double s_param = (spin * FUEL_RADIUS) / speed;
                    double c_l = MAGNUS_K * s_param;
                    double m_factor = 0.5 * AIR_DENSITY * c_l * FUEL_CROSS_AREA * speed;
                    Fm = new Translation3d(vel.getZ() * m_factor, 0.0, -vel.getX() * m_factor);
                }

                Translation3d accel = Fg.plus(Fd).plus(Fm).div(FUEL_MASS);
                vel = vel.plus(accel.times(dt));
            } else {
                break;
            }
        }

        publisher.set(trajectory.toArray(new Pose3d[0]));
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stop();
        shooter.stop();
        turret.stop();
        hood.stopHoodCmd();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
