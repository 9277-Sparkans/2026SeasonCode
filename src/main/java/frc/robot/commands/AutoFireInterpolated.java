package frc.robot.commands;

import frc.robot.subsystems.Turret;
import frc.robot.subsystems.AutoTrack;
import frc.robot.util.FuelSim;
import frc.robot.util.ShotCalculator.CalculatedShot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.TransferConstants;
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
 * autofire but using an interpolating tree map and iterative lookahead for moving shots,
 * except that this doesnt do any calculations and just spins up the shooter and spindexer
 */
public class AutoFireInterpolated extends Command {

    private final Indexer indexer;
    private final Turret turret;
    private final Shooter shooter;
    private final Hood hood;
    private final AutoTrack AutoTrack;
    private final Transfer transfer;
    private final Intake intake;

    private FuelSim fuelSim;
    private final Timer launchCooldown = new Timer();
    private static final double LAUNCH_COOLDOWN_SEC = 0.2;
    private int simFuelCount = 0;

    private boolean isDumping = false;
    private boolean isLeftDump = false;

    private double goodDist = 0.0;

    public boolean finished = false;

    public int shooterRpmFudge = 0;

    private final StructPublisher<Pose3d> targetPosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("AutoFire/TargetPose", Pose3d.struct)
            .publish();

    private final StructArrayPublisher<Pose3d> trajectoryPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("AutoFire/ActualTrajectory", Pose3d.struct)
            .publish();

    private final StructArrayPublisher<Pose3d> targetTrajectoryPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("AutoFire/TargetTrajectory", Pose3d.struct)
            .publish();

    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> speedsSupplier;

    public AutoFireInterpolated(Indexer indexer, Turret turret, Shooter shooter, Hood hood,
            AutoTrack AutoTrack, Transfer transfer, Intake intake,
            Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier) {
        this.indexer = indexer;
        this.turret = turret;
        this.shooter = shooter;
        this.hood = hood;
        this.AutoTrack = AutoTrack;
        this.transfer = transfer;
        this.intake = intake;
        this.poseSupplier = poseSupplier;
        this.speedsSupplier = speedsSupplier;
    }

    public void setFuelSim(FuelSim fuelSim) {
        this.fuelSim = fuelSim;
    }

    @Override
    public void initialize() {
        finished = false;
        launchCooldown.restart();

        CalculatedShot shot = AutoTrack.getLatestShot();

        transfer.activateTransfer();

        if (shot != null) {
            shooter.setTargetRPM(shot.shot().rpm());
        }
    }

    @Override
    public void execute() {
        CalculatedShot calculated = AutoTrack.getLatestShot();
        if (calculated == null)
            return;

        ShotData optimalShot = calculated.shot();

        double optimalShooterRPM = optimalShot.rpm() + shooterRpmFudge;
        shooter.setTargetRPM(optimalShooterRPM);
        hood.targetHoodAngle = optimalShot.hoodAngle();

        double shooterRPM = shooter.getMotorRPM();
        double hoodAngle = hood.getPosition();

        if (RobotBase.isSimulation()) {
            if (shooterRPM < 100) {
                shooterRPM = shooter.targetVel;
            }
        }

        SmartDashboard.putNumber("AutoFire/OptimalShooterRPM", optimalShooterRPM);
        SmartDashboard.putNumber("AutoFire/ActualShooterRPM", shooterRPM);
        SmartDashboard.putNumber("AutoFire/OptimalHoodAngle", optimalShot.hoodAngle());
        SmartDashboard.putNumber("AutoFire/ActualHoodAngle", hoodAngle);
        // SmartDashboard.putNumber("AutoFire/ACTUAL TURRET ANGLE",
        // turret.getPosition());

        // // Calibrated exit velocity polynomial (speed_scale=1.0)
        // double v_a = -4.1825e-7;
        // double v_b = 4.7510e-3;
        // double v_c = -5.1847;
        // double speed_scale = 1.1577; // Calibrated speed scale (Houston)
        // double linearVelocity = (v_a * (shooterRPM * shooterRPM) + v_b * shooterRPM + v_c) * speed_scale;
        // if (linearVelocity < 0)
        //     linearVelocity = 0;

        // // Calibrated spin polynomial (spin_scale=2.0 baked in)
        // double sp_a = 1.4750e-7;
        // double sp_b = -1.2261e-2;
        // double sp_c = -7.2365;
        // double spinVal = sp_a * (shooterRPM * shooterRPM) + sp_b * shooterRPM + sp_c;

        // Pose2d pose = poseSupplier.get();
        // ChassisSpeeds speeds = speedsSupplier.get();

        // targetPosePublisher.set(new Pose3d(AutoTrack.getCurrentTargetX(), AutoTrack.getCurrentTargetY(),
        //         AutoTrack.getCurrentTargetZ(), new Rotation3d()));

        // // Actual Trajectory
        // publishTrajectory(trajectoryPublisher, pose, speeds, linearVelocity, spinVal, hoodAngle,
        //         turret.getTurretAngle());

        // // Target Trajectory
        // double targetLinearVelocity = (v_a * (optimalShooterRPM * optimalShooterRPM) + v_b * optimalShooterRPM + v_c)
        //         * speed_scale;
        // double targetSpinFactor = sp_a * (optimalShooterRPM * optimalShooterRPM) + sp_b * optimalShooterRPM + sp_c;
        // if (targetLinearVelocity < 0)
        //     targetLinearVelocity = 0;

        // publishTrajectory(targetTrajectoryPublisher, pose, speeds, targetLinearVelocity, targetSpinFactor,
        //         optimalShot.hoodAngle(), AutoTrack.getDesiredTurretAngle());

        // // launch fuel in sim
        // if (RobotBase.isSimulation()) {
        //     if (fuelSim != null && launchCooldown.hasElapsed(LAUNCH_COOLDOWN_SEC)
        //             && intake.getFuelCount() > 0) {

        //         if (AutoTrack.isDumping()) {
        //             if (AutoTrack.isTurretOnTarget()) {
        //                 fuelSim.launchFuel(
        //                         MetersPerSecond.of(linearVelocity),
        //                         Degrees.of(56.0 - optimalShot.hoodAngle()),
        //                         Degrees.of(-AutoTrack.getDesiredTurretAngle()),
        //                         spinVal,
        //                         new edu.wpi.first.math.geometry.Transform3d(
        //                                 Constants.TurretConstants.ROBOT_TO_TURRET_TRANSFORM.getTranslation().getX(),
        //                                 Constants.TurretConstants.ROBOT_TO_TURRET_TRANSFORM.getTranslation().getY(),
        //                                 0.39,
        //                                 Constants.TurretConstants.ROBOT_TO_TURRET_TRANSFORM.getRotation()));
        //             }
        //         } else {
        //             fuelSim.launchFuel(
        //                     MetersPerSecond.of(linearVelocity),
        //                     Degrees.of(56.0 - optimalShot.hoodAngle()),
        //                     Degrees.of(-AutoTrack.getDesiredTurretAngle()),
        //                     spinVal,
        //                     new edu.wpi.first.math.geometry.Transform3d(
        //                             Constants.TurretConstants.ROBOT_TO_TURRET_TRANSFORM.getTranslation().getX(),
        //                             Constants.TurretConstants.ROBOT_TO_TURRET_TRANSFORM.getTranslation().getY(),
        //                             0.39,
        //                             Constants.TurretConstants.ROBOT_TO_TURRET_TRANSFORM.getRotation()));
        //         }
        //         intake.removeFuel();
        //         simFuelCount++;
        //         launchCooldown.restart();
        //     }
        // }

        // spindexer gate
        if (AutoTrack.isDumping()) {
            // when dumping, shoot whenever turret is on the target but dont limit by rpm
            if (AutoTrack.isTurretOnTarget()) {
                indexer.activate();
            } else {
                indexer.deactivate();
            }
        } else {
            // shoot whenever shooter and kicker rpm is within range only for hub
            if ((Math.abs(optimalShooterRPM - (shooterRPM + 100.0)) < 250.0)
                    && (transfer.getTransferRps() >= TransferConstants.kTargetTransferRps - 50.0) &&
                    (transfer.getTransferRps() <= TransferConstants.kTargetTransferRps + 20.0)
                    && (Math.abs(turret.getPosition() - AutoTrack.getDesiredTurretAngle()) < 4.0)) {
                indexer.activate();
            } else {
                indexer.deactivate();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexer.deactivate();
        // shooter.stop();
        // shooter.setidlerpm();
        hood.stopHoodCmd();
        transfer.stop();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    // private void publishTrajectory(StructArrayPublisher<Pose3d> publisher, Pose2d robotPose, ChassisSpeeds fieldSpeeds,
    //         double linearVelocity, double spin, double hoodAngle, double turretAngle) {
    //     Pose3d launchPose = new Pose3d(robotPose).plus(Constants.TurretConstants.ROBOT_TO_TURRET_TRANSFORM);

    //     double simHoodAngleRad = Math.toRadians(56.0 - hoodAngle);
    //     double simTurretYawRad = Math.toRadians(-turretAngle);

    //     double horizontalVel = Math.cos(simHoodAngleRad) * linearVelocity;
    //     double verticalVel = Math.sin(simHoodAngleRad) * linearVelocity;

    //     double xVel = horizontalVel * Math.cos(simTurretYawRad + launchPose.getRotation().getZ());
    //     double yVel = horizontalVel * Math.sin(simTurretYawRad + launchPose.getRotation().getZ());

    //     xVel += fieldSpeeds.vxMetersPerSecond;
    //     yVel += fieldSpeeds.vyMetersPerSecond;

    //     Translation3d pos = launchPose.getTranslation();
    //     Translation3d vel = new Translation3d(xVel, yVel, verticalVel);

    //     double dt = 0.01; // Higher accuracy for draggy physics
    //     int maxSteps = (int) (2.0 / dt);

    //     ArrayList<Pose3d> trajectory = new ArrayList<>();

    //     // Physics constants (matches shooter.py and FuelSim.java)
    //     double FUEL_MASS = 0.215;
    //     double FUEL_RADIUS = 0.075;
    //     double AIR_DENSITY = 1.18; // Houston air density
    //     double DRAG_COF = 1.5000; // Calibrated drag
    //     double MAGNUS_K = 1.5000;
    //     double FUEL_CROSS_AREA = Math.PI * FUEL_RADIUS * FUEL_RADIUS;
    //     double DRAG_FORCE_FACTOR = 0.5 * AIR_DENSITY * DRAG_COF * FUEL_CROSS_AREA;

    //     Translation3d targetPos = new Translation3d(AutoTrack.getCurrentTargetX(), AutoTrack.getCurrentTargetY(), AutoTrack.getCurrentTargetZ());
    //     boolean hit = false;
    //     double minDistance = Double.MAX_VALUE;

    //     for (int i = 0; i < maxSteps; i++) {
    //         trajectory.add(new Pose3d(pos, new Rotation3d()));
    //         pos = pos.plus(vel.times(dt));

    //         double distToTarget = pos.getDistance(targetPos);
    //         if (distToTarget < minDistance) {
    //             minDistance = distToTarget;
    //         }

    //         // Check if ball is within hub radius (0.6m) and height window
    //         if (distToTarget < 0.6 && Math.abs(pos.getZ() - targetPos.getZ()) < 0.1) {
    //             hit = true;
    //         }

    //         if (pos.getZ() > FUEL_RADIUS) {
    //             Translation3d Fg = new Translation3d(0, 0, -9.81).times(FUEL_MASS);
    //             Translation3d Fd = new Translation3d();
    //             Translation3d Fm = new Translation3d();

    //             double speed = vel.getNorm();
    //             if (speed > 1e-6) {
    //                 Fd = vel.times(-DRAG_FORCE_FACTOR * speed);

    //                 // Magnus effect (lift) from shooter.py
    //                 double s_param = (spin * FUEL_RADIUS) / speed;
    //                 double c_l = MAGNUS_K * s_param;
    //                 double m_factor = 0.5 * AIR_DENSITY * c_l * FUEL_CROSS_AREA * speed;
    //                 Fm = new Translation3d(vel.getZ() * m_factor, 0.0, -vel.getX() * m_factor);
    //             }

    //             Translation3d accel = Fg.plus(Fd).plus(Fm).div(FUEL_MASS);
    //             vel = vel.plus(accel.times(dt));
    //         } else {
    //             break;
    //         }
    //     }

    //     SmartDashboard.putBoolean("AutoFire/SimHit", hit);
    //     SmartDashboard.putNumber("AutoFire/SimMinMiss_m", minDistance);
    //     publisher.set(trajectory.toArray(new Pose3d[0]));
    // }
}
