package frc.robot.commands;

import frc.robot.subsystems.Turret;
import frc.robot.subsystems.AutoTrack;
import frc.robot.util.FuelSim;
import frc.robot.util.ShotCalculator.CalculatedShot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
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

    private FuelSim fuelSim;
    private final Timer launchCooldown = new Timer();
    private static final double LAUNCH_COOLDOWN_SEC = 0.2;
    private int simFuelCount = 0;

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
            AutoTrack AutoTrack) {
        this.indexer = indexer;
        this.turret = turret;
        this.shooter = shooter;
        this.hood = hood;
        this.AutoTrack = AutoTrack;
    }

    public void setFuelSim(FuelSim fuelSim) {
        this.fuelSim = fuelSim;
    }

    @Override
    public void initialize() {
        finished = false;
        launchCooldown.restart();

        CalculatedShot shot = AutoTrack.getLatestShot();
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

        double optimalShooterRPM = optimalShot.rpm();
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
        // SmartDashboard.putNumber("AutoFire/ACTUAL TURRET ANGLE", turret.getPosition());

        // launch fuel in sim
        if (RobotBase.isSimulation()) {
            if (fuelSim != null && launchCooldown.hasElapsed(LAUNCH_COOLDOWN_SEC)) {

                // fuel sim logic taken from hammerheads
                double linearVelocity = shooterRPM * (2.0 * Math.PI / 60.0) * 0.0508;

                if (AutoTrack.isDumping()) {
                    // if (targetValid) 
                    {
                        fuelSim.launchFuel(
                                MetersPerSecond.of(linearVelocity),
                                Degrees.of(90.0 - optimalShot.hoodAngle()),
                                Degrees.of(AutoTrack.getDesiredTurretAngle()),
                                Constants.TurretConstants.ROBOT_TO_TURRET_TRANSFORM);
                        simFuelCount++;
                        launchCooldown.restart();
                    }
                } else {
                    fuelSim.launchFuel(
                            MetersPerSecond.of(linearVelocity),
                            Degrees.of(90.0 - optimalShot.hoodAngle()),
                            Degrees.of(AutoTrack.getDesiredTurretAngle()),
                            Constants.TurretConstants.ROBOT_TO_TURRET_TRANSFORM);
                    simFuelCount++;
                    launchCooldown.restart();
                }
            }
        }

        // indexer.activate();

        // shoot whenever shooter rpm is within range
        if (Math.abs(optimalShooterRPM - (shooterRPM + 100.0)) < 300.0) {
            indexer.activate();
        } else {
            indexer.deactivate();
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexer.deactivate();
        shooter.stop();
        hood.stopHoodCmd();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
