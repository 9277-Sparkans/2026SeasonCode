package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants.ShotData;
import frc.robot.Utils;
import frc.robot.util.ShotCalculator;
import frc.robot.util.ShotCalculator.CalculatedShot;

/**
 * runs shot calculations in periodic so everything is already calculated, 
 * so when the driver pulls the trigger only the indexer and shooter spin
 */
public class AutoTrack extends SubsystemBase {
    private final Turret turret;
    private final Hood hood;
    private final Shooter shooter;

    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> speedsSupplier;

    private CalculatedShot latestShot = null;
    private double latestTurretAngle = 0.0;
    private double desiredTurretAngle = 0.0;
    private boolean isDumping = false;
    private boolean isLeftDump = false;
    public boolean tracking = true;
    public double targetDistance = 0.0;

    // target coords
    private double currentHubX = Constants.FieldConstants.BLUE_HUB_X;
    private double currentHubY = Constants.FieldConstants.BLUE_HUB_Y;
    private double currentTargetZ = Constants.FieldConstants.HUB_BLUE.getZ();

    // offset (degrees)
    public int turretFudge = 0;

    public AutoTrack(
            Turret turret,
            Hood hood,
            Shooter shooter,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> speedsSupplier) {
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;
        this.poseSupplier = poseSupplier;
        this.speedsSupplier = speedsSupplier;
    }

    @Override
    public void periodic() {
        if (!tracking)
            return;

        Pose2d pose = poseSupplier.get();
        ChassisSpeeds robotSpeeds = speedsSupplier.get();
        Rotation2d rotation = pose.getRotation();

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        currentHubX = alliance == Alliance.Blue
                ? Constants.FieldConstants.BLUE_HUB_X
                : Constants.FieldConstants.RED_HUB_X;
        currentHubY = alliance == Alliance.Blue
                ? Constants.FieldConstants.BLUE_HUB_Y
                : Constants.FieldConstants.RED_HUB_Y;

        isDumping = alliance == Alliance.Blue
                ? pose.getX() > currentHubX
                : pose.getX() < currentHubX;

        if (isDumping) {
            isLeftDump = pose.getY() > Constants.FieldConstants.BLUE_HUB_Y;

            if (alliance == Alliance.Blue) {
                currentHubX = isLeftDump
                        ? Constants.FieldConstants.BLUE_DUMP_LEFT_X
                        : Constants.FieldConstants.BLUE_DUMP_RIGHT_X;
                currentHubY = isLeftDump
                        ? Constants.FieldConstants.BLUE_DUMP_LEFT_Y
                        : Constants.FieldConstants.BLUE_DUMP_RIGHT_Y;
            } else {
                currentHubX = isLeftDump
                        ? Constants.FieldConstants.RED_DUMP_LEFT_X
                        : Constants.FieldConstants.RED_DUMP_RIGHT_X;
                currentHubY = isLeftDump
                        ? Constants.FieldConstants.RED_DUMP_LEFT_Y
                        : Constants.FieldConstants.RED_DUMP_RIGHT_Y;
            }
        }

        currentTargetZ = isDumping ? 0.0
                : (alliance == Alliance.Blue
                        ? Constants.FieldConstants.HUB_BLUE.getZ()
                        : Constants.FieldConstants.HUB_RED.getZ());
        Translation3d currentTarget = new Translation3d(currentHubX, currentHubY, currentTargetZ);

        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond, robotSpeeds.omegaRadiansPerSecond, rotation);

        // Calculate iterative shot with moving robot lookahead (5 iterations)
        // try 8 for the next test
        latestShot = ShotCalculator.calculateIterativeShot(
                pose, fieldSpeeds, currentTarget, 5, isDumping);

        ShotData optimalShot = latestShot.shot();
        Translation3d shotTarget = latestShot.target();
        Translation2d futureTurretPos = latestShot.futureTurretPos();

        // Calculate turret angle from future turret pos to hub
        Translation2d targetDirVector = shotTarget.toTranslation2d().minus(futureTurretPos);
        double targetDirectionRad = Math.atan2(targetDirVector.getY(), targetDirVector.getX());
        double targetDirectionDeg = Math.toDegrees(targetDirectionRad);

        desiredTurretAngle = Utils.wrapAngle(rotation.getDegrees() - targetDirectionDeg);

        latestTurretAngle = isDumping
                ? desiredTurretAngle
                : Utils.clamp(desiredTurretAngle,
                        Constants.TurretConstants.kMinimumAngle,
                        Constants.TurretConstants.kMaximumAngle);

        // Set turret target — it tracks continuously
        turret.target = latestTurretAngle + turretFudge;
        // hood.targetHoodAngle = optimalShot.hoodAngle();

        Translation2d turretPos2d = ShotCalculator.getTurretTranslation(pose);
        targetDistance = turretPos2d.getDistance(
                new Translation2d(currentHubX, currentHubY));
        double optimalRPM = optimalShot.rpm();

        SmartDashboard.putNumber("AutoTrack/TargetDistance", targetDistance);
        SmartDashboard.putNumber("AutoTrack/OptimalTurretAngle", latestTurretAngle);
        SmartDashboard.putNumber("AutoTrack/ActualTurretAngle", turret.getTurretAngle());
        SmartDashboard.putNumber("AutoTrack/DesiredTurretAngle", desiredTurretAngle);
        SmartDashboard.putNumber("AutoTrack/OptimalRPM", optimalRPM);
        SmartDashboard.putNumber("AutoTrack/OptimalHoodAngle", optimalShot.hoodAngle());
        SmartDashboard.putBoolean("AutoTrack/IsDumping", isDumping);
        SmartDashboard.putBoolean("AutoTrack/Tracking", tracking);
        SmartDashboard.putBoolean("AutoTrack/TurretCanReach", canTurretReachTarget());

        // calibration widget for collecting data points
        SmartDashboard.putNumber("Calibration/DistanceToTarget", targetDistance);
        SmartDashboard.putNumber("Calibration/CurrentRPM", shooter.getMotorRPM());
        SmartDashboard.putNumber("Calibration/TargetRPM", shooter.targetVel);
        SmartDashboard.putNumber("Calibration/TargetHoodAngle", hood.targetHoodAngle);
        SmartDashboard.putNumber("Calibration/CurrentHoodAngle", hood.getPosition());
        SmartDashboard.putBoolean("Calibration/IsDumping", isDumping);
        SmartDashboard.putString("Calibration/TargetType",
                isDumping ? (isLeftDump ? "DUMP_LEFT" : "DUMP_RIGHT") : "HUB");
    }

    public CalculatedShot getLatestShot() {
        return latestShot;
    }

    public double getLatestTurretAngle() {
        return latestTurretAngle;
    }

    public boolean isDumping() {
        return isDumping;
    }

    public boolean isTurretOnTarget() {
        if (RobotBase.isSimulation()) {
            return Math.abs(turret.target - latestTurretAngle) < 4.0;
        }
        return Math.abs(turret.getTurretAngle() - latestTurretAngle) < 4.0;
    }

    // for preventing shots (and penalties) if out of range when dumping, returns false if it was clamped
    public boolean canTurretReachTarget() {
        return Math.abs(desiredTurretAngle - latestTurretAngle) < 2.0;
    }


    public boolean isTurretTrulyOnTarget() {
        return isTurretOnTarget() && canTurretReachTarget();
    }

    public double getDesiredTurretAngle() {
        return desiredTurretAngle;
    }

    public void enableTracking() {
        tracking = true;
    }

    public void disableTracking() {
        tracking = false;
    }

    public boolean isTracking() {
        return tracking;
    }

    public double getCurrentTargetX() {
        return currentHubX;
    }

    public double getCurrentTargetY() {
        return currentHubY;
    }

    public double getCurrentTargetZ() {
        return currentTargetZ;
    }
}
