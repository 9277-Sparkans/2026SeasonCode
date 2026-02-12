package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;
import java.util.function.Supplier;

/**
 * Command that chases an AprilTag using PhotonVision.
 * Refactored to extend Command directly for better performance and reduced
 * logging.
 */
public class FuelChaseCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final PhotonCamera camera;
    private final Supplier<Pose3d> poseProvider;
    private final Transform3d robotToCamera;
    private final int idToChase;

    // PID Constants
    private static final double X_KP = 2.0;
    private static final double X_KI = 0.0;
    private static final double X_KD = 0.0;

    private static final double Y_KP = 2.0;
    private static final double Y_KI = 0.0;
    private static final double Y_KD = 0.0;

    private static final double OMEGA_KP = 2.0;
    private static final double OMEGA_KI = 0.0;
    private static final double OMEGA_KD = 0.0;

    // Constraints
    private static final TrapezoidProfile.Constraints LINEAR_CONSTRAINTS = new TrapezoidProfile.Constraints(1.0, 1.0); // m/s,
                                                                                                                       // m/s^2
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(Math.PI,
            Math.PI); // rad/s, rad/s^2

    // Tolerances
    private static final double LINEAR_TOLERANCE = 0.1; // meters
    private static final double OMEGA_TOLERANCE = Units.degreesToRadians(5);

    // Goal Configuration
    // Standard AprilTag: Z is Normal to the tag (out). X is Right. Y is Down.
    // To be 1m in front of the tag, we want Z=1.0.
    private static final Transform3d TAG_TO_GOAL = new Transform3d(
            new Translation3d(1.0, 0.0, 0.0), 

            new Rotation3d(0, 0, Math.PI)); // Face grid (180 deg)

    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController omegaController;

    private final SwerveRequest.FieldCentric driveRequest;

    private PhotonTrackedTarget lastTarget;
    private Pose2d goalPose;

    public FuelChaseCommand(
            int idToChase,
            PhotonCamera camera,
            CommandSwerveDrivetrain drivetrain,
            Supplier<Pose3d> poseProvider,
            Transform3d robotToCamera) {

        this.idToChase = idToChase;
        this.camera = camera;
        this.drivetrain = drivetrain;
        this.poseProvider = poseProvider;
        this.robotToCamera = robotToCamera;

        this.xController = new ProfiledPIDController(X_KP, X_KI, X_KD, LINEAR_CONSTRAINTS);
        this.yController = new ProfiledPIDController(Y_KP, Y_KI, Y_KD, LINEAR_CONSTRAINTS);
        this.omegaController = new ProfiledPIDController(OMEGA_KP, OMEGA_KI, OMEGA_KD, OMEGA_CONSTRAINTS);

        xController.setTolerance(LINEAR_TOLERANCE);
        yController.setTolerance(LINEAR_TOLERANCE);
        omegaController.setTolerance(OMEGA_TOLERANCE);
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        this.driveRequest = new SwerveRequest.FieldCentric()
                .withDeadband(0.1)
                .withRotationalDeadband(0.1)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        var robotPose = poseProvider.get();
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
        // Using getZ() for rotation around Z (Yaw) from Rotation3d
        omegaController.reset(robotPose.getRotation().getZ());

        lastTarget = null;
        goalPose = null;
    }

    @Override
    public void execute() {
        var robotPose = poseProvider.get();
        var results = camera.getAllUnreadResults();

        // Process new vision data if available
        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                Optional<PhotonTrackedTarget> targetOpt = result.getTargets().stream()
                        .filter(t -> t.getFiducialId() == idToChase)
                        .filter(t -> t.getPoseAmbiguity() <= 0.2 && t.getPoseAmbiguity() != -1)
                        .findFirst();

                if (targetOpt.isPresent()) {
                    var target = targetOpt.get();
                    if (lastTarget == null || !target.equals(lastTarget)) {
                        lastTarget = target;
                        updateGoal(target, robotPose);
                    }
                }
            }
        }

        // Drive logic
        if (goalPose == null) {
            drivetrain.setControl(driveRequest
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0));
            return;
        }

        double xSpeed = xController.calculate(robotPose.getX(), goalPose.getX());
        double ySpeed = yController.calculate(robotPose.getY(), goalPose.getY());

        double omegaSpeed = omegaController.calculate(robotPose.getRotation().getZ(),
                goalPose.getRotation().getRadians());

        if (xController.atGoal())
            xSpeed = 0;
        if (yController.atGoal())
            ySpeed = 0;
        if (omegaController.atGoal())
            omegaSpeed = 0;

        drivetrain.setControl(driveRequest
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(omegaSpeed));
    }

    private void updateGoal(PhotonTrackedTarget target, Pose3d robotPose3d) {
        // Transform: Robot -> Camera -> Target -> Goal
        // 1. Robot Pose (Field Frame) - now passed as Pose3d directly

        // 2. Camera Pose (Field Frame) = Robot Pose + RobotToCamera
        var cameraPose = robotPose3d.transformBy(robotToCamera);

        // 3. Target Pose (Field Frame) = Camera Pose + CameraToTarget
        var camToTarget = target.getBestCameraToTarget();
        var targetPose = cameraPose.transformBy(camToTarget);

        // 4. Goal Pose (Field Frame) = Target Pose + TargetToGoal
        // Note: targetPose is in Field Frame. transformBy applies the transform in the
        // Target's local frame.
        // If Tag Z is Normal (Out), and X is Right:
        // Then Translation(1,0,0) is 1m Right. Translation(0,0,1) is 1m Front.
        // Previous code used (1,0,0).
        var goalPose3d = targetPose.transformBy(TAG_TO_GOAL);

        this.goalPose = goalPose3d.toPose2d();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
    }
}