package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.Vision.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.Supplier;

/**
 * A command that uses PhotonVision to chase and approach an AprilTag.
 * Uses odometry for smooth PID feedback and calculates goal from vision.
 */
public class FuelChaseCommand extends Command {

    // PID Constants for X axis (forward/backward)
    private static final double X_KP = 2.0;
    private static final double X_KI = 0.0;
    private static final double X_KD = 0.0;
    private static final double X_TOLERANCE = 0.1;

    // PID Constants for Y axis (left/right)
    private static final double Y_KP = 2.0;
    private static final double Y_KI = 0.0;
    private static final double Y_KD = 0.0;
    private static final double Y_TOLERANCE = 0.1;

    // PID Constants for rotation
    private static final double OMEGA_KP = 2.0;
    private static final double OMEGA_KI = 0.0;
    private static final double OMEGA_KD = 0.0;
    private static final double OMEGA_TOLERANCE = Units.degreesToRadians(5);

    // Motion profile constraints
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(0.5, 0.5);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(0.5, 0.5);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(Math.PI,
            Math.PI);

    // Command configuration
    private final int idToChase;
    private final PhotonCamera camera;
    private final CommandSwerveDrivetrain drivetrain;
    private final Transform3d robotToCamera;
    private final Supplier<Pose2d> poseProvider;

    // How far away from the tag we want to be (meters) and facing it
    private static final Transform3d TAG_TO_GOAL = new Transform3d(
            new Translation3d(1.0, 0.0, 0.0), // Stay 1m in front of the tag
            new Rotation3d(0.0, 0.0, Math.PI)); // Face the tag (180 degrees)

    // PID controllers
    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController omegaController;

    // Swerve request for field-centric driving
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(0.1)
            .withRotationalDeadband(0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Runtime state
    private Pose2d goalPose;
    private PhotonTrackedTarget lastTarget;
    private int executeCallCount = 0;

    /**
     * Creates a new FuelChaseCommand.
     *
     * @param idToChase     The AprilTag fiducial ID to chase.
     * @param camera        The PhotonCamera to use for target detection.
     * @param drivetrain    The swerve drivetrain subsystem.
     * @param poseProvider  Supplier for the current robot pose.
     * @param robotToCamera Transform from robot center to camera.
     */
    public FuelChaseCommand(
            int idToChase,
            PhotonCamera camera,
            CommandSwerveDrivetrain drivetrain,
            Supplier<Pose2d> poseProvider,
            Transform3d robotToCamera) {
        this.idToChase = idToChase;
        this.camera = camera;
        this.drivetrain = drivetrain;
        this.poseProvider = poseProvider;
        this.robotToCamera = robotToCamera;

        // Initialize PID controllers
        xController = new ProfiledPIDController(X_KP, X_KI, X_KD, X_CONSTRAINTS);
        yController = new ProfiledPIDController(Y_KP, Y_KI, Y_KD, Y_CONSTRAINTS);
        omegaController = new ProfiledPIDController(OMEGA_KP, OMEGA_KI, OMEGA_KD, OMEGA_CONSTRAINTS);

        // Configure tolerances
        xController.setTolerance(X_TOLERANCE);
        yController.setTolerance(Y_TOLERANCE);
        omegaController.setTolerance(OMEGA_TOLERANCE);
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        // Require the drivetrain subsystem
        addRequirements(drivetrain);
    }

    /**
     * Convenience constructor with default camera and tag ID 1.
     *
     * @param drivetrain The swerve drivetrain subsystem.
     */
    public FuelChaseCommand(CommandSwerveDrivetrain drivetrain) {
        this(
                1,
                new PhotonCamera(VisionConstants.camera0Name),
                drivetrain,
                () -> drivetrain.getStateCopy().Pose,
                VisionConstants.robotToCamera0);
    }

    @Override
    public void initialize() {
        goalPose = null;
        lastTarget = null;
        executeCallCount = 0;

        var robotPose = poseProvider.get();
        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());

        // Debug: Log initialization
        Logger.recordOutput("FuelChase/Initialized", true);
        Logger.recordOutput("FuelChase/TargetTagId", idToChase);
        Logger.recordOutput("FuelChase/CameraConnected", camera.isConnected());
    }

    @Override
    public void execute() {
        executeCallCount++;

        // Get robot pose from drivetrain (should be vision-fused)
        var robotPose2d = poseProvider.get();
        var robotPose3d = new Pose3d(
                robotPose2d.getX(),
                robotPose2d.getY(),
                0,
                new Rotation3d(0, 0, robotPose2d.getRotation().getRadians()));

        // Debug: Log robot pose
        Logger.recordOutput("FuelChase/RobotPose", robotPose2d);
        Logger.recordOutput("FuelChase/RobotPoseX", robotPose2d.getX());
        Logger.recordOutput("FuelChase/RobotPoseY", robotPose2d.getY());
        Logger.recordOutput("FuelChase/RobotPoseRotDeg", robotPose2d.getRotation().getDegrees());
        Logger.recordOutput("FuelChase/ExecuteCallCount", executeCallCount);

        // Get camera results
        var results = camera.getAllUnreadResults();
        boolean hasResults = !results.isEmpty();
        boolean hasTargets = hasResults && results.get(results.size() - 1).hasTargets();

        // Debug: Log camera status
        Logger.recordOutput("FuelChase/CameraConnected", camera.isConnected());
        Logger.recordOutput("FuelChase/HasResults", hasResults);
        Logger.recordOutput("FuelChase/HasTargets", hasTargets);
        Logger.recordOutput("FuelChase/ResultCount", results.size());

        if (hasResults) {
            var result = results.get(results.size() - 1);
            Logger.recordOutput("FuelChase/LatestResultTimestamp", result.getTimestampSeconds());
            Logger.recordOutput("FuelChase/TargetCount", result.getTargets().size());

            if (result.hasTargets()) {
                // Log all visible tag IDs for debugging
                int[] visibleTagIds = result.getTargets().stream()
                        .mapToInt(t -> t.getFiducialId())
                        .toArray();
                Logger.recordOutput("FuelChase/VisibleTagIds", visibleTagIds);

                // Find the target with the ID we want to chase
                Optional<PhotonTrackedTarget> targetOpt = result.getTargets().stream()
                        .filter(t -> t.getFiducialId() == idToChase)
                        .filter(t -> t.getPoseAmbiguity() <= 0.2 && t.getPoseAmbiguity() != -1)
                        .findFirst();

                Logger.recordOutput("FuelChase/SpecificTargetFound", targetOpt.isPresent());

                if (targetOpt.isPresent()) {
                    var target = targetOpt.get();

                    // Debug: Log target info
                    Logger.recordOutput("FuelChase/TargetAmbiguity", target.getPoseAmbiguity());
                    Logger.recordOutput("FuelChase/TargetYaw", target.getYaw());
                    Logger.recordOutput("FuelChase/TargetPitch", target.getPitch());
                    Logger.recordOutput("FuelChase/TargetArea", target.getArea());

                    // Only recalculate goal when we see a NEW target
                    if (!target.equals(lastTarget)) {
                        lastTarget = target;

                        // Transform: robot → camera → target → goal
                        var cameraPose = robotPose3d.transformBy(robotToCamera);
                        var camToTarget = target.getBestCameraToTarget();
                        var targetPose = cameraPose.transformBy(camToTarget);
                        goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

                        // Debug: Log transformation chain
                        Logger.recordOutput("FuelChase/CameraPose", cameraPose.toPose2d());
                        Logger.recordOutput("FuelChase/TargetPose3d", targetPose);
                        Logger.recordOutput("FuelChase/TargetPose2d", targetPose.toPose2d());
                        Logger.recordOutput("FuelChase/GoalPose", goalPose);
                        Logger.recordOutput("FuelChase/GoalX", goalPose.getX());
                        Logger.recordOutput("FuelChase/GoalY", goalPose.getY());
                        Logger.recordOutput("FuelChase/GoalRotDeg", goalPose.getRotation().getDegrees());

                        // Update PID goals
                        xController.setGoal(goalPose.getX());
                        yController.setGoal(goalPose.getY());
                        omegaController.setGoal(goalPose.getRotation().getRadians());
                    }
                }
            }
        }

        // Log if we have a goal to pursue
        Logger.recordOutput("FuelChase/HasGoal", lastTarget != null);
        Logger.recordOutput("FuelChase/HasLastTarget", lastTarget != null);

        // Drive toward goal if we have one
        if (lastTarget == null) {
            Logger.recordOutput("FuelChase/State", "NO_TARGET");
            Logger.recordOutput("FuelChase/OutputX", 0.0);
            Logger.recordOutput("FuelChase/OutputY", 0.0);
            Logger.recordOutput("FuelChase/OutputOmega", 0.0);

            // Stop the drivetrain
            var stopRequest = driveRequest
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0);
            drivetrain.setControl(stopRequest);
            Logger.recordOutput("FuelChase/SetControlCalled", true);
            return;
        }

        Logger.recordOutput("FuelChase/State", "CHASING");

        // Calculate PID outputs
        double xSpeed = xController.calculate(robotPose2d.getX());
        double ySpeed = yController.calculate(robotPose2d.getY());
        double omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());

        // Debug: Log PID errors and outputs before clamping
        Logger.recordOutput("FuelChase/PID/XError", xController.getPositionError());
        Logger.recordOutput("FuelChase/PID/YError", yController.getPositionError());
        Logger.recordOutput("FuelChase/PID/OmegaError", omegaController.getPositionError());
        Logger.recordOutput("FuelChase/PID/XSpeedRaw", xSpeed);
        Logger.recordOutput("FuelChase/PID/YSpeedRaw", ySpeed);
        Logger.recordOutput("FuelChase/PID/OmegaSpeedRaw", omegaSpeed);

        // Zero out speeds if at goal
        if (xController.atGoal()) {
            xSpeed = 0;
        }
        if (yController.atGoal()) {
            ySpeed = 0;
        }
        if (omegaController.atGoal()) {
            omegaSpeed = 0;
        }

        // Debug: Log final outputs
        Logger.recordOutput("FuelChase/OutputX", xSpeed);
        Logger.recordOutput("FuelChase/OutputY", ySpeed);
        Logger.recordOutput("FuelChase/OutputOmega", omegaSpeed);
        Logger.recordOutput("FuelChase/XAtGoal", xController.atGoal());
        Logger.recordOutput("FuelChase/YAtGoal", yController.atGoal());
        Logger.recordOutput("FuelChase/OmegaAtGoal", omegaController.atGoal());

        // Apply swerve request
        var driveCmd = driveRequest
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(omegaSpeed);
        drivetrain.setControl(driveCmd);
        Logger.recordOutput("FuelChase/SetControlCalled", true);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("FuelChase/Ended", true);
        Logger.recordOutput("FuelChase/EndedInterrupted", interrupted);

        // Stop the drivetrain
        drivetrain.setControl(
                driveRequest
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * Checks if all PID controllers are at their goals.
     *
     * @return True if at goal position and orientation.
     */
    public boolean atGoal() {
        return xController.atGoal() && yController.atGoal() && omegaController.atGoal();
    }
}