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
 * Factory class that creates commands to chase AprilTags using PhotonVision.
 * Uses the CTRE SwerveRequest pattern via drivetrain.applyRequest().
 */
public class FuelChaseCommand {

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

    // How far away from the tag we want to be (meters) and facing it
    private static final Transform3d TAG_TO_GOAL = new Transform3d(
            new Translation3d(1.0, 0.0, 0.0), // Stay 1m in front of the tag
            new Rotation3d(0.0, 0.0, Math.PI)); // Face the tag (180 degrees)

    /**
     * Creates a command that chases an AprilTag using PhotonVision.
     * Uses PID control and odometry for smooth feedback.
     *
     * @param idToChase     The AprilTag fiducial ID to chase.
     * @param camera        The PhotonCamera to use for target detection.
     * @param drivetrain    The swerve drivetrain subsystem.
     * @param poseProvider  Supplier for the current robot pose.
     * @param robotToCamera Transform from robot center to camera.
     * @return A Command that chases the specified AprilTag.
     */
    public static Command createChaseCommand(
            int idToChase,
            PhotonCamera camera,
            CommandSwerveDrivetrain drivetrain,
            Supplier<Pose2d> poseProvider,
            Transform3d robotToCamera) {

        // Create reusable swerve request
        final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
                .withDeadband(0.1)
                .withRotationalDeadband(0.1)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        // Create PID controllers (persist in closure)
        final ProfiledPIDController xController = new ProfiledPIDController(X_KP, X_KI, X_KD, X_CONSTRAINTS);
        final ProfiledPIDController yController = new ProfiledPIDController(Y_KP, Y_KI, Y_KD, Y_CONSTRAINTS);
        final ProfiledPIDController omegaController = new ProfiledPIDController(OMEGA_KP, OMEGA_KI, OMEGA_KD,
                OMEGA_CONSTRAINTS);

        // Configure tolerances
        xController.setTolerance(X_TOLERANCE);
        yController.setTolerance(Y_TOLERANCE);
        omegaController.setTolerance(OMEGA_TOLERANCE);
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        // State that persists across execute cycles
        final Pose2d[] goalPose = new Pose2d[1]; // Array to allow mutation in lambda
        final PhotonTrackedTarget[] lastTarget = new PhotonTrackedTarget[1];
        final int[] executeCallCount = new int[1];

        // Create command using drivetrain.applyRequest()
        return drivetrain.applyRequest(() -> {
            executeCallCount[0]++;

            // Get robot pose from drivetrain (vision-fused)
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
            Logger.recordOutput("FuelChase/ExecuteCallCount", executeCallCount[0]);

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
                    // Log all visible tag IDs
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
                        if (!target.equals(lastTarget[0])) {
                            lastTarget[0] = target;

                            // Transform: robot → camera → target → goal
                            var cameraPose = robotPose3d.transformBy(robotToCamera);
                            var camToTarget = target.getBestCameraToTarget();
                            var targetPose = cameraPose.transformBy(camToTarget);
                            goalPose[0] = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

                            // Debug: Log transformation chain
                            Logger.recordOutput("FuelChase/CameraPose", cameraPose.toPose2d());
                            Logger.recordOutput("FuelChase/TargetPose3d", targetPose);
                            Logger.recordOutput("FuelChase/TargetPose2d", targetPose.toPose2d());
                            Logger.recordOutput("FuelChase/GoalPose", goalPose[0]);
                            Logger.recordOutput("FuelChase/GoalX", goalPose[0].getX());
                            Logger.recordOutput("FuelChase/GoalY", goalPose[0].getY());
                            Logger.recordOutput("FuelChase/GoalRotDeg", goalPose[0].getRotation().getDegrees());

                            // Update PID goals
                            xController.setGoal(goalPose[0].getX());
                            yController.setGoal(goalPose[0].getY());
                            omegaController.setGoal(goalPose[0].getRotation().getRadians());
                        }
                    }
                }
            }

            // Log if we have a goal to pursue
            Logger.recordOutput("FuelChase/HasGoal", lastTarget[0] != null);
            Logger.recordOutput("FuelChase/HasLastTarget", lastTarget[0] != null);

            // Drive toward goal if we have one
            if (lastTarget[0] == null) {
                Logger.recordOutput("FuelChase/State", "NO_TARGET");
                Logger.recordOutput("FuelChase/OutputX", 0.0);
                Logger.recordOutput("FuelChase/OutputY", 0.0);
                Logger.recordOutput("FuelChase/OutputOmega", 0.0);

                // Stop the drivetrain
                return driveRequest
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0);
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

            // Return swerve request with calculated speeds
            return driveRequest
                    .withVelocityX(xSpeed)
                    .withVelocityY(ySpeed)
                    .withRotationalRate(omegaSpeed);
        }).beforeStarting(() -> {
            // Initialize on command start
            goalPose[0] = null;
            lastTarget[0] = null;
            executeCallCount[0] = 0;

            var robotPose = poseProvider.get();
            omegaController.reset(robotPose.getRotation().getRadians());
            xController.reset(robotPose.getX());
            yController.reset(robotPose.getY());

            // Debug: Log initialization
            Logger.recordOutput("FuelChase/Initialized", true);
            Logger.recordOutput("FuelChase/TargetTagId", idToChase);
            Logger.recordOutput("FuelChase/CameraConnected", camera.isConnected());
        }).finallyDo((interrupted) -> {
            // Log when command ends
            Logger.recordOutput("FuelChase/Ended", true);
            Logger.recordOutput("FuelChase/EndedInterrupted", interrupted);
        });
    }

    /**
     * Convenience method that creates a chase command for tag ID 1 using camera0.
     *
     * @param drivetrain The swerve drivetrain subsystem.
     * @return A Command that chases AprilTag ID 1.
     */
    public static Command createDefaultChaseCommand(CommandSwerveDrivetrain drivetrain) {
        return createChaseCommand(
                1,
                new PhotonCamera(VisionConstants.camera0Name),
                drivetrain,
                () -> drivetrain.getStateCopy().Pose,
                VisionConstants.robotToCamera0);
    }
}