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

    private static final double X_KP = 2;
    private static final double X_KI = 0;
    private static final double X_KD = 0;
    private static final double X_TOLERANCE = 0.1;

    private static final double Y_KP = 2;
    private static final double Y_KI = 0;
    private static final double Y_KD = 0;
    private static final double Y_TOLERANCE = 0.1;

    private static final double OMEGA_KP = 2;
    private static final double OMEGA_KI = 0;
    private static final double OMEGA_KD = 0;
    private static final double OMEGA_TOLERANCE = Units.degreesToRadians(5);

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(0.5, 0.5);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(0.5, 0.5);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(Math.PI,
            Math.PI);

    private final int idToChase;
    private final PhotonCamera camera;
    private final CommandSwerveDrivetrain drivetrain;
    private final Transform3d robotToCamera;

    // How far away from the tag we want to be (meters) and facing it
    private static final Transform3d TAG_TO_GOAL = new Transform3d(
            new Translation3d(1.0, 0.0, 0.0), // Stay 1m in front of the tag
            new Rotation3d(0.0, 0.0, Math.PI)); // Face the tag (180 degrees)

    private final Supplier<Pose2d> poseProvider;

    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController omegaController;

    // Swerve request for field-centric driving
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(0.1)
            .withRotationalDeadband(0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private Pose2d goalPose;
    private PhotonTrackedTarget lastTarget;

    /**
     * Creates a new FuelChaseCommand.
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

        xController = new ProfiledPIDController(X_KP, X_KI, X_KD, X_CONSTRAINTS);
        yController = new ProfiledPIDController(Y_KP, Y_KI, Y_KD, Y_CONSTRAINTS);
        omegaController = new ProfiledPIDController(OMEGA_KP, OMEGA_KI, OMEGA_KD, OMEGA_CONSTRAINTS);

        xController.setTolerance(X_TOLERANCE);
        yController.setTolerance(Y_TOLERANCE);
        omegaController.setTolerance(OMEGA_TOLERANCE);
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain);
    }

    /**
     * Convenience constructor with default camera and tag ID 1.
     */
    public FuelChaseCommand(CommandSwerveDrivetrain drivetrain) {
        this(1, new PhotonCamera(VisionConstants.camera0Name), drivetrain,
                () -> drivetrain.getStateCopy().Pose, VisionConstants.robotToCamera0);
    }

    @Override
    public void initialize() {
        goalPose = null;
        lastTarget = null;
        var robotPose = poseProvider.get();
        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }

    @Override
    public void execute() {
        // Use odometry for stable PID feedback
        var robotPose2d = poseProvider.get();
        var robotPose3d = new Pose3d(
                robotPose2d.getX(),
                robotPose2d.getY(),
                0,
                new Rotation3d(0, 0, robotPose2d.getRotation().getRadians()));

        Logger.recordOutput("FuelChase/RobotPose", robotPose2d);

        // Get camera results
        var results = camera.getAllUnreadResults();
        boolean hasTargets = !results.isEmpty() && results.get(results.size() - 1).hasTargets();
        Logger.recordOutput("FuelChase/HasTargets", hasTargets);

        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);

            if (result.hasTargets()) {
                // Find the target with the ID we want to chase
                Optional<PhotonTrackedTarget> targetOpt = result.getTargets().stream()
                        .filter(t -> t.getFiducialId() == idToChase)
                        .filter(t -> t.getPoseAmbiguity() <= 0.2 && t.getPoseAmbiguity() != -1)
                        .findFirst();

                Logger.recordOutput("FuelChase/SpecificTargetFound", targetOpt.isPresent());

                if (targetOpt.isPresent()) {
                    var target = targetOpt.get();

                    // Only recalculate goal when we see a NEW target
                    if (!target.equals(lastTarget)) {
                        lastTarget = target;

                        // Transform: robot → camera → target → goal
                        var cameraPose = robotPose3d.transformBy(robotToCamera);
                        var camToTarget = target.getBestCameraToTarget();
                        var targetPose = cameraPose.transformBy(camToTarget);
                        goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

                        Logger.recordOutput("FuelChase/TargetPose", targetPose.toPose2d());
                        Logger.recordOutput("FuelChase/GoalPose", goalPose);

                        // Update PID goals
                        xController.setGoal(goalPose.getX());
                        yController.setGoal(goalPose.getY());
                        omegaController.setGoal(goalPose.getRotation().getRadians());
                    }
                }
            }
        }

        // Drive toward goal if we have one
        if (lastTarget == null) {
            Logger.recordOutput("FuelChase/OutputX", 0.0);
            Logger.recordOutput("FuelChase/OutputY", 0.0);
            Logger.recordOutput("FuelChase/OutputOmega", 0.0);
            drivetrain.setControl(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
            return;
        }

        // Calculate PID outputs
        double xSpeed = xController.calculate(robotPose2d.getX());
        if (xController.atGoal())
            xSpeed = 0;

        double ySpeed = yController.calculate(robotPose2d.getY());
        if (yController.atGoal())
            ySpeed = 0;

        double omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
        if (omegaController.atGoal())
            omegaSpeed = 0;

        Logger.recordOutput("FuelChase/OutputX", xSpeed);
        Logger.recordOutput("FuelChase/OutputY", ySpeed);
        Logger.recordOutput("FuelChase/OutputOmega", omegaSpeed);

        // Drive field-relative
        drivetrain.setControl(
                driveRequest
                        .withVelocityX(xSpeed)
                        .withVelocityY(ySpeed)
                        .withRotationalRate(omegaSpeed));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public boolean atGoal() {
        return xController.atGoal() && yController.atGoal() && omegaController.atGoal();
    }
}