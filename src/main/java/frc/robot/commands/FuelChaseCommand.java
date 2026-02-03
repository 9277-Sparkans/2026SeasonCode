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

import java.util.Optional;
import java.util.function.Supplier;

/**
 * A command that uses PhotonVision to chase and approach an AprilTag.
 * Uses the target's 3D pose from PhotonVision to calculate the goal position
 * and drive towards it using profiled PID controllers.
 */
public class FuelChaseCommand extends Command {

    private static final double X_KP = 3;
    private static final double X_KI = 0;
    private static final double X_KD = 0;
    private static final double X_TOLERANCE = 0.2;

    private static final double Y_KP = 3;
    private static final double Y_KI = 0;
    private static final double Y_KD = 0;
    private static final double Y_TOLERANCE = 0.2;

    private static final double OMEGA_KP = 2;
    private static final double OMEGA_KI = 0;
    private static final double OMEGA_KD = 0;
    private static final double OMEGA_TOLERANCE = Units.degreesToRadians(3);

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

    private final int idToChase;
    private final PhotonCamera camera;
    private final CommandSwerveDrivetrain drivetrain;
    private final Transform3d robotToCamera;

    // How far away from the tag we want to be (meters) and facing it
    private static final Transform3d TAG_TO_GOAL = new Transform3d(
            new Translation3d(1.5, 0.0, 0.0), // Stay 1.5 meters in front of the tag
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

    /**
     * Creates a new FuelChaseCommand.
     *
     * @param idToChase     The AprilTag ID to chase
     * @param camera        The PhotonCamera to use for target detection
     * @param drivetrain    The swerve drivetrain subsystem
     * @param poseProvider  A supplier for the current robot pose (fallback/initial)
     * @param robotToCamera The transform from the robot center to the camera
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
     * Convenience constructor that uses the default camera name and a default
     * tag id (1) and the drivetrain state as the pose supplier.
     *
     * @param drivetrain The swerve drivetrain
     */
    public FuelChaseCommand(CommandSwerveDrivetrain drivetrain) {
        this(1, new PhotonCamera(VisionConstants.camera0Name), drivetrain,
                () -> drivetrain.getStateCopy().Pose, VisionConstants.robotToCamera0);
    }

    @Override
    public void initialize() {
        goalPose = null;
        var robotPose = poseProvider.get();
        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }

    @Override
    public void execute() {
        var robotPose2d = poseProvider.get();
        boolean specificTargetFound = false;

        // Get all unread results from the camera and use the most recent one
        var results = camera.getAllUnreadResults();

        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1); // Get the most recent result

            if (result.hasTargets()) {
                // Find the target with the ID we want to chase
                Optional<PhotonTrackedTarget> targetOpt = result.getTargets().stream()
                        .filter(t -> t.getFiducialId() == idToChase)
                        .findFirst();

                if (targetOpt.isPresent()) {
                    PhotonTrackedTarget target = targetOpt.get();
                    // Get the tag's field-relative pose from the layout
                    Optional<Pose3d> tagPoseOpt = VisionConstants.aprilTagLayout.getTagPose(idToChase);

                    if (tagPoseOpt.isPresent()) {
                        Pose3d tagPose = tagPoseOpt.get();
                        specificTargetFound = true;

                        // Calculate robot pose from vision
                        Pose3d robotPose3d = org.photonvision.PhotonUtils.estimateFieldToRobotAprilTag(
                                target.getBestCameraToTarget(),
                                tagPose,
                                robotToCamera);
                        robotPose2d = robotPose3d.toPose2d();

                        // Calculate the goal pose: where we want the robot to be relative to the tag
                        // TAG_TO_GOAL defines the offset from the tag
                        Pose3d goalPose3d = tagPose.transformBy(TAG_TO_GOAL);
                        goalPose = goalPose3d.toPose2d();

                        // Set the PID goals
                        xController.setGoal(goalPose.getX());
                        yController.setGoal(goalPose.getY());
                        omegaController.setGoal(goalPose.getRotation().getRadians());
                    }
                }
            }
        }

        // Calculate the drive outputs using the PID controllers
        if (specificTargetFound && goalPose != null) {
            double xSpeed = xController.calculate(robotPose2d.getX());
            double ySpeed = yController.calculate(robotPose2d.getY());
            double omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());

            // Clamp speeds to reasonable values
            xSpeed = clamp(xSpeed, -3.0, 3.0);
            ySpeed = clamp(ySpeed, -3.0, 3.0);
            omegaSpeed = clamp(omegaSpeed, -4.0, 4.0);

            // Drive the robot (field-relative)
            drivetrain.setControl(
                    driveRequest
                            .withVelocityX(xSpeed)
                            .withVelocityY(ySpeed)
                            .withRotationalRate(omegaSpeed));
        } else {
            // No valid goal or target lost, stop the robot
            drivetrain.setControl(
                    driveRequest
                            .withVelocityX(0)
                            .withVelocityY(0)
                            .withRotationalRate(0));
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends
        drivetrain.setControl(
                driveRequest
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        // Never finishes on its own - meant to be used with whileTrue()
        return false;
    }

    /**
     * Checks if the robot has reached the goal within tolerance.
     *
     * @return true if all controllers are at their goals
     */
    public boolean atGoal() {
        return xController.atGoal() && yController.atGoal() && omegaController.atGoal();
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}