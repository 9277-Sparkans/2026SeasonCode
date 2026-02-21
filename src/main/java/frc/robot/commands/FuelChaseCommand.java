package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;
import java.util.function.Supplier;

/**
 * Command that chases an AprilTag using PhotonVision and PathPlanner's
 * pathfindToPose.
 */
public class FuelChaseCommand extends Command {

    private final PhotonCamera camera;
    private final Supplier<Pose3d> poseProvider;
    private final Transform3d robotToCamera;
    private final int idToChase;

    private static final Transform3d TAG_TO_GOAL = new Transform3d(
            new Translation3d(1.0, 0.0, 0.0),
            new Rotation3d(0, 0, Math.PI)); // Face grid (180 deg)

    private PhotonTrackedTarget lastTarget;
    private Pose2d currentGoalPose;
    private Command currentPathfindingCommand;

    // PathPlanner Constraints
    private static final PathConstraints CONSTRAINTS = new PathConstraints(
            2.0, // max velocity m/s
            2.0, // max acceleration m/s^2
            Units.degreesToRadians(180), // max angular velocity rad/s
            Units.degreesToRadians(180) // max angular acceleration rad/s^2
    );

    // Re-path if the target estimate moves by more than this amount
    private static final double REPATH_DISTANCE_THRESHOLD = 0.2; // meters
    private static final double REPATH_ANGLE_THRESHOLD = Units.degreesToRadians(10); // rad

    public FuelChaseCommand(
            int idToChase,
            PhotonCamera camera,
            Supplier<Pose3d> poseProvider,
            Transform3d robotToCamera) {

        this.idToChase = idToChase;
        this.camera = camera;
        this.poseProvider = poseProvider;
        this.robotToCamera = robotToCamera;

    }

    @Override
    public void initialize() {
        lastTarget = null;
        currentGoalPose = null;
        currentPathfindingCommand = null;
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
    }

    private void updateGoal(PhotonTrackedTarget target, Pose3d robotPose3d) {
        // Transform: Robot -> Camera -> Target -> Goal
        var cameraPose = robotPose3d.transformBy(robotToCamera);
        var camToTarget = target.getBestCameraToTarget();
        var targetPose = cameraPose.transformBy(camToTarget);
        var goalPose3d = targetPose.transformBy(TAG_TO_GOAL);

        Pose2d newGoalPose = goalPose3d.toPose2d();

        boolean shouldRepath = false;
        if (currentGoalPose == null || currentPathfindingCommand == null) {
            shouldRepath = true;
        } else {
            // Check if the target has moved significantly
            double distance = currentGoalPose.getTranslation().getDistance(newGoalPose.getTranslation());
            double angleDiff = Math.abs(currentGoalPose.getRotation().minus(newGoalPose.getRotation()).getRadians());

            if (distance > REPATH_DISTANCE_THRESHOLD || angleDiff > REPATH_ANGLE_THRESHOLD) {
                shouldRepath = true;
            }
        }

        if (shouldRepath) {
            if (currentPathfindingCommand != null) {
                currentPathfindingCommand.cancel();
            }

            currentGoalPose = newGoalPose;
            // 0.0 is the goal end velocity
            currentPathfindingCommand = AutoBuilder.pathfindToPose(currentGoalPose, CONSTRAINTS, 0.0);
            edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().schedule(currentPathfindingCommand);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (currentPathfindingCommand != null) {
            currentPathfindingCommand.cancel();
            currentPathfindingCommand = null;
        }
        currentGoalPose = null;
        lastTarget = null;
    }

    @Override
    public boolean isFinished() {
        // This command runs until interrupted (e.g., button release in RobotContainer).
        // It continuously tracks and updates the pathfinder if needed.
        return false;
    }
}