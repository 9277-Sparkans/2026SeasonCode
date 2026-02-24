package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.Supplier;
import java.util.Set;

/**
 * Factory class for PathPlanner path-following commands.
 */
public class PathCommands {

    /**
     * Creates a command to follow a pre-made PathPlanner path.
     * 
     * @param pathName The name of the path file (without .path extension).
     *                 Path files should be in src/main/deploy/pathplanner/paths/
     * @return A command that follows the path, or a print command if path fails to
     *         load.
     */
    public static Command followPath(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            return Commands.print("Failed to load path: " + pathName + " - " + e.getMessage());
        }
    }

    /**
     * Creates a command to pathfind to a specific pose and then follow a path.
     * Useful for starting from an arbitrary position.
     * 
     * @param pathName The name of the path file.
     * @return A command that pathfinds to the start of the path, then follows it.
     */
    public static Command pathfindThenFollowPath(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            return AutoBuilder.pathfindThenFollowPath(path, null);
        } catch (Exception e) {
            return Commands.print("Failed to load path: " + pathName + " - " + e.getMessage());
        }
    }

    /**
     * Determines whether to run the "TrenchF" or "TrenchB" path based on the
     * robot's pose.
     *
     * @param poseSupplier A supplier for the robot's current pose
     * @return A command that decides which path to load and run
     */
    public static Command getTrenchCommand(Supplier<Pose2d> poseSupplier) {
        return Commands.defer(() -> {
            Pose2d pose = poseSupplier.get();
            // TODO: adjust this condition to accurately reflect whether the robot
            // is starting from the back or front of the trench.
            // For example, if X coordinate is less than the middle of the field (approx
            // 8.27m):
            boolean goingForward = pose.getX() < 8.27;

            if (goingForward) {
                return pathfindThenFollowPath("TrenchF");
            } else {
                return pathfindThenFollowPath("TrenchB");
            }
        }, Set.of());
    }
}
