package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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

}
