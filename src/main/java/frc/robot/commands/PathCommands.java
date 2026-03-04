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
            return Commands.none();
        }
    }

    /**
     * Creates a command to follow a Choreo trajectory.
     * 
     * @param pathName The name of the Choreo trajectory file (without .traj
     *                 extension).
     * @return A command that follows the trajectory.
     */
    public static Command followChoreoPath(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(pathName);
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            return Commands.none();
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
            return Commands.none();
        }
    }

    /**
     * Creates a command to pathfind to a Choreo trajectory and then follow it.
     * 
     * @param pathName The name of the Choreo trajectory file.
     * @return A command that pathfinds to the start of the trajectory, then follows
     *         it.
     */
    public static Command pathfindThenFollowChoreoPath(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(pathName);
            return AutoBuilder.pathfindThenFollowPath(path, null);
        } catch (Exception e) {
            return Commands.none();
        }
    }

}
