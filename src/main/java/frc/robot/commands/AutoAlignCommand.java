package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.littletonrobotics.junction.Logger;

public class AutoAlignCommand {

    /**
     * Creates a command that pathfinds to the "Start 2" path and then follows it.
     * 
     * @param drivetrain The drivetrain subsystem to use for pathfinding and
     *                   following.
     * @return A command that represents the pathfinding and following sequence.
     */
    public static Command getAutoAlignCommand(CommandSwerveDrivetrain drivetrain) {
        try {
            // Load the "Start 2" path from the deploy directory
            PathPlannerPath path = PathPlannerPath.fromPathFile("trench");

            // Define pathfinding constraints (max velocity, max acceleration, max angular
            // velocity, max angular acceleration)
            // Using reasonable defaults matching last year's or common defaults
            PathConstraints constraints = new PathConstraints(
                    3.0, 3.0,
                    Units.degreesToRadians(540), Units.degreesToRadians(720));

            // Generate the pathfinding then following command
            Command autoAlign = AutoBuilder.pathfindThenFollowPath(path, constraints);

            return autoAlign
                    .beforeStarting(() -> {
                        var pose = drivetrain.getStateCopy().Pose;
                        System.out.println("AutoAlign: Starting at Pose: " + pose);
                        Logger.recordOutput("AutoAlign/Active", true);
                        Logger.recordOutput("AutoAlign/StartPose", pose);
                    })
                    .andThen(() -> System.out.println("AutoAlign: Command finished!"))
                    .finallyDo((interrupted) -> {
                        if (interrupted) {
                            System.out.println("AutoAlign: Command interrupted!");
                        }
                        Logger.recordOutput("AutoAlign/Active", false);
                    });
        } catch (Exception e) {
            // Handle cases where the path might not be found or other errors
            System.err.println("!!! AUTO ALIGN ERROR: " + e.getMessage() + " !!!");
            return Commands.print("!!! AUTO ALIGN ERROR: " + e.getMessage() + " !!!");
        }
    }
}
