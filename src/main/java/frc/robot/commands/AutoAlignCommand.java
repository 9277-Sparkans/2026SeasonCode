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
            PathPlannerPath path = PathPlannerPath.fromPathFile("climbtest");

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

    /**
     * Determines whether to run the "TrenchF" or "TrenchB" path based on the
     * robot's pose.
     * Starts pathfinding then follows the path, with constraints and logging.
     *
     * @param drivetrain The drivetrain subsystem
     * @return A command that decides which path to load and run
     */
    public static Command getTrenchCommand(CommandSwerveDrivetrain drivetrain) {
        return Commands.defer(() -> {
            var currentPose = drivetrain.getStateCopy().Pose;
            boolean goingForward = currentPose.getX() < 8.27;
            String pathName = goingForward ? "TrenchF" : "TrenchB";

            try {
                PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
                PathConstraints constraints = new PathConstraints(
                        3.0, 3.0,
                        Units.degreesToRadians(540), Units.degreesToRadians(720));

                Command trenchAlign = AutoBuilder.pathfindThenFollowPath(path, constraints);

                return trenchAlign
                        .beforeStarting(() -> {
                            System.out
                                    .println("TrenchAuto: Starting at Pose: " + currentPose + " running: " + pathName);
                            Logger.recordOutput("TrenchAuto/Active", true);
                            Logger.recordOutput("TrenchAuto/StartPose", currentPose);
                        })
                        .andThen(() -> System.out.println("TrenchAuto: Command finished!"))
                        .finallyDo((interrupted) -> {
                            if (interrupted) {
                                System.out.println("TrenchAuto: Command interrupted!");
                            }
                            Logger.recordOutput("TrenchAuto/Active", false);
                        });
            } catch (Exception e) {
                System.err.println("!!! TRENCH AUTO ERROR: " + e.getMessage() + " !!!");
                return Commands.print("!!! TRENCH AUTO ERROR: " + e.getMessage() + " !!!");
            }
        }, java.util.Set.of());
    }
}
