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

    // Shared pathfinding constraints for all auto-align commands
    private static final PathConstraints CONSTRAINTS = new PathConstraints(
            3.0, 3.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

    // -------------------------------------------------------------------------
    // Auto routines (from PathPlanner .auto files)
    // -------------------------------------------------------------------------

    /** S1 → Shoot → Depost → Return cycle, with climb at the end. */
    public static Command getS1DPR_C(CommandSwerveDrivetrain drivetrain) {
        return buildAuto("S1.1-S-DPR-C", drivetrain);
    }

    /** S1 → Shoot → Depost → Return cycle, no climb. */
    public static Command getS1DPR(CommandSwerveDrivetrain drivetrain) {
        return buildAuto("S1.1-S-DPR", drivetrain);
    }

    /** S2 start → Depost. */
    public static Command getS2DP(CommandSwerveDrivetrain drivetrain) {
        return buildAuto("S2.DP", drivetrain);
    }

    /** S2 start → Human Player station. */
    public static Command getS2HP(CommandSwerveDrivetrain drivetrain) {
        return buildAuto("S2.HP", drivetrain);
    }

    /** S3 start → Human Player station. */
    public static Command getS3HP(CommandSwerveDrivetrain drivetrain) {
        return buildAuto("S3.HP", drivetrain);
    }

    // -------------------------------------------------------------------------
    // Pathfind-then-follow helpers (teleop use)
    // -------------------------------------------------------------------------

    /**
     * Pathfinds to the start of "climbtest" then follows it.
     * Wired to the Y button in RobotContainer.
     */
    public static Command getAutoAlignCommand(CommandSwerveDrivetrain drivetrain) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile("climbtest");
            Command autoAlign = AutoBuilder.pathfindThenFollowPath(path, CONSTRAINTS);

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
            System.err.println("!!! AUTO ALIGN ERROR: " + e.getMessage() + " !!!");
            return Commands.print("!!! AUTO ALIGN ERROR: " + e.getMessage() + " !!!");
        }
    }

    /**
     * Determines whether to run the "TrenchF" or "TrenchB" path based on the
     * robot's pose. Starts pathfinding then follows the path.
     */
    public static Command getTrenchCommand(CommandSwerveDrivetrain drivetrain) {
        return Commands.defer(() -> {
            var currentPose = drivetrain.getStateCopy().Pose;
            boolean goingForward = currentPose.getX() < 8.27;
            String pathName = goingForward ? "TrenchF" : "TrenchB";

            try {
                PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
                Command trenchAlign = AutoBuilder.pathfindThenFollowPath(path, CONSTRAINTS);

                return trenchAlign
                        .beforeStarting(() -> {
                            System.out.println(
                                    "TrenchAuto: Starting at Pose: " + currentPose + " running: " + pathName);
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

    // -------------------------------------------------------------------------
    // Internal helpers
    // -------------------------------------------------------------------------

    /**
     * Builds a full autonomous routine from a PathPlanner .auto file.
     * Wraps the AutoBuilder command with standard logging.
     *
     * @param autoName   The name of the .auto file (without extension).
     * @param drivetrain The drivetrain for logging start pose.
     * @return The auto command, or a print command on failure.
     */
    private static Command buildAuto(String autoName, CommandSwerveDrivetrain drivetrain) {
        try {
            Command auto = AutoBuilder.buildAuto(autoName);
            return auto
                    .beforeStarting(() -> {
                        var pose = drivetrain.getStateCopy().Pose;
                        System.out.println("Auto [" + autoName + "]: Starting at Pose: " + pose);
                        Logger.recordOutput("Auto/ActiveRoutine", autoName);
                        Logger.recordOutput("Auto/StartPose", pose);
                    })
                    .andThen(() -> System.out.println("Auto [" + autoName + "]: Finished!"))
                    .finallyDo((interrupted) -> {
                        if (interrupted) {
                            System.out.println("Auto [" + autoName + "]: Interrupted!");
                        }
                        Logger.recordOutput("Auto/ActiveRoutine", "none");
                    });
        } catch (Exception e) {
            System.err.println("!!! AUTO BUILD ERROR [" + autoName + "]: " + e.getMessage() + " !!!");
            return Commands.print("!!! AUTO BUILD ERROR [" + autoName + "]: " + e.getMessage() + " !!!");
        }
    }
}
