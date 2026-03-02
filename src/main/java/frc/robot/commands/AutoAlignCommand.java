package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants.FieldConstants;
import org.littletonrobotics.junction.Logger;

public class AutoAlignCommand {

    // pathfind constraints
    private static final PathConstraints CONSTRAINTS = new PathConstraints(
            3.0, 3.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

    // jeet autos
    // public static Command getS1DPR_C(CommandSwerveDrivetrain drivetrain) {
    // return buildAuto("S1.1-S-DPR-C", drivetrain);
    // }

    // public static Command getS1DPR(CommandSwerveDrivetrain drivetrain) {
    // return buildAuto("S1.1-S-DPR", drivetrain);
    // }

    // public static Command getS2DP(CommandSwerveDrivetrain drivetrain) {
    // return buildAuto("S2.DP", drivetrain);
    // }

    // public static Command getS2HP(CommandSwerveDrivetrain drivetrain) {
    // return buildAuto("S2.HP", drivetrain);
    // }

    // public static Command getS3HP(CommandSwerveDrivetrain drivetrain) {
    // return buildAuto("S3.HP", drivetrain);
    // }

    // auto align (pathfind then follow path)
    public static Command getAutoAlignCommand(CommandSwerveDrivetrain drivetrain) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile("joelclimb");
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

    // trench forward or back
    public static Command getTrenchCommand(CommandSwerveDrivetrain drivetrain) {
        return Commands.defer(() -> {
            var currentPose = drivetrain.getStateCopy().Pose;

            // Use hub pose as reference: aligned with both trenches on each axis
            boolean isRed = DriverStation.getAlliance()
                    .orElse(Alliance.Blue) == Alliance.Red;
            double hubX = isRed ? FieldConstants.RED_HUB_X
                    : FieldConstants.BLUE_HUB_X;
            double hubY = isRed ? FieldConstants.RED_HUB_Y
                    : FieldConstants.BLUE_HUB_Y;

            boolean goingForward = currentPose.getX() < hubX; // neutral-zone side of hub → drive forward into trench
            boolean rightTrench = currentPose.getY() <= hubY; // below hub Y → right trench

            String side = rightTrench ? "R" : "L";
            String direction = goingForward ? "F" : "B";
            String pathName = "Trench" + direction + side; // e.g. TrenchFR, TrenchBL

            try {
                PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
                Command trenchAlign = AutoBuilder.pathfindThenFollowPath(path, CONSTRAINTS);

                return trenchAlign
                        .beforeStarting(() -> {
                            System.out.println(
                                    "TrenchAuto: Starting at Pose: " + currentPose + " running: " + pathName);
                            Logger.recordOutput("TrenchAuto/Active", true);
                            Logger.recordOutput("TrenchAuto/StartPose", currentPose);
                            Logger.recordOutput("TrenchAuto/Path", pathName);
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

    // autobuilder
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
