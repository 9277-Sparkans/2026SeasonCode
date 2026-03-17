package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants.FieldConstants;

import java.util.Set;

import org.littletonrobotics.junction.Logger;

public class AutoAlignCommand {

    // pathfind constraints
    private static final PathConstraints CONSTRAINTS = new PathConstraints(
            5.5, 5.5,
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
    // public static Command getAutoAlignCommand(CommandSwerveDrivetrain drivetrain)
    // {
    // try {
    // PathPlannerPath path = PathPlannerPath.fromPathFile("joelclimb");
    // Command autoAlign = AutoBuilder.pathfindThenFollowPath(path, CONSTRAINTS);

    // return autoAlign
    // .beforeStarting(() -> {
    // var pose = drivetrain.getStateCopy().Pose;
    // System.out.println("AutoAlign: Starting at Pose: " + pose);
    // Logger.recordOutput("AutoAlign/Active", true);
    // Logger.recordOutput("AutoAlign/StartPose", pose);
    // })
    // .andThen(() -> System.out.println("AutoAlign: Command finished!"))
    // .finallyDo((interrupted) -> {
    // if (interrupted) {
    // System.out.println("AutoAlign: Command interrupted!");
    // }
    // Logger.recordOutput("AutoAlign/Active", false);
    // });

    // outpost autoalign
    public static Command getOutpostCommand(CommandSwerveDrivetrain drivetrain) {
        try {
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("shoot3tooutpost");
            Command outpostAlign = AutoBuilder.pathfindThenFollowPath(path, CONSTRAINTS);

            return outpostAlign
                    .beforeStarting(() -> {
                        var pose = drivetrain.getStateCopy().Pose;
                        Logger.recordOutput("OutpostAuto/Active", true);
                        Logger.recordOutput("OutpostAuto/StartPose", pose);
                    })
                    .finallyDo((interrupted) -> {
                        Logger.recordOutput("OutpostAuto/Active", false);
                    });
        } catch (Exception e) {
            return Commands.none();
        }
    }

    // ─── shared helper ───────────────────────────────────────────────────────────

    private static Command buildPathCommand(String logPrefix, String pathName, Pose2d startPose) {
        try {
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(pathName);
            return AutoBuilder.pathfindThenFollowPath(path, CONSTRAINTS)
                    .beforeStarting(() -> {
                        Logger.recordOutput(logPrefix + "/Active", true);
                        Logger.recordOutput(logPrefix + "/StartPose", startPose);
                        Logger.recordOutput(logPrefix + "/Path", pathName);
                    })
                    .finallyDo(interrupted -> Logger.recordOutput(logPrefix + "/Active", false));
        } catch (Exception e) {
            return Commands.none();
        }
    }

    // ─── auto climb ──────────────────────────────────────────────────────────────

    public static Command getAutoClimbCommand(CommandSwerveDrivetrain drivetrain) {
        return Commands.defer(() -> {
            var pose = drivetrain.getStateCopy().Pose;
            boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

            boolean isLeft = isRed
                    ? pose.getY() < FieldConstants.RED_HUB_Y
                    : pose.getY() > FieldConstants.BLUE_HUB_Y;

            String pathName = isLeft ? "ClimbL" : "ClimbR";
            return buildPathCommand("ClimbAuto", pathName, pose);
        }, Set.of());
    }

    // ─── trench ──────────────────────────────────────────────────────────────────

    public static Command getTrenchCommand(CommandSwerveDrivetrain drivetrain) {
        return Commands.defer(() -> {
            var pose = drivetrain.getStateCopy().Pose;
            boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

            boolean goingForward = isRed
                    ? pose.getX() > FieldConstants.RED_HUB_X
                    : pose.getX() < FieldConstants.BLUE_HUB_X;

            boolean rightTrench = isRed
                    ? pose.getY() >= FieldConstants.RED_HUB_Y
                    : pose.getY() <= FieldConstants.BLUE_HUB_Y;

            Pose2d targetPose;
            if (goingForward) { // F
                if (rightTrench) { // R
                    targetPose = new Pose2d(7.03864860534668, 1.0326759815216064,
                            new edu.wpi.first.math.geometry.Rotation2d());
                } else { // L
                    targetPose = new Pose2d(7.03864860534668, 7.142057418823242,
                            new edu.wpi.first.math.geometry.Rotation2d());
                }
            } else { // B
                if (rightTrench) { // R
                    targetPose = new Pose2d(2.642042398452759, 0.7739725112915039,
                            new edu.wpi.first.math.geometry.Rotation2d());
                } else { // L
                    targetPose = new Pose2d(2.642042398452759, 7.311770915985107,
                            new edu.wpi.first.math.geometry.Rotation2d());
                }
            }

            return AutoBuilder.pathfindToPoseFlipped(targetPose, CONSTRAINTS)
                    .beforeStarting(() -> {
                        Logger.recordOutput("TrenchAuto/Active", true);
                        Logger.recordOutput("TrenchAuto/TargetPose", targetPose);
                    })
                    .finallyDo((interrupted) -> {
                        Logger.recordOutput("TrenchAuto/Active", false);
                    });
        }, Set.of());
    }

    // autobuilder
    private static Command buildAuto(String autoName, CommandSwerveDrivetrain drivetrain) {
        try {
            Command auto = AutoBuilder.buildAuto(autoName);
            return auto
                    .beforeStarting(() -> {
                        var pose = drivetrain.getStateCopy().Pose;
                        Logger.recordOutput("Auto/ActiveRoutine", autoName);
                        Logger.recordOutput("Auto/StartPose", pose);
                    })
                    .finallyDo((interrupted) -> {
                        Logger.recordOutput("Auto/ActiveRoutine", "none");
                    });
        } catch (Exception e) {
            return Commands.none();
        }
    }
}
