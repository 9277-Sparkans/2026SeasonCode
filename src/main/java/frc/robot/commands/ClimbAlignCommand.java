package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DriveAssistConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Climb.ClimbState;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import static edu.wpi.first.units.Units.MetersPerSecond;
import frc.robot.generated.TunerConstants;

public class ClimbAlignCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Climb climb;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;

    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric();

    private Pose2d prealigntargetPose;
    private Pose2d preTargetPose;
    private Pose2d finalTargetPose;

    private enum AlignState {
        PRE_ALIGN, PRE_CLIMB, FINAL_CLIMB
    }

    private AlignState currentState;

    private static final double X_TOLERANCE = 0.02;
    private static final double Y_TOLERANCE = 0.01;
    private static final double THETA_TOLERANCE = Math.toRadians(5.0);

    public ClimbAlignCommand(CommandSwerveDrivetrain drivetrain, Climb climb) {
        this.drivetrain = drivetrain;
        this.climb = climb;

        this.xController = new PIDController(DriveAssistConstants.TRANSLATION_kP, DriveAssistConstants.TRANSLATION_kI,
                DriveAssistConstants.TRANSLATION_kD);
        this.yController = new PIDController(DriveAssistConstants.TRANSLATION_kP, DriveAssistConstants.TRANSLATION_kI,
                DriveAssistConstants.TRANSLATION_kD);
        this.thetaController = new PIDController(DriveAssistConstants.ROTATION_kP, DriveAssistConstants.ROTATION_kI,
                DriveAssistConstants.ROTATION_kD);

        this.thetaController.enableContinuousInput(-Math.PI, Math.PI);

        this.xController.setTolerance(X_TOLERANCE);
        this.yController.setTolerance(Y_TOLERANCE);
        this.thetaController.setTolerance(THETA_TOLERANCE);

        addRequirements(drivetrain, climb);
    }

    @Override
    public void initialize() {
        climb.climbUp();

        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        Pose2d currentPose = drivetrain.getStateCopy().Pose;

        boolean isUpperChain = currentPose.getY() > FieldConstants.BLUE_HUB_Y;

        double preAlignX, preAlignY, preX, preY, finalX, finalY;
        Rotation2d targetRot;

        if (isRed) {
            targetRot = isUpperChain ? Rotation2d.fromDegrees(FieldConstants.RED_RIGHT_CLIMB_HEADING)
                    : Rotation2d.fromDegrees(FieldConstants.RED_LEFT_CLIMB_HEADING);

            preAlignX = isUpperChain ? FieldConstants.RED_PRE_ALIGN_RIGHT_X : FieldConstants.RED_PRE_ALIGN_LEFT_X;
            preAlignY = isUpperChain ? FieldConstants.RED_PRE_ALIGN_RIGHT_Y : FieldConstants.RED_PRE_ALIGN_LEFT_Y;

            preX = isUpperChain ? FieldConstants.RED_PRE_RIGHT_CLIMB_X : FieldConstants.RED_PRE_LEFT_CLIMB_X;
            preY = isUpperChain ? FieldConstants.RED_PRE_RIGHT_CLIMB_Y : FieldConstants.RED_PRE_LEFT_CLIMB_Y;

            finalX = isUpperChain ? FieldConstants.RED_RIGHT_CLIMB_X : FieldConstants.RED_LEFT_CLIMB_X;
            finalY = isUpperChain ? FieldConstants.RED_RIGHT_CLIMB_Y : FieldConstants.RED_LEFT_CLIMB_Y;
        } else {
            targetRot = isUpperChain ? Rotation2d.fromDegrees(FieldConstants.BLUE_LEFT_CLIMB_HEADING)
                    : Rotation2d.fromDegrees(FieldConstants.BLUE_RIGHT_CLIMB_HEADING);

            preAlignX = isUpperChain ? FieldConstants.BLUE_PRE_ALIGN_LEFT_X : FieldConstants.BLUE_PRE_ALIGN_RIGHT_X;
            preAlignY = isUpperChain ? FieldConstants.BLUE_PRE_ALIGN_LEFT_Y : FieldConstants.BLUE_PRE_ALIGN_RIGHT_Y;

            preX = isUpperChain ? FieldConstants.BLUE_PRE_LEFT_CLIMB_X : FieldConstants.BLUE_PRE_RIGHT_CLIMB_X;
            preY = isUpperChain ? FieldConstants.BLUE_PRE_LEFT_CLIMB_Y : FieldConstants.BLUE_PRE_RIGHT_CLIMB_Y;

            finalX = isUpperChain ? FieldConstants.BLUE_LEFT_CLIMB_X : FieldConstants.BLUE_RIGHT_CLIMB_X;
            finalY = isUpperChain ? FieldConstants.BLUE_LEFT_CLIMB_Y : FieldConstants.BLUE_RIGHT_CLIMB_Y;
        }

        this.prealigntargetPose = new Pose2d(preAlignX, preAlignY, targetRot);
        this.preTargetPose = new Pose2d(preX, preY, targetRot);
        this.finalTargetPose = new Pose2d(finalX, finalY, targetRot);
        this.currentState = AlignState.PRE_ALIGN;

        this.xController.reset();
        this.yController.reset();
        this.thetaController.reset();
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getStateCopy().Pose;

        Pose2d activeTarget;
        double maxSpeed;

        switch (currentState) {
            case PRE_ALIGN:
                activeTarget = prealigntargetPose;
                maxSpeed = 0.4;
                break;
            case PRE_CLIMB:
                activeTarget = preTargetPose;
                maxSpeed = 0.4;
                break;
            case FINAL_CLIMB:
            default:
                activeTarget = finalTargetPose;
                maxSpeed = 0.4;
                break;
        }

        double vx = xController.calculate(currentPose.getX(), activeTarget.getX());
        double vy = yController.calculate(currentPose.getY(), activeTarget.getY());
        double omega = thetaController.calculate(currentPose.getRotation().getRadians(),
                activeTarget.getRotation().getRadians());

        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        if (isRed) {
            vx = -vx;
            vy = -vy;
        }

        if (xController.atSetpoint())
            vx = 0;
        if (yController.atSetpoint())
            vy = 0;
        if (thetaController.atSetpoint())
            omega = 0;

        double maxRot = Math.PI;

        vx = MathUtil.clamp(vx, -maxSpeed, maxSpeed);
        vy = MathUtil.clamp(vy, -maxSpeed, maxSpeed);
        omega = MathUtil.clamp(omega, -maxRot, maxRot);

        // state machine transitions
        if (xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint()) {
            if (currentState == AlignState.PRE_ALIGN) {
                currentState = AlignState.PRE_CLIMB;
            } else if (currentState == AlignState.PRE_CLIMB) {
                currentState = AlignState.FINAL_CLIMB;
            } else if (currentState == AlignState.FINAL_CLIMB) {
                climb.climbHang();
            }
        }

        drivetrain.setControl(fieldCentricDrive
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(omega));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(fieldCentricDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
