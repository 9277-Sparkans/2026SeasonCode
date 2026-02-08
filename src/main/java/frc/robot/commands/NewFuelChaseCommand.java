package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.FuelVision;
import org.littletonrobotics.junction.Logger;

/**
 * Command that uses the Jetson's game piece detection to automatically
 * drive toward and intake fuel/game pieces.
 * 
 * Designed to be used with whileTrue() - runs while button is held,
 * stops everything when released.
 */
public class NewFuelChaseCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Intake intake;
    private final FuelVision vision;
    private final PIDController turnController;

    // Swerve request for robot-relative driving (turn relative to camera)
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
            .withDeadband(0.05)
            .withRotationalDeadband(0.05)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // ============== TUNING CONSTANTS ==============
    // PID for turning toward the target
    private static final double kP_TURN = 0.03; // degrees -> rad/s (start conservative)
    private static final double kI_TURN = 0.0;
    private static final double kD_TURN = 0.002;

    // Speed limits
    private static final double MAX_TURN_SPEED = 1.5; // rad/s - safety cap
    private static final double FORWARD_SPEED = 1.0; // m/s - approach speed

    // Area threshold to stop driving (game piece is very close/in intake)
    private static final double TARGET_AREA_THRESHOLD = 50000.0; // pixels

    // Tolerance for "centered" on target
    private static final double TARGET_X_TOLERANCE = 2.0; // degrees

    public NewFuelChaseCommand(
            CommandSwerveDrivetrain drivetrain,
            Intake intake,
            FuelVision vision) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.vision = vision;

        this.turnController = new PIDController(kP_TURN, kI_TURN, kD_TURN);
        this.turnController.setSetpoint(0); // We want targetX = 0 (centered)
        this.turnController.setTolerance(TARGET_X_TOLERANCE);

        // Require drivetrain and intake so no other command fights us
        addRequirements(drivetrain, intake);
    }

    @Override
    public void initialize() {
        turnController.reset();
        Logger.recordOutput("GamePieceChase/Active", true);
    }

    @Override
    public void execute() {
        // 1. Always run intake while this command is active
        intake.intake();

        // 2. Vision-based driving
        if (vision.hasTarget()) {
            double targetX = vision.getTargetX();
            double targetArea = vision.getTargetArea();

            // Calculate turn output (targetX in degrees, output in rad/s)
            double rotationOutput = turnController.calculate(targetX);

            // Clamp rotation for safety
            rotationOutput = Math.max(-MAX_TURN_SPEED, Math.min(MAX_TURN_SPEED, rotationOutput));

            // Determine forward speed based on target area
            double forwardSpeed;
            if (targetArea > TARGET_AREA_THRESHOLD) {
                forwardSpeed = 0.0; // Very close, just intake
            } else if (targetArea > TARGET_AREA_THRESHOLD * 0.5) {
                forwardSpeed = FORWARD_SPEED * 0.5; // Getting close, slow down
            } else {
                forwardSpeed = FORWARD_SPEED; // Full speed approach
            }

            // Log for debugging
            Logger.recordOutput("GamePieceChase/TargetX", targetX);
            Logger.recordOutput("GamePieceChase/TargetArea", targetArea);
            Logger.recordOutput("GamePieceChase/RotationOutput", rotationOutput);
            Logger.recordOutput("GamePieceChase/ForwardSpeed", forwardSpeed);
            Logger.recordOutput("GamePieceChase/HasTarget", true);

            // Drive robot-relative (forward is toward camera target)
            drivetrain.setControl(
                    driveRequest
                            .withVelocityX(forwardSpeed)
                            .withVelocityY(0)
                            .withRotationalRate(rotationOutput));

        } else {
            // No target seen - stop driving but keep intaking
            Logger.recordOutput("GamePieceChase/HasTarget", false);
            Logger.recordOutput("GamePieceChase/RotationOutput", 0.0);
            Logger.recordOutput("GamePieceChase/ForwardSpeed", 0.0);

            drivetrain.setControl(
                    driveRequest
                            .withVelocityX(0)
                            .withVelocityY(0)
                            .withRotationalRate(0));
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop everything when command ends (button released)
        drivetrain.setControl(
                driveRequest
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0));
        intake.stopRoller();

        Logger.recordOutput("GamePieceChase/Active", false);
    }

    @Override
    public boolean isFinished() {
        // Never auto-finish - run while button is held
        return false;
    }
}
