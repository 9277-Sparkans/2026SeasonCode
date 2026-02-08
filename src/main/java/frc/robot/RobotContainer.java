// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Vision.Vision;
import frc.robot.Vision.VisionIOPhotonVision;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.Supplier;
import frc.robot.Vision.VisionConstants;

import frc.robot.commands.TurretTracking;
import frc.robot.commands.NewFuelChaseCommand;
import frc.robot.commands.AutoFire;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.FuelVision;

public class RobotContainer {
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandXboxController joystick = new CommandXboxController(0);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        private final VisionIOPhotonVision camera0 = new VisionIOPhotonVision(VisionConstants.camera0Name,
                        VisionConstants.robotToCamera0);
        private final VisionIOPhotonVision camera1 = new VisionIOPhotonVision(VisionConstants.camera1Name,
                        VisionConstants.robotToCamera1);
        // private final VisionIOPhotonVision camera2 = new
        // VisionIOPhotonVision(VisionConstants.camera2Name,
        // VisionConstants.robotToCamera2);
        // private final VisionIOPhotonVision camera3 = new
        // VisionIOPhotonVision(VisionConstants.camera3Name,
        // VisionConstants.robotToCamera3);

        public final Vision vision = new Vision(
                        (Vision.VisionConsumer) drivetrain::addVisionMeasurement,
                        (Supplier<Pose2d>) (() -> drivetrain.getStateCopy().Pose),
                        camera0, camera1);

        private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
        public final Intake intake = new Intake();
        public final Turret turret = new Turret();
        public final Transfer transfer = new Transfer();
        public final FuelVision gamePieceVision = new FuelVision();

        public final AutoFire autoFireCommand = new AutoFire(turret, transfer, shooterSubsystem);

        public RobotContainer() {
                NamedCommands.registerCommand("testNamedCommand",
                                Commands.runOnce(() -> System.out.println("this named command works")));

                configureBindings();
        }

        private void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * 0.3)
                                                .withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.3)
                                                .withRotationalRate(-joystick.getRightX() * MaxAngularRate * 0.3)));

                // Idle while the robot is disabled
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
                joystick.b().whileTrue(drivetrain.applyRequest(
                                () -> point.withModuleDirection(
                                                new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

                joystick.povUp().onTrue(shooterSubsystem.runHoodCmd());
                joystick.povDown().onTrue(shooterSubsystem.runHoodReverseCmd());
                joystick.povUp().onFalse(shooterSubsystem.stopHoodCmd());
                joystick.povDown().onFalse(shooterSubsystem.stopHoodCmd());

                // Run SysId routines when holding back/start and X/Y
                joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // Turret controls
                joystick.rightTrigger().onTrue(Commands.runOnce(() -> turret.spinPositive()));
                joystick.leftTrigger().onTrue(Commands.runOnce(() -> turret.spinNegative()));
                joystick.leftTrigger().onFalse(Commands.runOnce(() -> turret.stop()));
                joystick.rightTrigger().onFalse(Commands.runOnce(() -> turret.stop()));

                // Shooter speed controls
                joystick.leftBumper().onTrue(Commands.runOnce(() -> shooterSubsystem.decreaseSpeed()));
                joystick.rightBumper().onTrue(Commands.runOnce(() -> shooterSubsystem.increaseSpeed()));

                // Turret tracking
                joystick.y().onTrue(new TurretTracking(turret, () -> drivetrain.getStateCopy().Pose));

                // Transfer
                joystick.b().onTrue(Commands.runOnce(() -> transfer.activateTransfer()));

                drivetrain.registerTelemetry(logger::telemeterize);

                // ====== GAME PIECE CHASE (X button) ======
                // Uses Jetson YOLO detection to chase and intake fuel
                // Hold X button to run - drives toward detected game piece and runs intake
                joystick.x().whileTrue(new NewFuelChaseCommand(drivetrain, intake, gamePieceVision));
        }

        public Command getAutonomousCommand() {
                // TODO: Replace with actual auto routine
                return Commands.print("No auto configured");
        }
        }
}
