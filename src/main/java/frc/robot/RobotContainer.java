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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Vision.Vision;
import frc.robot.Vision.VisionIOPhotonVision;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.Supplier;
import frc.robot.Vision.VisionConstants;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.TurretTracking;
import frc.robot.commands.FuelChaseCommand;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.commands.AutoFire;
import frc.robot.subsystems.Transfer;

public class RobotContainer {
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandXboxController joystick = new CommandXboxController(0);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        private final VisionIOPhotonVision cameraIO = new VisionIOPhotonVision(VisionConstants.camera0Name,
                        VisionConstants.robotToCamera0);

        public final Vision vision = new Vision(
                        (Vision.VisionConsumer) drivetrain::addVisionMeasurement,
                        (Supplier<Pose2d>) (() -> drivetrain.getStateCopy().Pose),
                        cameraIO);

        private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
        public final Intake intake = new Intake();
        public final Turret turret = new Turret();
        public final Transfer transfer = new Transfer();

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
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * 0.3) // Drive
                                                                                                                         // forward
                                                                                                                         // with
                                                                                                                         // negative
                                                                                                                         // Y
                                                                                                                         // (forward)
                                                .withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.3) // Drive left with
                                                                                                      // negative X
                                                                                                      // (left)
                                                .withRotationalRate(-joystick.getRightX() * MaxAngularRate * 0.3) // Drive
                                                                                                                  // counterclockwise
                                                                                                                  // with
                                                                                                                  // negative
                                                                                                                  // X
                                                                                                                  // (left)
                                ));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
                joystick.b().whileTrue(drivetrain.applyRequest(
                                () -> point.withModuleDirection(
                                                new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

                // joystick.x().onTrue(shooterSubsystem.shootCmd());
                joystick.povUp().onTrue(shooterSubsystem.runHoodCmd());
                joystick.povDown().onTrue(shooterSubsystem.runHoodReverseCmd());

                joystick.povUp().onFalse(shooterSubsystem.stopHoodCmd());
                joystick.povDown().onFalse(shooterSubsystem.stopHoodCmd());

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // reset the field-centric heading on left bumper press
                // joystick.leftBumper().onTrue(drivetrain.runOnce(() ->
                // drivetrain.seedFieldCentric()));

                joystick.rightTrigger().onTrue(Commands.runOnce(() -> turret.spinPositive()));
                joystick.leftTrigger().onTrue(Commands.runOnce(() -> turret.spinNegative()));
                joystick.leftTrigger().onFalse(Commands.runOnce(() -> turret.stop()));
                joystick.rightTrigger().onFalse(Commands.runOnce(() -> turret.stop()));

                joystick.leftBumper().onTrue(Commands.runOnce(() -> shooterSubsystem.decreaseSpeed()));
                joystick.rightBumper().onTrue(Commands.runOnce(() -> shooterSubsystem.increaseSpeed()));

                joystick.y().onTrue(new TurretTracking((turret)));

                joystick.b().onTrue(Commands.runOnce(() -> transfer.activateTransfer()));

                drivetrain.registerTelemetry(logger::telemeterize);

                // Chase fuel ball with X button
                joystick.x().whileTrue(new FuelChaseCommand(25, cameraIO.getCamera(), drivetrain,
                                () -> drivetrain.getStateCopy().Pose));

                // Follow a PathPlanner path with POV Left
                // Replace "examplePath" with your actual path file name

                // fix butten stuff *cough coguh* tyler change your button bindings

                // joystick.rightTrigger()
                // .whileTrue(
                // new InstantCommand(() -> intake.intakeCommand()))
                // .onFalse(
                // new InstantCommand(() -> intake.stopRollerCommand()));

                // joystick.leftTrigger()
                // .whileTrue(
                // new InstantCommand(() -> intake.outtakeCommand()))
                // .onFalse(
                // new InstantCommand(() -> intake.stopRollerCommand()));

                // joystick.rightBumper()
                // .whileTrue(
                // new InstantCommand(() -> intake.deployCommand()));

                // joystick.leftBumper()
                // .whileTrue(
                // new InstantCommand(() -> intake.retractCommand()));

                // joystick.rightTrigger()
                // .whileTrue(
                // new InstantCommand(() -> transfer.activateTransferCommand()))
                // .onFalse(
                // new InstantCommand(() -> transfer.stopTransferCommand()));

                // joystick.rightStick()
                // .whileTrue(
                // new InstantCommand(() -> autoFireCommand.execute()));
        }

        public Command getAutonomousCommand() {
                return new PathPlannerAuto("boi");
        }
}
