// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Vision.Vision;
import frc.robot.Vision.VisionIOPhotonVision;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.Supplier;
import frc.robot.Vision.VisionConstants;

import frc.robot.commands.TurretTracking;
import frc.robot.subsystems.Turret;
import frc.robot.commands.AutoAlignCommand;

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

        public final CommandXboxController joystick = new CommandXboxController(0);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        private final VisionIOPhotonVision camera0 = edu.wpi.first.wpilibj.RobotBase.isSimulation()
                        ? new frc.robot.Vision.VisionIOPhotonVisionSim(VisionConstants.camera0Name,
                                        VisionConstants.robotToCamera0)
                        : new VisionIOPhotonVision(VisionConstants.camera0Name,
                                        VisionConstants.robotToCamera0);
        private final VisionIOPhotonVision camera1 = edu.wpi.first.wpilibj.RobotBase.isSimulation()
                        ? new frc.robot.Vision.VisionIOPhotonVisionSim(VisionConstants.camera1Name,
                                        VisionConstants.robotToCamera1)
                        : new VisionIOPhotonVision(VisionConstants.camera1Name,
                                        VisionConstants.robotToCamera1);

        public final Vision vision = new Vision(
                        (Vision.VisionConsumer) drivetrain::addVisionMeasurement,
                        (Supplier<Pose2d>) (() -> drivetrain.getStateCopy().Pose),
                        camera0, camera1);
        public final Turret turret = new Turret(() -> drivetrain.getStateCopy().Pose,
                        () -> drivetrain.getStateCopy().Speeds);

        public RobotContainer() {
                NamedCommands.registerCommand("testNamedCommand",
                                Commands.runOnce(() -> System.out.println("this named command works")));

                SmartDashboard.putData("Git Info", new Sendable() {
                        @Override
                        public void initSendable(SendableBuilder builder) {
                                builder.addStringProperty("Branch", () -> BuildConstants.GIT_BRANCH, null);
                                builder.addStringProperty("Commit", () -> BuildConstants.GIT_SHA, null);
                                builder.addStringProperty("Date of commit", () -> BuildConstants.GIT_DATE, null);
                                builder.addStringProperty("Uncommitted changes",
                                                () -> new Boolean(BuildConstants.DIRTY > 0).toString(), null);
                        }
                });

                configureBindings();
        }

        private void configureBindings() {

                // reset the field-centric heading on left bumper press
                joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
                joystick.b().whileTrue(drivetrain.applyRequest(
                                () -> point.withModuleDirection(
                                                new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

                // Seed field centric
                joystick.povUp().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                // Turret tracking (camera-backed) — run on X button per request
                joystick.x().toggleOnTrue(
                                new TurretTracking(turret));

                joystick.y().whileTrue(AutoAlignCommand.getAutoAlignCommand(drivetrain));

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

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        public Command getAutonomousCommand() {
                return new TurretTracking(turret);
                // new FuelChaseCommand(
                // 25, camera0.getCamera(), drivetrain, // Change to the camera that will
                // // chase fuel once
                // // we get the fuel pose
                // // calculations working!!
                // drivetrain::getPose3d, VisionConstants.robotToCamera0),
                // new TurretTracking(turret, () -> drivetrain.getStateCopy().Pose));
        }
}
