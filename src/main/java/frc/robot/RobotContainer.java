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
// import frc.robot.commands.FuelChaseCommand;
import frc.robot.Constants.OIConstants;
// import frc.robot.Utils.Lookup;
// import frc.robot.commands.LockMode.lockState;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Indexer;
// import frc.robot.commands.AutoFire;
// import frc.robot.commands.LockMode;
import frc.robot.subsystems.Transfer;
// import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Hinge;

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

        private final VisionIOPhotonVision camera0 = new VisionIOPhotonVision(VisionConstants.camera0Name,
                        VisionConstants.robotToCamera0);
        private final VisionIOPhotonVision camera1 = new VisionIOPhotonVision(VisionConstants.camera1Name,
                        VisionConstants.robotToCamera1);

        public final Vision vision = new Vision(
                        (Vision.VisionConsumer) drivetrain::addVisionMeasurement,
                        (Supplier<Pose2d>) (() -> drivetrain.getStateCopy().Pose),
                        camera0, camera1);
//     public final Lookup lookup = Utils.createLookup(hood, shooter);
//     public final AutoFire autoFireCommand = new AutoFire(turret, transfer, shooter, hood, intake, lookup);
//     public final LockMode lockModeCommand = new LockMode(turret, shooter, hood);

//         public final Shooter shooter = new Shooter();
//         public final Intake intake = new Intake();
        public final Turret turret = new Turret();
//         public final Transfer transfer = new Transfer();
//         public final Hood hood = new Hood();
//         public final Climb climb = new Climb();
//         public final Indexer indexer = new Indexer();
//         public final Hinge hinge = new Hinge();

//         public final AutoFire autoFireCommand = new AutoFire(turret, transfer, shooter, hood);

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

        // CLIMB button controls
        // joystick.y().whileTrue(climb.climbUp());
        // joystick.x().whileTrueFalse(climb.climbHang());
        // joystick.a().whileTrue(climb.climbDown());

        // joystick.povRight().onTrue(climb.runClimbCommand());
        // joystick.povRight().onFalse(climb.stopCommand());
        

                // CLIMB button controls (disabled — climb subsystem currently not used)
                // joystick.povRight().whileTrue(climb.climbUp());
                // joystick.povLeft().whileTrue(climb.climbDown());

                joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
                joystick.b().whileTrue(drivetrain.applyRequest(
                                () -> point.withModuleDirection(
                                                new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // joystick.povUp().onFalse(Commands.runOnce(() -> hood.stopHoodCmd()));
        // joystick.povDown().onFalse(Commands.runOnce(() -> hood.stopHoodCmd()));

        // Hood controls are disabled while hood subsystem is out-of-scope
        // joystick.povUp().whileTrue(Commands.runOnce(() -> hood.hoodMoveTgt()));
        // joystick.povUp().onFalse(Commands.runOnce(() -> hood.stop()));

                // Seed field centric
                joystick.povUp().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                // Turret controls
                joystick.rightTrigger().whileTrue(turret.turretPos());
                joystick.leftTrigger().whileTrue(turret.turretNeg());
                joystick.leftTrigger().onFalse(Commands.runOnce(() -> turret.stop()));
                joystick.rightTrigger().onFalse(Commands.runOnce(() -> turret.stop()));

        // Turret tracking (camera-backed) — run on X button per request
        joystick.x().whileTrue(new TurretTracking(turret, camera0.getCamera(), VisionConstants.robotToCamera0,
                        drivetrain::getPose3d));
        joystick.x().onFalse(Commands.runOnce(() -> turret.stop(), turret));


        // SHOOTER button controls
        // joystick.leftBumper().onTrue(Commands.runOnce(() -> shooter.decreaseSpeed()));
        // joystick.rightBumper().onTrue(Commands.runOnce(() -> shooter.setVel()));
        // joystick.rightBumper().onFalse(Commands.runOnce(() -> shooter.stop()));

                // Intake controls
                // joystick.povRight().whileTrue(intake.intakeCommand()).onFalse(intake.stopRollerCommand());
                // joystick.povLeft().whileTrue(intake.outtakeCommand()).onFalse(intake.stopRollerCommand());

                // Turret tracking moved to X button to avoid duplicate bindings
                // joystick.y().onTrue(new TurretTracking(turret, camera0.getCamera(), VisionConstants.robotToCamera0,
                //                 drivetrain::getPose3d));

                // Transfer control
                // joystick.b().onTrue(Commands.runOnce(() -> transfer.toggleTransfer()));

                // Indexer control
                // Moved to right stick click or something else to avoid conflict with Transfer
                // (B) or TurretTracking (Y)?
                // Let's use right stick button
                // joystick.rightStick().onTrue(indexer.toggleIndexer());

                // Chase fuel ball with X button (PhotonVision feature)
                // joystick.x().whileTrue(new FuelChaseCommand(
                // 25, camera0.getCamera(), drivetrain,
                // drivetrain::getPose3d, VisionConstants.robotToCamera0));

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        public Command getAutonomousCommand() {
                return new TurretTracking(turret, camera0.getCamera(), VisionConstants.robotToCamera0,
                                drivetrain::getPose3d);
                // new FuelChaseCommand(
                // 25, camera0.getCamera(), drivetrain, // Change to the camera that will
                // // chase fuel once
                // // we get the fuel pose
                // // calculations working!!
                // drivetrain::getPose3d, VisionConstants.robotToCamera0),
                // new TurretTracking(turret, () -> drivetrain.getStateCopy().Pose));
        }
}
