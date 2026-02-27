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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Vision.Vision;
import frc.robot.Vision.VisionIOPhotonVision;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.QuickAccessConstants;
import frc.robot.Constants.QuickAccessConstants.ControlTypes;
import java.util.function.Supplier;
import frc.robot.Vision.VisionConstants;

import frc.robot.commands.TurretTracking;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Hinge;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.LockMode;

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
        private final SendableChooser<Command> autoChooser = new SendableChooser<>();

        public final CommandXboxController joystick = new CommandXboxController(OIConstants.kOperatorControllerPort);
        public final CommandJoystick translateStick = new CommandJoystick(OIConstants.kDriverTranslateStickPort);
        public final CommandJoystick rotateStick = new CommandJoystick(OIConstants.kDriverRotateStickPort);

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
        private final VisionIOPhotonVision camera2 = edu.wpi.first.wpilibj.RobotBase.isSimulation()
                        ? new frc.robot.Vision.VisionIOPhotonVisionSim(VisionConstants.camera2Name,
                                        VisionConstants.robotToCamera2)
                        : new VisionIOPhotonVision(VisionConstants.camera2Name,
                                        VisionConstants.robotToCamera2);

        public final Vision vision = new Vision(
                        (Vision.VisionConsumer) drivetrain::addVisionMeasurement,
                        (Supplier<Pose2d>) (() -> drivetrain.getStateCopy().Pose),
                        camera0, camera1, camera2);
        public final Turret turret = new Turret();
        public final Climb climb = new Climb();
        public final Shooter shooter = new Shooter();
        public final Hood hood = new Hood();
        public final Transfer transfer = new Transfer();
        public final Indexer indexer = new Indexer();
        public final Intake intake = new Intake();
        public final Hinge hinge = new Hinge();

        public RobotContainer() {
                NamedCommands.registerCommand("testNamedCommand",
                                Commands.runOnce(() -> System.out.println("this named command works")));

                // Auto chooser
                autoChooser.setDefaultOption("Do Nothing", Commands.none());
                autoChooser.addOption("S1 DPR + Climb (S1.1-S-DPR-C)", AutoAlignCommand.getS1DPR_C(drivetrain));
                autoChooser.addOption("S1 DPR (S1.1-S-DPR)", AutoAlignCommand.getS1DPR(drivetrain));
                autoChooser.addOption("S2 Depost (S2.DP)", AutoAlignCommand.getS2DP(drivetrain));
                autoChooser.addOption("S2 Human Player (S2.HP)", AutoAlignCommand.getS2HP(drivetrain));
                autoChooser.addOption("S3 Human Player (S3.HP)", AutoAlignCommand.getS3HP(drivetrain));
                SmartDashboard.putData("Auto Chooser", autoChooser);

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

                // turret tracking toggled on x button
                joystick.x().toggleOnTrue(
                                new TurretTracking(turret, () -> drivetrain.getStateCopy().Pose));

                // auto align while holding y button
                joystick.y().whileTrue(AutoAlignCommand.getAutoAlignCommand(drivetrain));

                // run proper trench path on right bumper based on current pose
                // joystick buttons for sticks mode
                translateStick.button(4).whileTrue(AutoAlignCommand.getAutoAlignCommand(drivetrain));
                translateStick.button(2).onTrue(shooter.shooterStop());

                // Climb up, autoalign, climb hang, lock mode center and shoot
                rotateStick.button(4).onTrue(
                                Commands.sequence(
                                                climb.climbUp(),
                                                AutoAlignCommand.getAutoAlignCommand(drivetrain),
                                                Commands.waitSeconds(1.5), // Tune: time for hang arm to settle before
                                                                           // hanging
                                                climb.climbDown()));

                rotateStick.button(1).onTrue(climb.climbUp());
                rotateStick.button(2).onTrue(climb.climbDown());

                rotateStick.button(3).toggleOnTrue(new TurretTracking(turret, () -> drivetrain.getStateCopy().Pose));

                // driver sticks support
                drivetrain.setDefaultCommand(
                                drivetrain.applyRequest(() -> {
                                        double x, y, rot;
                                        if (QuickAccessConstants.controlType == ControlTypes.DRIVER_STICKS) {
                                                x = -translateStick.getRawAxis(1);

                                                y = -translateStick.getRawAxis(0);

                                                rot = -rotateStick.getRawAxis(0);
                                        } else {
                                                x = -joystick.getLeftY();
                                                y = -joystick.getLeftX();
                                                rot = -joystick.getRightX();
                                        }

                                        return drive.withVelocityX(x * MaxSpeed * 0.3)
                                                        .withVelocityY(y * MaxSpeed * 0.3)
                                                        .withRotationalRate(rot * MaxAngularRate * 0.3);
                                }));

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}
