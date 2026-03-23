// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Vision.Vision;
import frc.robot.Vision.VisionConstants;
import frc.robot.Vision.VisionIOPhotonVision;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.Supplier;

import frc.robot.subsystems.Turret;
import frc.robot.util.AllianceShifts;
import frc.robot.util.FuelSim;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.QuickAccessConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.QuickAccessConstants.ControlTypes;
import frc.robot.Utils.Lookup;
import frc.robot.commands.AutoFire;
import frc.robot.commands.LockMode;
import frc.robot.commands.TurretTracking;
import frc.robot.commands.AutoFire.TargetHub;
import frc.robot.commands.LockMode.LockState;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Hinge;
import frc.robot.Vision.Vision;
import frc.robot.Vision.VisionIOPhotonVision;
import edu.wpi.first.math.geometry.Pose2d;
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
        private double driveTrainVelocityPercent = 0.55;
        private boolean movingConstantSpeed = false;

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);
        private final SendableChooser<Command> autoChooser = new SendableChooser<>();

        public final CommandXboxController joystick = new CommandXboxController(OIConstants.kDriverControllerPort);
        public final CommandJoystick translateStick = new CommandJoystick(OIConstants.kDriverTranslateStickPort);
        public final CommandJoystick rotateStick = new CommandJoystick(OIConstants.kDriverRotateStickPort);
        public final Joystick operatorJoystick = new Joystick(OIConstants.kOperatorControllerPort);

        private static final boolean OPERATOR_JOYSTICK_DEBUG = false;

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

        public final Turret turret = new Turret();
        public final Climb climb = new Climb();
        public final Shooter shooter = new Shooter();
        public final Hood hood = new Hood();
        public final Intake intake = new Intake();
        public final Transfer transfer = new Transfer();
        public final Indexer indexer = new Indexer();
        public final Hinge hinge = new Hinge();
        public final AutoFire autoFireCommand;
        // public final LockMode lockModeCommand = new LockMode(turret, shooter, hood);

        public boolean manualControl = false;
        public boolean lockmode = false;

        public final Timer teleopTimer = new Timer();

        public final Vision vision = new Vision(
                        (Vision.VisionConsumer) drivetrain::addVisionMeasurement,
                        (Supplier<Pose2d>) (() -> drivetrain.getStateCopy().Pose),
                        camera0, camera1, camera2);

        public final Lookup lookup = Utils.createLookup();
        private FuelSim fuelSim;

        public RobotContainer() {
                if (RobotBase.isSimulation()) {
                        configureFuelSim();
                }
                autoFireCommand = new AutoFire(indexer, turret, shooter, hood,
                                                () -> drivetrain.getStateCopy().Speeds,
                                                () -> drivetrain.getStateCopy().Pose, lookup);

                if (RobotBase.isSimulation()) {
                        autoFireCommand.setFuelSim(fuelSim);
                }

                NamedCommands.registerCommand("intake",
                                intake.intakeCommand());
                // NamedCommands.registerCommand("climbUp",
                // climb.climbUp());
                // NamedCommands.registerCommand("climbHang",
                // climb.climbHang());
                NamedCommands.registerCommand("climbUp",
                                Commands.runOnce(() -> climb.climbUp()));
                NamedCommands.registerCommand("climbHang",
                                Commands.runOnce(() -> climb.climbHang()));
                NamedCommands.registerCommand("autofire",
                                autoFireCommand);
                NamedCommands.registerCommand("hingeDown",
                                hinge.hingeDown());
                NamedCommands.registerCommand("hingeUp",
                                hinge.hingeUp());
                NamedCommands.registerCommand("index",
                                Commands.runOnce(() -> indexer.setVel()));
                NamedCommands.registerCommand("kicker",
                                Commands.runOnce(() -> transfer.toggleTransfer()));

                turret.setDefaultCommand(turret.initDefaultCommand(turret));
                hood.setDefaultCommand(hood.initDefaultCommand());
                hinge.setDefaultCommand(hinge.initDefaultCommand(hinge));

                // auto chooser
                autoChooser.setDefaultOption("No auto", Commands.none());
                // jeet autos, add final autos in later
                // autoChooser.addOption("S1 DPR + Climb (S1.1-S-DPR-C)",
                // AutoAlignCommand.getS1DPR_C(drivetrain));
                // autoChooser.addOption("S1 DPR (S1.1-S-DPR)",
                // AutoAlignCommand.getS1DPR(drivetrain));
                // autoChooser.addOption("S2 Depost (S2.DP)",
                // AutoAlignCommand.getS2DP(drivetrain));
                // autoChooser.addOption("S2 Human Player (S2.HP)",
                // AutoAlignCommand.getS2HP(drivetrain));
                // autoChooser.addOption("S3 Human Player (S3.HP)",
                // AutoAlignCommand.getS3HP(drivetrain));

                SmartDashboard.putData("Auto Chooser", autoChooser);

                SmartDashboard.putData("Git Info", new Sendable() {
                        @Override
                        public void initSendable(SendableBuilder builder) {
                                builder.addStringProperty("Branch", () -> BuildConstants.GIT_BRANCH, null);
                                builder.addStringProperty("Commit", () -> BuildConstants.GIT_SHA, null);
                                builder.addStringProperty("Date of commit", () -> BuildConstants.GIT_DATE, null);
                                builder.addStringProperty("Uncommitted changes",
                                                () -> Boolean.valueOf(BuildConstants.DIRTY > 0).toString(),
                                                null);
                        }
                });

                configureBindings();
        }

        public void startTeleop() {
                teleopTimer.start();
        }

        public void teleopPeriodic() {
                if (teleopTimer.get() >= 140)
                        teleopTimer.stop();

                SmartDashboard.putNumber("Shifts/Match Time", teleopTimer.get());
                SmartDashboard.putString(
                                "Shifts/Remaining Shift Time",
                                String.format("%.1f",
                                                Math.max(AllianceShifts.getRemainingShiftTime(teleopTimer), 0.0)));
                SmartDashboard.putBoolean("Shifts/Shift Active", AllianceShifts.areWeActive(teleopTimer));
                SmartDashboard.putString(
                                "Shifts/Game State", AllianceShifts.getCurrentShift(teleopTimer).toString());
                // SmartDashboard.putNumber("Bot velocity magnitude", drivetrain.speeds.get);
        }

        private void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                drivetrain.applyRequest(() -> {
                                        double x, y, rot;
                                        if (movingConstantSpeed && QuickAccessConstants.controlType == ControlTypes.DRIVER_STICKS)
                                        {
                                                double maxVal = Math.max(Math.abs(-translateStick.getRawAxis(1)), Math.abs(-translateStick.getRawAxis(0)));
                                                x = (-translateStick.getRawAxis(1) / maxVal) * ShooterConstants.autoFireDriveSpeed;
                                                y = (-translateStick.getRawAxis(0) / maxVal) * ShooterConstants.autoFireDriveSpeed;
                                                rot = -rotateStick.getRawAxis(0);
                                        } 
                                        else if (QuickAccessConstants.controlType == ControlTypes.DRIVER_STICKS) {
                                                x = -translateStick.getRawAxis(1);
                                                y = -translateStick.getRawAxis(0);
                                                rot = -rotateStick.getRawAxis(0);
                                        }
                                        else 
                                        {
                                                x = -joystick.getLeftY();
                                                y = -joystick.getLeftX();
                                                rot = -joystick.getRightX();
                                        }
                                        return drive.withVelocityX(x * MaxSpeed * driveTrainVelocityPercent)
                                                        .withVelocityY(y * MaxSpeed * driveTrainVelocityPercent)
                                                        .withRotationalRate(rot * MaxAngularRate
                                                                        * driveTrainVelocityPercent);
                                }));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                // joystick.start().and(joystick.povUp()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                // joystick.start().and(joystick.povDown()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                // joystick.start().and(joystick.povRight()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                // joystick.start().and(joystick.povLeft()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // reset the field-centric heading on left bumper press
                joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
                // joystick.().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                joystick.start().and(joystick.povUp()).whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                joystick.start().and(joystick.povDown()).whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
                joystick.start().and(joystick.povRight()).whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
                joystick.start().and(joystick.povLeft()).whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));

                // CLIMB button controls
                // joystick.y().onTrue(climb.climbUp());
                // joystick.y().onFalse(climb.stopCommand());
                // joystick.x().onTrue(climb.climbHang());
                // joystick.x().onFalse(climb.stopCommand());
                // joystick.a().onTrue(climb.climbDown());
                // joystick.a().onFalse(climb.stopCommand());

                // joystick.a().onTrue(climb.runClimbCommand());
                // joystick.a().onFalse(climb.stopCommand());

                // joystick.y().onTrue(climb.runClimbNegCommand());
                // joystick.y().onFalse(climb.stopCommand());

                // HOOD button controls
                // joystick.povUp().whileTrue(Commands.runOnce(() -> hood.moveHoodToAngle(5)));
                // joystick.povUp().whileTrue(Commands.runOnce(() ->
                // hood.moveHoodToAngle(hood.targetHoodAngle)));
                // joystick.povUp().onTrue(Commands.runOnce(() ->
                // hood.moveHoodToAngle(Angle.ofBaseUnits(hood.targetHoodAngle, Degree))));

                // joystick.povUp().onTrue(Commands.runOnce(() -> hood.runHood()));
                // joystick.povDown().onTrue(Commands.runOnce(() -> hood.runHoodReverse()));
                // joystick.povUp().onFalse(Commands.runOnce(() -> hood.stopHoodCmd()));
                // joystick.povDown().onFalse(Commands.runOnce(() -> hood.stopHoodCmd()));

                // TURRET button controls
                // joystick.rightTrigger().whileTrue(turret.turretPos());
                // joystick.leftTrigger().whileTrue(turret.turretNeg());
                // joystick.leftTrigger().onFalse(Commands.runOnce(() -> turret.stop()));
                // joystick.rightTrigger().onFalse(Commands.runOnce(() -> turret.stop()));

                // joystick.x().whileTrue(new TurretTracking((turret)));
                // joystick.x().onFalse(Commands.runOnce(() -> turret.stop(), turret));

                // joystick.x().whileTrue(
                // Commands.runOnce(() -> lockModeCommand.setLockState(LockState.TRENCHLEFT),
                // turret, shooter, hood));

                // SHOOTER button controls
                // joystick.leftBumper().onTrue(Commands.runOnce(() ->
                // shooter.decreaseSpeed()));
                // joystick.rightBumper().onTrue(Commands.runOnce(() ->
                // shooter.increaseSpeed()));

                // joystick.x().onTrue(shooter.shootCmd());
                // joystick.x().whileTrue(Commands.runOnce(() -> autoFireCommand.execute()));

                // TRANSFER button controls
                joystick.b().onTrue(Commands.runOnce(() -> transfer.toggleTransfer()));

                // INTAKE button controls
                joystick.y()
                                .whileTrue(
                                                intake.intakeCommand())
                                .onFalse(
                                                intake.stopRollerCommand());

                joystick.a()
                                .whileTrue(
                                                intake.outtakeCommand())
                                .onFalse(
                                                intake.stopRollerCommand());

                // joystick.y().whileTrue(AutoAlignCommand.getTrenchCommand(drivetrain));

                // HINGE button controls
                // joystick.povRight()
                // .onTrue(hinge.hingeUp())
                // .onFalse(hinge.hingeStopCommand());

                // joystick.povLeft()
                // .onTrue(hinge.hingeDown())
                // .onFalse(hinge.hingeStopCommand());

                // Indexer button controls
                // joystick.b().onTrue(indexer.toggleIndexer());
                configureOperatorConsole();

                drivetrain.registerTelemetry(logger::telemeterize);

                // for flight sticks controls, go to this https://gpadtester.com/ and put the
                // button id +1 (so button 1 would actually be button 2 on here)
                // rotateStick.button(3).whileTrue(AutoAlignCommand.getAutoAlignCommand(drivetrain));

                // ---------- FLIGHT STICK DRIVER PREFERENCE (DO NOT CHANGE) ---------- //

                // intake
                rotateStick.button(OIConstants.kSticks_trigger).onTrue(intake.intakeCommand());
                rotateStick.button(OIConstants.kSticks_trigger).onFalse(intake.stopRollerCommand());

                // outtake
                rotateStick.button(OIConstants.kSticks_centerHandle).onTrue(intake.outtakeCommand());
                rotateStick.button(OIConstants.kSticks_centerHandle).onFalse(intake.stopRollerCommand());

                // climb autoalign
                rotateStick.button(OIConstants.kSticks_leftHandle)
                                .whileTrue(AutoAlignCommand.getAutoClimbCommand(drivetrain));

                // boost
                rotateStick.button(OIConstants.kSticks_rightHandle)
                                .whileTrue(Commands.runOnce(() -> driveTrainVelocityPercent = 1.0));
                rotateStick.button(OIConstants.kSticks_rightHandle)
                                .onFalse(Commands.runOnce(() -> driveTrainVelocityPercent = 0.55));

                // shooter up down
                // rotateStick.button(OIConstants.povUp).onTrue(Commands.either(
                // Commands.runOnce(() -> shooter.increaseSpeed()),
                // Commands.runOnce(() -> {}),
                // () -> manualControl));
                // rotateStick.button(OIConstants.povDown).onTrue(Commands.either(
                // Commands.runOnce(() -> shooter.decreaseSpeed()),
                // Commands.runOnce(() -> {}),
                // () -> manualControl));

                // turret left right
                // rotateStick.button(OIConstants.povRight).onTrue(Commands.either(
                // turret.turretPos(),
                // Commands.runOnce(() -> {}),
                // () -> manualControl));
                // rotateStick.button(OIConstants.povLeftt).onTrue(Commands.either(
                // turret.turretNeg(),
                // Commands.runOnce(() -> {}),
                // () -> manualControl));

                // GYRO RESET
                translateStick.button(OIConstants.kRightSticks_leftGrid_bottomLeft)
                                .toggleOnTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
                translateStick.button(OIConstants.kRightSticks_leftGrid_bottomLeft)
                                .toggleOnTrue(Commands.runOnce(() -> vision.enableVision = false));

                // autofire
                // translateStick.button(OIConstants.kSticks_trigger).whileTrue(autoFireCommand);

                translateStick.button(OIConstants.kSticks_trigger).whileTrue(Commands.either(
                                autoFireCommand,
                                Commands.runOnce(() -> indexer.setVel()),
                                () -> !lockmode));
                translateStick.button(OIConstants.kSticks_trigger).onFalse(indexer.indexerStop());

                //kicker
                translateStick.button(OIConstants.kSticks_centerHandle).whileTrue(Commands.runOnce(() -> transfer.activateTransfer()));
                translateStick.button(OIConstants.kSticks_centerHandle).onFalse(Commands.runOnce(() -> transfer.stop()));
                
                translateStick.button(OIConstants.kSticks_leftHandle).whileTrue(Commands.runOnce(() -> indexer.setVel())).onFalse(indexer.indexerStop());

                //outpost autoalign
                // translateStick.button(OIConstants.kSticks_leftHandle).whileTrue(AutoAlignCommand.getOutpostCommand(drivetrain));

                // trench autoalign
                translateStick.button(OIConstants.kSticks_rightHandle)
                                .whileTrue(AutoAlignCommand.getTrenchCommand(drivetrain));

                // spindexer
                translateStick.button(OIConstants.kLeftSticks_leftGrid_bottomLeft)
                                .whileTrue(Commands.runOnce(() -> indexer.setVel()));

        }

        private void configureOperatorConsole() {
                // Operator
                // operator(1)
                //                 .onTrue(Commands.runOnce(() -> hood.moveHoodToAngle(hood.targetHoodAngle)));
                operator(OIConstants.kKeyboard_duck)
                        .onTrue(Commands.runOnce(() -> movingConstantSpeed = true))
                        .onFalse(Commands.runOnce(() -> movingConstantSpeed = false));
                operator(OIConstants.kKeyboard_modeToggle)
                                .onTrue(Commands.runOnce(() -> manualControl = !manualControl));

                operator(OIConstants.kKeyboard_trackToggle)
                                .onTrue(Commands.runOnce(() -> lockmode = !lockmode));

                operator(OIConstants.kKeyboard_lockModeToggle)
                                .whileTrue(new LockMode(turret, shooter, hood, LockState.LOCK));

                operator(OIConstants.kKeyboard_lockModeLeft)
                                .whileTrue(new LockMode(turret, shooter, hood, LockState.LEFT));

                operator(OIConstants.kKeyboard_lockModeCenter)
                                .whileTrue(new LockMode(turret, shooter, hood, LockState.CENTER));

                operator(OIConstants.kKeyboard_lockModeRight)
                                .whileTrue(new LockMode(turret, shooter, hood, LockState.RIGHT));

                operator(OIConstants.kKeyboard_lockModeTrenchLeft)
                                .whileTrue(new LockMode(turret, shooter, hood, LockState.TRENCHLEFT));

                operator(OIConstants.kKeyboard_lockModeTrenchRight)
                                .whileTrue(new LockMode(turret, shooter, hood, LockState.TRENCHRIGHT));

                operator(OIConstants.kKeyboard_climbUp)
                                .onTrue(climb.climbUp());

                operator(OIConstants.kKeyboard_climbHang)
                                .onTrue(climb.climbHang());

                operator(OIConstants.kKeyboard_climbDown)
                                .onTrue(climb.climbDown());

                operator(OIConstants.kKeyboard_intakeDeploy)
                                .onTrue(hinge.hingeDown())
                                .onFalse(hinge.hingeStopCommand());

                operator(OIConstants.kKeyboard_intakeRetract)
                                .onTrue(hinge.hingeUp())
                                .onFalse(hinge.hingeStopCommand());
        }

        public JoystickButton operator(int keyCode) {
                if (OPERATOR_JOYSTICK_DEBUG) {
                        new JoystickButton(operatorJoystick, keyCode)
                                        .onTrue(Commands.runOnce(() -> System.out
                                                        .println("Pressed operator keycode " + keyCode)));
                }
                return new JoystickButton(operatorJoystick, keyCode);
        }

        public Command getAutonomousCommand() {
                // return autoChooser.getSelected();
                return new PathPlannerAuto("tuffauto");
        }

        private void configureFuelSim() {
                fuelSim = new FuelSim();
                // Register robot with its dimensions (from Constants)
                fuelSim.registerRobot(
                                Constants.RobotDimensions.FULL_WIDTH,
                                Constants.RobotDimensions.FULL_LENGTH,
                                Constants.RobotDimensions.BUMPER_HEIGHT,
                                () -> drivetrain.getStateCopy().Pose,
                                () -> drivetrain.getStateCopy().Speeds // Field-relative speeds
                );

                // Register intake region - DISABLED FOR UNLIMITED FUEL TESTING
                fuelSim.registerIntake(
                                0.3, 0.7, -0.4, 0.4, // bounding box relative to robot center
                                () -> intake.isIntaking() && intake.canIntake(), // active when intaking and NOT full
                                () -> intake.addFuel()); // increment fuel count when a ball is picked up

                fuelSim.setLoggingFrequency(100.0);
                fuelSim.spawnStartingFuel();
                fuelSim.start();
        }

        public void updateSim() {
                if (fuelSim != null) {
                        fuelSim.updateSim();
                }
        }
}
