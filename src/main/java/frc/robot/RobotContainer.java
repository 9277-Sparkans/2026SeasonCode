// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.QuickAccessConstants;
import frc.robot.Constants.QuickAccessConstants.ControlTypes;
import frc.robot.Utils.Lookup;
import frc.robot.commands.AutoFire;
import frc.robot.commands.LockMode;
import frc.robot.commands.TurretTracking;
import frc.robot.commands.LockMode.LockState;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Hinge;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandXboxController joystick = new CommandXboxController(OIConstants.kDriverControllerPort);
    public final CommandJoystick translateStick = new CommandJoystick(OIConstants.kDriverTranslateStickPort);
    public final CommandJoystick rotateStick = new CommandJoystick(OIConstants.kDriverRotateStickPort);
    public final Joystick operatorJoystick = new Joystick(OIConstants.kOperatorControllerPort);

    private static final boolean OPERATOR_JOYSTICK_DEBUG = false;

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final VisionIOPhotonVision camera0 = edu.wpi.first.wpilibj.RobotBase.isSimulation()
            ? new frc.robot.Vision.VisionIOPhotonVisionSim(VisionConstants.camera0Name, VisionConstants.robotToCamera0)
            : new VisionIOPhotonVision(VisionConstants.camera0Name, VisionConstants.robotToCamera0);
    private final VisionIOPhotonVision camera1 = edu.wpi.first.wpilibj.RobotBase.isSimulation()
            ? new frc.robot.Vision.VisionIOPhotonVisionSim(VisionConstants.camera1Name, VisionConstants.robotToCamera1)
            : new VisionIOPhotonVision(VisionConstants.camera1Name, VisionConstants.robotToCamera1);
    private final VisionIOPhotonVision camera2 = edu.wpi.first.wpilibj.RobotBase.isSimulation()
            ? new frc.robot.Vision.VisionIOPhotonVisionSim(VisionConstants.camera2Name, VisionConstants.robotToCamera2)
            : new VisionIOPhotonVision(VisionConstants.camera2Name, VisionConstants.robotToCamera2);

    // public final Vision vision = new Vision(
    //         (Vision.VisionConsumer) drivetrain::addVisionMeasurement,
    //         (Supplier<Pose2d>) (() -> drivetrain.getStateCopy().Pose),
    //         camera0, camera1, camera2);

    private final Shooter shooter = new Shooter();
    public final Intake intake = new Intake();
    public final Turret turret = new Turret();
    public final Transfer transfer = new Transfer();
    public final Hood hood = new Hood();
    public final Climb climb = new Climb();
    public final Indexer indexer = new Indexer();
    public final Hinge hinge = new Hinge();

    public final Lookup lookup = Utils.createLookup(hood, shooter);
    public final AutoFire autoFireCommand = new AutoFire(turret, transfer, shooter, hood, intake, lookup);
    public final LockMode lockModeCommand = new LockMode(turret, shooter, hood);

    public boolean manualControl = false;

    public RobotContainer() {

        turret.setDefaultCommand(turret.initDefaultCommand(turret));
        // hinge.setDefaultCommand(hinge.initDefaultCommand(hinge));

        NamedCommands.registerCommand("testNamedCommand",
                Commands.runOnce(() -> System.out.println("this named command works")));

        SmartDashboard.putData("Git Info", new Sendable() {
            @SuppressWarnings("removal")
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addStringProperty("Branch", () -> BuildConstants.GIT_BRANCH, null);
                builder.addStringProperty("Commit", () -> BuildConstants.GIT_SHA, null);
                builder.addStringProperty("Date of commit", () -> BuildConstants.GIT_DATE, null);
                builder.addStringProperty("Uncommitted changes", () -> new Boolean(BuildConstants.DIRTY > 0).toString(),
                        null);
            }
        });

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
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
                    return drive.withVelocityX(x * MaxSpeed * 0.5)
                            .withVelocityY(y * MaxSpeed * 0.5)
                            .withRotationalRate(rot * MaxAngularRate * 0.5);
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
        joystick.rightTrigger().whileTrue(turret.turretPos());
        joystick.leftTrigger().whileTrue(turret.turretNeg());
        joystick.leftTrigger().onFalse(Commands.runOnce(() -> turret.stop()));
        joystick.rightTrigger().onFalse(Commands.runOnce(() -> turret.stop()));

        // joystick.x().whileTrue(new TurretTracking((turret)));
        // joystick.x().onFalse(Commands.runOnce(() -> turret.stop(), turret));

        joystick.x().whileTrue(
                Commands.runOnce(() -> lockModeCommand.setLockState(LockState.NEUTRAL), turret, shooter, hood));

        // SHOOTER button controls
        joystick.leftBumper().onTrue(Commands.runOnce(() -> shooter.decreaseSpeed()));
        joystick.rightBumper().onTrue(Commands.runOnce(() -> shooter.increaseSpeed()));

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

        // HINGE button controls
        // joystick.povRight()
        // .onTrue(hinge.hingeUp())
        // .onFalse(hinge.hingeStopCommand());

        // joystick.povLeft()
        // .onTrue(hinge.hingeDown())
        // .onFalse(hinge.hingeStopCommand());

        // Indexer button controls
        joystick.b().onTrue(indexer.toggleIndexer());

        configureOperatorConsole();

        drivetrain.registerTelemetry(logger::telemeterize);

        // for flight sticks controls, go to this https://gpadtester.com/ and put the
        // button id +1 (so button 1 would actually be button 2 on here)
        // rotateStick.button(3).whileTrue(AutoAlignCommand.getAutoAlignCommand(drivetrain));
        translateStick.button(2).toggleOnTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        translateStick.button(1).onTrue(Commands.runOnce(() -> transfer.toggleTransfer()));
        translateStick.button(1).onTrue(indexer.toggleIndexer());
        rotateStick.button(1).onTrue(intake.outtakeCommand());
        rotateStick.button(1).onFalse(intake.stopRollerCommand());
        rotateStick.button(2).onFalse(intake.stopRollerCommand());
        rotateStick.button(2).onTrue(intake.intakeCommand());

    }

    private void configureOperatorConsole() {
        // Operator
        operator(1)
                .onTrue(Commands.runOnce(() -> hood.moveHoodToAngle(hood.targetHoodAngle)));

        operator(OIConstants.kKeyboard_modeToggle)
                .onTrue(Commands.runOnce(() -> manualControl = !manualControl));

        operator(OIConstants.kKeyboard_lockModeToggle)
                .onTrue(Commands.runOnce(() -> System.out.println("this will toggle lock mode!")));

        operator(OIConstants.kKeyboard_lockModeLeft)
                .whileTrue(Commands.runOnce(() -> lockModeCommand.setLockState(LockState.LEFT), turret, shooter, hood));

        operator(OIConstants.kKeyboard_lockModeCenter)
                .whileTrue(
                        Commands.runOnce(() -> lockModeCommand.setLockState(LockState.CENTER), turret, shooter, hood));

        operator(OIConstants.kKeyboard_lockModeRight)
                .whileTrue(
                        Commands.runOnce(() -> lockModeCommand.setLockState(LockState.RIGHT), turret, shooter, hood));

        operator(OIConstants.kKeyboard_lockModeTrenchLeft)
                .whileTrue(Commands.runOnce(() -> lockModeCommand.setLockState(LockState.TRENCHLEFT), turret, shooter,
                        hood));

        operator(OIConstants.kKeyboard_lockModeTrenchRight)
                .whileTrue(Commands.runOnce(() -> lockModeCommand.setLockState(LockState.TRENCHRIGHT), turret, shooter,
                        hood));

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
                    .onTrue(Commands.runOnce(() -> System.out.println("Pressed operator keycode " + keyCode)));
        }
        return new JoystickButton(operatorJoystick, keyCode);
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("testAuto");
    }

}
