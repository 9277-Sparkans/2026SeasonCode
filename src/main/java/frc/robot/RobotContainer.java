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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.Constants.OIConstants;
import frc.robot.Utils.Lookup;
import frc.robot.commands.AutoFire;
import frc.robot.commands.LockMode;
import frc.robot.commands.TurretTracking;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Hinge;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandXboxController joystick = new CommandXboxController(OIConstants.kDriverControllerPort);
    public final Joystick operatorJoystick = new Joystick(OIConstants.kOperatorControllerPort);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

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
    
    public RobotContainer() {
        NamedCommands.registerCommand("testNamedCommand", Commands.runOnce(() -> System.out.println("this named command works")));

        SmartDashboard.putData("Git Info", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addStringProperty("Branch", () -> BuildConstants.GIT_BRANCH, null);
                builder.addStringProperty("Commit", () -> BuildConstants.GIT_SHA, null);
                builder.addStringProperty("Date of commit", () -> BuildConstants.GIT_DATE, null);
                builder.addStringProperty("Uncommitted changes", () -> new Boolean(BuildConstants.DIRTY > 0).toString(), null);
            }
        });

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * 0.5) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.5) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * 0.5) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.start().and(joystick.povUp()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.start().and(joystick.povDown()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.povRight()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.povLeft()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    
        // reset the field-centric heading on left bumper press
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // joystick . start () . and ( joystick . povUp () ) . whileTrue ( shooter .
        // sysIdQuasistatic ( SysIdRoutine . Direction . kForward ) ) ;
        // joystick . start () . and ( joystick . povDown () ) . whileTrue ( shooter .
        // sysIdQuasistatic ( SysIdRoutine . Direction . kReverse ) ) ;
        // joystick . start () . and ( joystick . povRight () ) . whileTrue ( shooter .
        // sysIdDynamic ( SysIdRoutine . Direction . kForward ) ) ;
        // joystick . start () . and ( joystick . povLeft () ) . whileTrue ( shooter .
        // sysIdDynamic ( SysIdRoutine . Direction . kReverse ) ) ;
        // joystick.start().and(joystick.povUp()).whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // joystick.start().and(joystick.povDown()).whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // joystick.start().and(joystick.povRight()).whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // joystick.start().and(joystick.povLeft()).whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // CLIMB button controls
        // joystick.y().onTrue(climb.climbUp());
        // joystick.y().onFalse(climb.stopCommand());
        // joystick.x().onTrue(climb.climbHang());
        // joystick.x().onFalse(climb.stopCommand());
        // joystick.a().onTrue(climb.climbDown());
        // joystick.a().onFalse(climb.stopCommand());

        joystick.a().onTrue(climb.runClimbCommand());
        joystick.a().onFalse(climb.stopCommand());
        
        joystick.y().onTrue(climb.runClimbNegCommand());
        joystick.y().onFalse(climb.stopCommand());
        
        // HOOD button controls
        // joystick.povUp().whileTrue(Commands.runOnce(() -> hood.moveHoodToAngle(hood.targetHoodAngle)));
        // joystick.povDown().onTrue(Commands.runOnce(() -> hood.runHoodReverse()));
        // joystick.povUp().onTrue(Commands.runOnce(() -> hood.moveHoodToAngle(Angle.ofBaseUnits(hood.targetHoodAngle, Degree))));
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

        // SHOOTER button controls
        joystick.leftBumper().onTrue(Commands.runOnce(() -> shooter.decreaseSpeed()));
        joystick.rightBumper().onTrue(Commands.runOnce(() -> shooter.increaseSpeed()));


        // joystick.x().onTrue(shooter.shootCmd());
        // joystick.x().whileTrue(Commands.runOnce(() -> autoFireCommand.execute()));


        // TRANSFER button controls
        // joystick.b().onTrue(Commands.runOnce(() -> transfer.toggleTransfer()));


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
        //     .onTrue(hinge.hingeUp())
        //     .onFalse(hinge.hingeStopCommand());

        // joystick.povLeft()
        //     .onTrue(hinge.hingeDown())
        //     .onFalse(hinge.hingeStopCommand());
        
        
        // Indexer button controls
        joystick.b().onTrue(indexer.toggleIndexer()); 

        // // Operator
        // operator(OIConstants.kKeyboard_lockModeLeft)
        //     .onTrue(Commands.runOnce(() -> System.out.println("Lock left")));

        // operator(OIConstants.kKeyboard_lockModeRight)
        //     .onTrue(Commands.runOnce(() -> System.out.println("Lock right")));

        // operator(OIConstants.kKeyboard_lockModeCenter)
        //     .onTrue(Commands.runOnce(() -> System.out.println("Lock center")));

        // // operator(OIConstants.kKeyboard_lockModeFire)
        // //     .onTrue(Commands.runOnce(() -> System.out.println("Lock fire")));

        // operator(OIConstants.kKeyboard_climbUp)
        //     .onTrue(climb.climbUp());

        // operator(OIConstants.kKeyboard_climbDown)
        //     .onTrue(climb.climbDown());

        // operator(OIConstants.kKeyboard_autoFire)
        //     .onTrue(autoFireCommand);

        // operator(16)
        //     .onTrue(Commands.runOnce(() -> System.out.println("boobbbbbbxxxxccsdsssxcccccccxxxx")));

        // drivetrain.registerTelemetry(logger::telemeterize);
    }

    public JoystickButton operator(int keyCode) {
        return new JoystickButton(operatorJoystick, keyCode);
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("testAuto");
    }
}