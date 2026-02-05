// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.commands.AutoFire;
import frc.robot.commands.TurretTracking;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Climb;


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

    public final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Shooter shooter = new Shooter();
    public final Intake intake = new Intake();
    public final Turret turret = new Turret();
    public final Transfer transfer = new Transfer();
    public final Hood hood = new Hood();
    public final Climb climb = new Climb();
    public final Indexer indexer = new Indexer();

    public final AutoFire autoFireCommand = new AutoFire(turret, transfer, shooter, hood);

    public RobotContainer() {
        NamedCommands.registerCommand("testNamedCommand", Commands.runOnce(() -> System.out.println("this named command works")));

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * 0.3) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.3) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * 0.3) // Drive counterclockwise with negative X (left)
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
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        //joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // CLIMB button controls
        joystick.povRight().whileTrue(climb.climbUp());
        joystick.povLeft().whileTrue(climb.climbDown()); 


        // HOOD button controls
        joystick.povUp().onTrue(Commands.runOnce(() -> hood.runHood()));
        joystick.povDown().onTrue(Commands.runOnce(() -> hood.runHoodReverse()));

        // joystick.povUp().onFalse(Commands.runOnce(() -> hood.stopHoodCmd()));
        joystick.povDown().onFalse(Commands.runOnce(() -> hood.stopHoodCmd()));


        // TURRET button controls
        joystick.rightTrigger().whileTrue(turret.turretPos());
        joystick.leftTrigger().whileTrue(turret.turretNeg());
        joystick.leftTrigger().onFalse(Commands.runOnce(() -> turret.stop()));
        joystick.rightTrigger().onFalse(Commands.runOnce(() -> turret.stop()));

        joystick.x().onTrue(new TurretTracking((turret)));
        joystick.x().onFalse(Commands.runOnce(() -> turret.stop(), turret));

        // SHOOTER button controls
        joystick.leftBumper().onTrue(Commands.runOnce(() -> shooter.decreaseSpeed()));
        joystick.rightBumper().onTrue(Commands.runOnce(() -> shooter.increaseSpeed()));

        // joystick.x().onTrue(shooter.shootCmd());
        // joystick.x().whileTrue(Commands.runOnce(() -> autoFireCommand.execute()));


        // TRANSFER button controls
        joystick.b().onTrue(Commands.runOnce(() -> transfer.toggleTransfer()));

        // joystick.povRight()
        //     .whileTrue(
        //         new InstantCommand(() -> intake.intakeCommand()))
        //     .onFalse(
        //         new InstantCommand(() -> intake.stopRollerCommand()));

        // INTAKE button controls
        // joystick.povRight()
        //     .whileTrue(
        //         new InstantCommand(() -> intake.intakeCommand()))
        //     .onFalse(
        //         new InstantCommand(() -> intake.stopRollerCommand()));

        // joystick.povLeft()
        //     .whileTrue(
        //         new InstantCommand(() -> intake.outtakeCommand()))
        //     .onFalse(
        //         new InstantCommand(() -> intake.stopRollerCommand()));

        // joystick.y()
        //     .whileTrue(
        //         new InstantCommand(() -> intake.deployCommand()));

        // joystick.a()
        //     .whileTrue(
        //         new InstantCommand(() -> intake.retractCommand()));
        
        
        // ROLLER button controls
        joystick.y().onTrue(indexer.toggleIndexer()); 

        drivetrain.registerTelemetry(logger::telemeterize);
    }   

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("testAuto");
    }
}