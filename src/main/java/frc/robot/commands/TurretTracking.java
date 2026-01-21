// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Turret;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meter;

import frc.robot.Limelight;

import edu.wpi.first.math.geometry.Translation2d;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretTracking extends Command {
  private Turret turret;

  /** Creates a new TurretTracking. */

  public TurretTracking(Turret turret) {

    this.turret = turret;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    var pose = Limelight.getPose();
    boolean isBlue = Limelight.getIsBlue();
    Translation2d hub = Limelight.getHub(isBlue);

    var angleToHub = Limelight.GetAngle();
    System.out.println("bot position is " + pose.pose);
    System.out.println(hub);
    System.out.println("angle is " + angleToHub);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
