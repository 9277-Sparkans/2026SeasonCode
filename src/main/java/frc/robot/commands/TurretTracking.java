// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;
import edu.wpi.first.math.filter.LinearFilter;

public class TurretTracking extends Command {
  private Turret turret;
  // private PhotonCamera camera; // Removed unused
  // private Transform3d robotToCamera; // Removed unused
  private final Supplier<Pose3d> poseProvider;

  double angleToHub;
  double angleTohHubLocal;

  /** Creates a new TurretTracking. */

  public TurretTracking(Turret turret, PhotonCamera camera, Transform3d robotToCamera, Supplier<Pose3d> poseProvider) {

    this.turret = turret;
    // this.camera = camera;
    // this.robotToCamera = robotToCamera;
    this.poseProvider = poseProvider;
    addRequirements(turret);

    SmartDashboard.putData("Turret Stats", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Angle to Hub", () -> angleToHub, null);
      }
    });
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose3d robotPose = poseProvider.get();

    double dx = frc.robot.Constants.FieldConstants.HUB_X - robotPose.getX();
    double dy = frc.robot.Constants.FieldConstants.HUB_Y - robotPose.getY();

    double angleToHubRad = Math.atan2(dy, dx);
    double robotYaw = robotPose.getRotation().getZ();

    double angleToHubRobotRelative = angleToHubRad - robotYaw;

    // Normalize to -180 to 180
    angleToHubRobotRelative = Math.atan2(Math.sin(angleToHubRobotRelative), Math.cos(angleToHubRobotRelative));

    angleToHub = Math.toDegrees(angleToHubRobotRelative);
    angleTohHubLocal = angleToHub; // Keeping this for whatever it was used for, presumably display

    turret.turretMoveTgt(angleToHub);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}