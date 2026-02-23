// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;
import frc.robot.Constants.TurretConstants;

public class TurretTracking extends Command {
  private Turret turret;
  private final Supplier<Pose2d> poseSupplier;

  double angleToHub;

  /**
   * Creates a new TurretTracking.
   *
   * @param turret       The turret subsystem.
   * @param poseSupplier Supplies the robot's pose from the drivetrain.
   */
  public TurretTracking(Turret turret, Supplier<Pose2d> poseSupplier) {
    this.turret = turret;
    this.poseSupplier = poseSupplier;
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
    Pose2d robotPose2d = poseSupplier.get();

    if (robotPose2d == null) {
      return;
    }

    Pose3d robotPose = new Pose3d(robotPose2d);

    // Calculate turret position in field coordinates
    Translation2d turretTranslation = robotPose
        .transformBy(TurretConstants.ROBOT_TO_TURRET_TRANSFORM)
        .toPose2d()
        .getTranslation();

    // Target position (Hub)
    Translation2d target = new Translation2d(
        frc.robot.Constants.FieldConstants.HUB_X,
        frc.robot.Constants.FieldConstants.HUB_Y);

    // Direction from turret to hub
    Translation2d direction = target.minus(turretTranslation);

    // Angle of that direction relative to the robot's heading (use drivetrain pose
    // rotation)
    Rotation2d robotYaw = robotPose2d.getRotation();
    double angleToHubRad = direction.getAngle().plus(robotYaw).getRadians();

    // Normalize to -PI to PI
    angleToHubRad = Math.atan2(Math.sin(angleToHubRad), Math.cos(angleToHubRad));

    angleToHub = Math.toDegrees(angleToHubRad);

    turret.target = angleToHub;
    turret.defaultCommand();
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