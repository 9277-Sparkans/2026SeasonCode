// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

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
  private final Supplier<Pose3d[]> robotPosesSupplier;
  private final Supplier<Rotation2d> yawSupplier;

  double angleToHub;

  /**
   * Creates a new TurretTracking.
   *
   * @param turret             The turret subsystem.
   * @param robotPosesSupplier Supplies all robot pose observations from vision
   *                           (same as Vision/Summary/RobotPoses in NT).
   * @param yawSupplier        Supplies the robot's heading from the drivetrain
   *                           gyro.
   */
  public TurretTracking(Turret turret, Supplier<Pose3d[]> robotPosesSupplier,
      Supplier<Rotation2d> yawSupplier) {
    this.turret = turret;
    this.robotPosesSupplier = robotPosesSupplier;
    this.yawSupplier = yawSupplier;
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

  private Pose3d lastKnownPose = null;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose3d[] robotPoses = robotPosesSupplier.get();

    // Update last known pose if we have new vision data
    if (robotPoses != null && robotPoses.length > 0) {
      lastKnownPose = robotPoses[0];
    }

    // If we've never seen a pose, do nothing
    if (lastKnownPose == null) {
      return;
    }

    // Use the last known good pose
    Pose3d robotPose = lastKnownPose;

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

    // Angle of that direction relative to the robot's heading (use drivetrain gyro
    // yaw)
    Rotation2d robotYaw = yawSupplier.get();
    double angleToHubRad = direction.getAngle().plus(robotYaw).getRadians();

    // Normalize to -PI to PI
    angleToHubRad = Math.atan2(Math.sin(angleToHubRad), Math.cos(angleToHubRad));

    angleToHub = Math.toDegrees(angleToHubRad);

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