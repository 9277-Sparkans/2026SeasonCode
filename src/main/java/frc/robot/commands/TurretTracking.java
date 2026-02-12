// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

public class TurretTracking extends Command {
  private Turret turret;
  private PhotonCamera camera;
  private Transform3d robotToCamera;
  double angleToHub;

  /** Creates a new TurretTracking. */

  public TurretTracking(Turret turret, PhotonCamera camera, Transform3d robotToCamera) {

    this.turret = turret;
    this.camera = camera;
    this.robotToCamera = robotToCamera;
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
    var results = camera.getAllUnreadResults();
    boolean isBlue = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;

    // Valid IDs based on alliance
    // Blue: 25, 26; Red: 9, 10
    int id1 = isBlue ? 25 : 9;
    int id2 = isBlue ? 26 : 10;

    PhotonTrackedTarget bestTarget = null;

    if (!results.isEmpty()) {
      var result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        List<PhotonTrackedTarget> targets = result.getTargets();
        for (PhotonTrackedTarget target : targets) {
          int id = target.getFiducialId();
          if (id == id1 || id == id2) {
            bestTarget = target;
            break;
          }
        }
      }
    }

    if (bestTarget == null) {
      angleToHub = 0.0;
    } else {
      Transform3d cameraToTarget = bestTarget.getBestCameraToTarget();
      Transform3d robotToTarget = robotToCamera.plus(cameraToTarget);

      // Calculate angle in radians then convert to degrees
      // atan2(y, x) gives the angle relative to the robot's forward axis (X-axis)
      angleToHub = Math.toDegrees(Math.atan2(robotToTarget.getY(), robotToTarget.getX()));
    }

    if (angleToHub == 0.0) {
      return;
    } else {
      turret.turretMoveTgt(angleToHub);
    }
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