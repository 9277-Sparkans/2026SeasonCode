// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretTracking extends Command {
  private Turret turret;
  double angleToHub;

  private final Supplier<Pose2d> poseSupplier;
  private static final Translation2d targetLocation = new Translation2d(4.0218614, 4.2124376); // Midpoint of 25 and 26

  /** Creates a new TurretTracking. */
  public TurretTracking(Turret turret, Supplier<Pose2d> poseSupplier) {
    this.turret = turret;
    this.poseSupplier = poseSupplier;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = poseSupplier.get();

    double targetAngleRad = Math.atan2(targetLocation.getY() - robotPose.getY(),
        targetLocation.getX() - robotPose.getX());
    double targetAngleDeg = Math.toDegrees(targetAngleRad);

    double robotRotationDeg = robotPose.getRotation().getDegrees();

    double turretAngleDeg = targetAngleDeg - robotRotationDeg;

    // Normalize to -180 to 180
    while (turretAngleDeg > 180)
      turretAngleDeg -= 360;
    while (turretAngleDeg < -180)
      turretAngleDeg += 360;

    turret.setTurretToAngle(turretAngleDeg);
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
