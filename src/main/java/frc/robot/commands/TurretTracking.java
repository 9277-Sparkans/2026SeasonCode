package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

public class TurretTracking extends Command {
  private Turret turret;

  /**
   * Creates a new TurretTracking.
   *
   * @param turret The turret subsystem.
   */
  public TurretTracking(Turret turret) {
    this.turret = turret;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.stop(); // Reset filter and state before starting tracking
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Select target based on alliance
    var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance()
        .orElse(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue);
    edu.wpi.first.math.geometry.Translation3d target = (alliance == edu.wpi.first.wpilibj.DriverStation.Alliance.Red)
        ? frc.robot.Constants.FieldConstants.HUB_RED
        : frc.robot.Constants.FieldConstants.HUB_BLUE;

    // Track the target using the new subsystem method
    turret.trackTarget(target);
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