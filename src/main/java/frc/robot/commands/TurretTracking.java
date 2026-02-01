// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import frc.robot.Limelight;
import edu.wpi.first.math.geometry.Translation2d;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretTracking extends Command {
  private Turret turret;
  double angleToHub;

  /** Creates a new TurretTracking. */

  public TurretTracking(Turret turret) {

    this.turret = turret;
    // Use addRequirements() here to declare subsystem dependencies.

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
  public void execute() 
  {
    var pose = Limelight.getPose();
    boolean isBlue = Limelight.getIsBlue();
    Translation2d hub = Limelight.getHub(isBlue);

    angleToHub = Limelight.GetAngle();

    System.out.println(angleToHub);
    
    //return Commands.runOnce(() -> Turret.turretMoveTgt());

    //turret.setTurretToAngle(angleToHub); // hopefully this doesnt explode !
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
