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

import edu.wpi.first.math.geometry.Translation2d;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretTracking extends Command {
  private static final Translation2d redHub = new Translation2d(11.915521, 4.034536);
  // private static final Translation2d blueHub = new Translation2d(182.105, 158.84);
  private static final Translation2d blueHub = new Translation2d(4.625467, 4.034536);

  private Turret turret;

  boolean isBlue = false;

  // public double GetTx()
  // {
  //   boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;

  //   long tid = NetworkTableInstance.getDefault().getTable("limelight-a").getEntry("tid").getInteger(0);
  //   if (isBlue)
  //   {
  //     return (tid == 26 || tid == 25) ? NetworkTableInstance.getDefault().getTable("limelight-a").getEntry("tx").getDouble(1111) : 222222; 
  //   }
  //   else
  //   {
  //     return (tid == 9 || tid == 10) ? NetworkTableInstance.getDefault().getTable("limelight-a").getEntry("tx").getDouble(0) : 0; 
  //   }  }


  
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
  public void execute() {
    //System.out.println("Tx is " + GetTx());
    
    //System.out.println("Tag ID is " + tid)

    var pose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-a");

    boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;

    final Translation2d hub;

    if (isBlue){
      hub = blueHub;
    }
    else {
      hub = redHub;
    }
    //var distanceToHub = hub.minus(pose.pose.getTranslation());
    var angleToHub = hub.minus(pose.pose.getTranslation()).getAngle().getDegrees();
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
