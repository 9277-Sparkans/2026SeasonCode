// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public final Timer timer = new Timer();

  public Robot() {
    // Record metadata
    // org.littletonrobotics.junction.Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    // org.littletonrobotics.junction.Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    // org.littletonrobotics.junction.Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    // org.littletonrobotics.junction.Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    // Set up data receivers
    if (isReal()) {
      org.littletonrobotics.junction.Logger.addDataReceiver(new org.littletonrobotics.junction.wpilog.WPILOGWriter());
      org.littletonrobotics.junction.Logger
          .addDataReceiver(new org.littletonrobotics.junction.networktables.NT4Publisher());
    } else {
      org.littletonrobotics.junction.Logger
          .addDataReceiver(new org.littletonrobotics.junction.networktables.NT4Publisher());
    }

    // Start AdvantageKit logger
    org.littletonrobotics.junction.Logger.start();

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    // CommandScheduler.getInstance().schedule(m_robotContainer.lockModeCommand);
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    timer.start();
  }

  @Override
  public void teleopExit() {
    timer.stop();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
