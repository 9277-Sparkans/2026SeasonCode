package frc.robot.commands;

import java.util.Random;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Led;

public class LedImplement extends Command {
  private Led led;

  public LedImplement(Led led) {
    this.led = led;

    addRequirements(led);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  public static Command implement(Led led) {
    return Commands.runOnce(() ->led.m_anim0State = Led.AnimationType.Rainbow);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
