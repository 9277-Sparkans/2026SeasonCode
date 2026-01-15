package frc.robot.subsystems;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Intake extends SubsystemBase 
{
	CANBus kCANBus = CANBus.roboRIO();
	TalonFX deployment;
	TalonFX roller;

	public Intake() 
	{	
		deployment = new TalonFX(Constants.IntakeConstants.deploymentID, kCANBus);

		Slot0Configs deplomentPID = new Slot0Configs();
		deplomentPID.kP = Constants.IntakeConstants.deploymentKP;
		deplomentPID.kI = Constants.IntakeConstants.deploymentKI;
		deplomentPID.kD = Constants.IntakeConstants.deploymentKD; 

		deployment.getConfigurator().apply(deplomentPID);


		roller = new TalonFX(Constants.IntakeConstants.rollerID, kCANBus);

		Slot0Configs rollerPID = new Slot0Configs();
		rollerPID.kP = Constants.IntakeConstants.rollerKP;
		rollerPID.kI = Constants.IntakeConstants.rollerKI;
		rollerPID.kD = Constants.IntakeConstants.rollerKD; 

		roller.getConfigurator().apply(rollerPID);
	}

	public Command deployCommand() {
		return Commands.runOnce(() -> deployIntake());
	}

	public Command intakeCommand() {
		return Commands.runOnce(() -> intake());
	}

	public Command outtakeCommand() {
		return Commands.runOnce(() -> outtake());
	}

	public void deployIntake()
	{
		
	}

	public void intake()
	{
		roller.set(1);
	}

	public void outtake()
	{
		roller.set(-1);
	}
}