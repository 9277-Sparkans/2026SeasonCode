package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Intake extends SubsystemBase 
{
	CANBus kCANBus = CANBus.roboRIO();
	private final TalonFX deployment;
	private final TalonFXConfiguration deploymentConfig;

	private final TalonFX roller;
	private final TalonFXConfiguration rollerConfig;
	//added for test
	private double degToRotations(double degrees){
			return (degrees / 360.0) * IntakeConstants.deploymentGearRatio;
		}

	public Intake() 
	{	
				
		deployment = new TalonFX(Constants.IntakeConstants.deploymentID, kCANBus);
		deploymentConfig = new TalonFXConfiguration();

		deploymentConfig.Slot0.kP = Constants.IntakeConstants.deploymentKP;
		deploymentConfig.Slot0.kI = Constants.IntakeConstants.deploymentKI;
		deploymentConfig.Slot0.kD = Constants.IntakeConstants.deploymentKD; 

		// deploymentConfig.Voltage.PeakForwardVoltage = IntakeConstants.deploymentMaxVoltage;
		// deploymentConfig.Voltage.PeakReverseVoltage = -IntakeConstants.deploymentMaxVoltage;
		// deploymentConfig.MotionMagic.MotionMagicAcceleration = IntakeConstants.deploymentMaxAcceleration;
		// deploymentConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.deploymentMaxVelocity;

		roller = new TalonFX(Constants.IntakeConstants.rollerID, kCANBus);
		rollerConfig = new TalonFXConfiguration();

		rollerConfig.Slot0.kP = Constants.IntakeConstants.rollerKP;
		rollerConfig.Slot0.kI = Constants.IntakeConstants.rollerKI;
		rollerConfig.Slot0.kD = Constants.IntakeConstants.rollerKD;

		// rollerConfig.Voltage.PeakForwardVoltage = IntakeConstants.rollerMaxVoltage;
		// rollerConfig.Voltage.PeakReverseVoltage = -IntakeConstants.rollerMaxVoltage;
		// rollerConfig.MotionMagic.MotionMagicAcceleration = IntakeConstants.rollerMaxAcceleration;
		// rollerConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.rollerMaxVelocity;

		deployment.getConfigurator().apply(deploymentConfig);
    	roller.getConfigurator().apply(rollerConfig);
	}

	public Command deployCommand() {
		return Commands.runOnce(() -> deployIntake());
	}

	public Command retractCommand() {
		return Commands.runOnce(() -> retractIntake());
	}

	public Command intakeCommand() {
		return Commands.runOnce(() -> intake());
	}

	public Command outtakeCommand() {
		return Commands.runOnce(() -> outtake());
	}
	
	public Command stopRollerCommand() {
		return Commands.runOnce(() -> stopRoller());
	}

	// public void deployIntake()
	// {
	// 	MotionMagicVoltage deploymentRequest = new MotionMagicVoltage(IntakeConstants.deploymentMaxDeg).withSlot(0);
	// 	deployment.setControl(deploymentRequest.withPosition(GetDeploymentPosition()));
	// }
//
	public void deployIntake(){
		MotionMagicVoltage request = 
		new MotionMagicVoltage(0).withSlot(0)
		.withPosition(degToRotations(IntakeConstants.deploymentMaxDeg));
	
		// MotionMagicVoltage deploymentRequest = new MotionMagicVoltage(0).withSlot(0);
		deployment.setControl(request.withPosition(GetDeploymentPosition()));
	}

	public void retractIntake()
	{
		MotionMagicVoltage request = 
		new MotionMagicVoltage(0).withSlot(0)
		.withPosition(degToRotations(0));
	
		// MotionMagicVoltage deploymentRequest = new MotionMagicVoltage(0).withSlot(0);
		deployment.setControl(request.withPosition(GetDeploymentPosition()));
	}

	public void intake()
	{
		roller.set(IntakeConstants.intakeSpeed);
	}

	public void outtake()
	{
		roller.set(-IntakeConstants.intakeSpeed);
	}

	public void stopRoller()
	{
		roller.set(0);
	}

	public double GetDeploymentPosition()
	{
		double rawPosition = deployment.getPosition().getValueAsDouble() / Constants.IntakeConstants.deploymentGearRatio;
		double normalizedValue = (rawPosition / Constants.IntakeConstants.deploymentCountsPerRevolution);
		return normalizedValue * 360;
	}
}