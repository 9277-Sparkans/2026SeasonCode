package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Intake extends SubsystemBase {
	
	private final TalonFX intakeMotor;
	private final TalonFXConfiguration intakeMotorConfig;
  	final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);


	public Intake() {	
	
		intakeMotor = new TalonFX(Constants.IntakeConstants.intakeMotorId);
		intakeMotorConfig = new TalonFXConfiguration();

		intakeMotorConfig.Slot0.kP = Constants.IntakeConstants.intake_kP;
		intakeMotorConfig.Slot0.kI = Constants.IntakeConstants.intake_kI;
		intakeMotorConfig.Slot0.kD = Constants.IntakeConstants.intake_kD;
		intakeMotorConfig.Voltage.PeakForwardVoltage = IntakeConstants.intakeMaxVoltage;
		intakeMotorConfig.Voltage.PeakReverseVoltage = -IntakeConstants.intakeMaxVoltage;
		intakeMotorConfig.MotionMagic.MotionMagicAcceleration = IntakeConstants.intakeMaxAcceleration;
		intakeMotorConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.intakeMaxVelocity;

    	intakeMotor.getConfigurator().apply(intakeMotorConfig);
	}

	
	public Command intakeCommand() {
		return Commands.runOnce(() -> intake());
	}

	public Command outtakeCommand() {
		return Commands.runOnce(() -> outtake());
	}
	
	public Command stopRollerCommand() {
		return Commands.runOnce(() -> stop());
	}

	public void intake()
	{
		intakeMotor.set(IntakeConstants.intakeSpeed);
		
	}

	public void outtake()
	{
		intakeMotor.set(-IntakeConstants.intakeSpeed);
	}

	public void stop()
	{
		intakeMotor.set(0);
	}

	public void setVel() {
		double tgt = IntakeConstants.intakeSpeed / IntakeConstants.kIntakeGearRatio;
		intakeMotor.setControl(m_request.withVelocity(tgt)); //rps
  	}

}