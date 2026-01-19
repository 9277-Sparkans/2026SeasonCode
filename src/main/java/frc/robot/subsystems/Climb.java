package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;


import frc.robot.Constants.ClimbConstants;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Climb extends SubsystemBase 
{
    //public enum ClimbState { RAISE, LOWER, ASCEND, DESCEND, HANG }
	CANBus kCANBus = CANBus.roboRIO();
	private final TalonFX climbMotor;
	private final TalonFXConfiguration climbConfig;

	public Climb() 
	{	
		climbMotor = new TalonFX(Constants.ClimbConstants.kClimbMotorID, kCANBus);
		climbConfig = new TalonFXConfiguration();

        

		 // Current limit
        CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
        limits.StatorCurrentLimit = Constants.ClimbConstants.kClimbCURRENT_LIMIT;
        limits.StatorCurrentLimitEnable = true;
        climbMotor.getConfigurator().apply(limits);


        // Brake mode
        MotorOutputConfigs motcfg = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withDutyCycleNeutralDeadband(0.0);
        climbMotor.getConfigurator().apply(motcfg);
	}



	public void raise() {
        setPercentOutput(Constants.ClimbConstants.kClimb_SPEED);	}

	public void lower() {
        setPercentOutput(-Constants.ClimbConstants.kClimb_SPEED);	}

	public void hang() {
        climbMotor.setControl(new NeutralOut());
	}

    public void setPercentOutput(double percent) {
        DutyCycleOut control = new DutyCycleOut(percent).withEnableFOC(false);//Could play with feild oriented control to ensure the indexer doesnt index unless in positions where robot can shoot.
        climbMotor.setControl(control);
	
	}
}