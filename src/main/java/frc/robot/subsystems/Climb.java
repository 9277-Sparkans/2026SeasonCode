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

public class Climb extends SubsystemBase 
{
    //public enum ClimbState { RAISE, LOWER, ASCEND, DESCEND, HANG }
	CANBus kCANBus = CANBus.roboRIO();
	private final TalonFX climb;
	private final TalonFXConfiguration climbConfig;

	public Climb() 
	{	
		climb = new TalonFX(Constants.IntakeConstants.kClimbMotorID, kCANBus);
		climbConfig = new TalonFXConfiguration();

        climbConfig.StatorCurrentLimit = Constants.IndexerConstants.kCURRENT_LIMIT;
        climbConfig.StatorCurrentLimitEnable = true;  
        climbConfig.withNeutralMode(NeutralModeValue.Brake)
        climb.getConfigurator().apply(climbConfig);
        
	}



	public Command raise() {
        setPercentOutput(kClimb_SPEED);	}

	public Command lower() {
        setPercentOutput(-kClimb_SPEED);	}

	public Command hang() {
        indexerMotor.setControl(new NeutralOut());
	}

    public void setPercentOutput(double percent) {
        DutyCycleOut control = new DutyCycleOut(percent).withEnableFOC(false);//Could play with feild oriented control to ensure the indexer doesnt index unless in positions where robot can shoot.
        climb.setControl(control);
	

}