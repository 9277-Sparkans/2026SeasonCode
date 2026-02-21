package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.Constants.HingeConstants;
import frc.robot.Constants.IntakeConstants;
// import frc.robot.subsystems.Intake.HingeState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Hinge extends SubsystemBase{

    private final TalonFX hinge;
    private final TalonFXConfiguration hingeConfig;

    MotionMagicVoltage m_request = new MotionMagicVoltage(0).withSlot(0);

    private double degToRotations(double degrees){
		return (degrees / 360.0) * Constants.HingeConstants.hingeGearRatio;
	}
    
    public enum HingeState {
        UP,
        DOWN
    }

    private HingeState hingeState;

    public Hinge() {
        hinge = new TalonFX(Constants.HingeConstants.kHingeMotorId);
		hingeConfig = new TalonFXConfiguration();

        hingeConfig.CurrentLimits.StatorCurrentLimit = Constants.HingeConstants.kHingeCurrentLimit;
        hingeConfig.CurrentLimits.StatorCurrentLimitEnable = true;

		hingeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        hingeConfig.Slot0.kP = Constants.HingeConstants.hinge_kP;
		hingeConfig.Slot0.kI = Constants.HingeConstants.hinge_kI;
		hingeConfig.Slot0.kD = Constants.HingeConstants.hinge_kD; 
		hingeConfig.Slot0.kV = Constants.HingeConstants.hinge_kV;
        hingeConfig.Slot0.kS = Constants.HingeConstants.hinge_kS;


		hingeConfig.Voltage.PeakForwardVoltage = HingeConstants.hingeMaxVoltage;
		hingeConfig.Voltage.PeakReverseVoltage = -HingeConstants.hingeMaxVoltage;
		hingeConfig.MotionMagic.MotionMagicAcceleration = Constants.HingeConstants.hingeMaxAcceleration;
		hingeConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.HingeConstants.hingeMaxVelocity;

        hinge.getConfigurator().apply(hingeConfig);

    }

    public void setState(HingeState state) {
        hingeState = state;
    }

    public HingeState getState() {
        return hingeState;
    }

    // state for climb up
    public void states(HingeState state) {

        switch (state) {
            case UP:
                hinge.setControl(m_request.withPosition(degToRotations(Constants.HingeConstants.hingeMaxDeg)));
                break;
             case DOWN:
                hinge.setControl(m_request.withPosition(70.0));
                break;

        }
    }

    /* ================= COMMANDS ================= */

    public Command hingeUp() {
        return Commands.runOnce(() -> {
            // hinge.setControl(hingey.withPosition(degToRotations(Constants.HingeConstants.hingeMaxDeg)));
            states(HingeState.UP);
        });
    }

    public Command hingeDown() {
        return Commands.runOnce(() -> {
            // hinge.setControl(hingey.withPosition(0.0));
            states(HingeState.DOWN);
        });
    }

    public Command hingeStopCommand() {
        return Commands.runOnce(() -> {
            hinge.set(0.0);
        });
    }
}
