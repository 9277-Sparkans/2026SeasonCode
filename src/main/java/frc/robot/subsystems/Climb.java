package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Climb extends SubsystemBase {
    private final TalonFX climbMotor;
    private final TalonFXConfiguration climbConfig;
    MotionMagicVoltage m_request = new MotionMagicVoltage(0.0).withSlot(0);

    private ClimbState climbState;

    private double inchesToRotations(double inches){
		return (inches / 0.1) * Constants.ClimbConstants.kClimbGearRatio;
	}

    public enum ClimbState {
        DOWN,
        UP,
        HANG,
    }

    public Climb() {
        // making the climb use the canivore
        climbMotor = new TalonFX(Constants.ClimbConstants.kClimbMotorID, Constants.CanBusConstants.kCANivore);
        // making the climb use the canivore
        climbConfig = new TalonFXConfiguration();

        climbMotor.setPosition(0.0);

        /* PID */
        climbConfig.Slot0.kP = ClimbConstants.kClimb_kP;
        climbConfig.Slot0.kI = ClimbConstants.kClimb_kI;
        climbConfig.Slot0.kD = ClimbConstants.kClimb_kD;
        climbConfig.Slot0.kV = ClimbConstants.kClimb_kV;
        climbConfig.Slot0.kS = ClimbConstants.kClimb_kS;
        climbConfig.Slot0.kG = ClimbConstants.kClimb_kG;

        climbConfig.CurrentLimits.StatorCurrentLimit = ClimbConstants.kClimbCurrent_Limit;
        climbConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        /* Motion Magic */
        climbConfig.MotionMagic.MotionMagicCruiseVelocity = 100;
        climbConfig.MotionMagic.MotionMagicAcceleration = 100;

        /* Brake on neutral */
        climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        climbMotor.getConfigurator().apply(climbConfig);
    }

    public void setState(ClimbState state) {
        climbState = state;
    }

    public ClimbState getState() {
        return climbState;
    }

    public double getPosition () {
        double position = climbMotor.getPosition().getValueAsDouble();
        return position;
    }

    @Override
    public void periodic() {
        // System.out.println("climbPos is " + getPosition());
    }

    // state for climb up
    public void states(ClimbState state) {
        switch (state) {
            case UP:
                climbMotor.setControl(m_request.withPosition(ClimbConstants.kClimbUp));
                break;
            case DOWN:
                climbMotor.setControl(m_request.withPosition(ClimbConstants.kClimbDown));
                break;
            case HANG:
                climbMotor.setControl(m_request.withPosition(ClimbConstants.kClimbHang));
                break;
        }

        SmartDashboard.putString("Climb/State", climbState.name());
        SmartDashboard.putNumber("Climb/Position", climbMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Climb/Velocity", climbMotor.getVelocity().getValueAsDouble());
    }

    /* ================= COMMANDS ================= */

    public void climbUpper() {
        setState(ClimbState.UP);
    }

    public void climbDowner() {
        setState(ClimbState.DOWN);
    }

    public void climbHanger() {
        setState(ClimbState.HANG);
    }

    public Command climbUp() {
        return Commands.runOnce(() -> setState(ClimbState.UP), this);
    }

    public Command climbDown() {
        return Commands.runOnce(() -> setState(ClimbState.DOWN), this);
    }

    public Command climbHang() {
        return Commands.runOnce(() -> setState(ClimbState.HANG), this);
    }

}