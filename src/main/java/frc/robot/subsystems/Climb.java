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
    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0.0).withSlot(0);
    private final NeutralOut m_neutral = new NeutralOut();

    private ClimbState climbState = ClimbState.STOP;

    public enum ClimbState {
        DOWN,
        UP,
        HANG,
        STOP,
        MANUAL
    }

    public Climb() {
        // making the climb use the canivore
        climbMotor = new TalonFX(Constants.ClimbConstants.kClimbMotorID, Constants.CanBusConstants.kCANivore);
        climbConfig = new TalonFXConfiguration();

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

    @Override
    public void periodic() {
        switch (climbState) {
            case UP:
                climbMotor.setControl(m_request.withPosition(ClimbConstants.kClimbUp));
                break;
            case DOWN:
                climbMotor.setControl(m_request.withPosition(ClimbConstants.kClimbDown));
                break;
            case HANG:
                climbMotor.setControl(m_request.withPosition(ClimbConstants.kClimbHang));
                break;
            case STOP:
                climbMotor.setControl(m_neutral);
                break;
            case MANUAL:
                // Motor set directly; don't override here
                break;
        }

        SmartDashboard.putString("Climb/State", climbState.name());
        SmartDashboard.putNumber("Climb/Position", climbMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Climb/Velocity", climbMotor.getVelocity().getValueAsDouble());
    }

    /* ================= COMMANDS ================= */

    public Command climbUp() {
        return Commands.runOnce(() -> setState(ClimbState.UP), this);
    }

    public Command climbDown() {
        return Commands.runOnce(() -> {
            setState(ClimbState.DOWN);
            System.out.println("climb down");
        }, this);
    }

    public Command climbHang() {
        return Commands.runOnce(() -> setState(ClimbState.HANG), this);
    }

    public Command stopCommand() {
        return Commands.runOnce(() -> setState(ClimbState.STOP), this);
    }

    public Command runClimbCommand() {
        return Commands.startEnd(
                () -> {
                    setState(ClimbState.MANUAL);
                    climbMotor.set(0.5);
                },
                () -> setState(ClimbState.STOP),
                this);
    }

    public void runClimb() {
        setState(ClimbState.MANUAL);
        climbMotor.set(0.5);
    }

    public Command runClimbNegCommand() {
        return Commands.startEnd(
                () -> {
                    setState(ClimbState.MANUAL);
                    climbMotor.set(-0.5);
                },
                () -> setState(ClimbState.STOP),
                this);
    }

    public void runClimbNeg() {
        setState(ClimbState.MANUAL);
        climbMotor.set(-0.5);
    }

    public void stop() {
        setState(ClimbState.STOP);
    }
}