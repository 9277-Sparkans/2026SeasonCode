package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Climb extends SubsystemBase {
    //CANBus kCANBus = CANBus.roboRIO();
    private final TalonFX climbMotor;
    private final TalonFXConfiguration climbConfig;
    MotionMagicVoltage m_request = new MotionMagicVoltage(-5).withSlot(0);


    public enum ClimbState {
        DOWN,
        UP,
        HANG
    }

    private ClimbState climbState;

    public Climb() {

        climbMotor = new TalonFX(Constants.ClimbConstants.kClimbMotorID);
        climbConfig = new TalonFXConfiguration();

        /* PID */
        climbConfig.Slot0.kP = 3;
        climbConfig.Slot0.kI = 0.0;
        climbConfig.Slot0.kD = 0.1;
        climbConfig.Slot0.kV = 0.12;
        climbConfig.Slot0.kS = 0.3;
        climbConfig.Slot0.kG = 0;

        climbConfig.CurrentLimits.StatorCurrentLimit = 100;
        climbConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        /* Default Motion Magic (UP profile) */
        climbConfig.MotionMagic.MotionMagicCruiseVelocity = 30;
        climbConfig.MotionMagic.MotionMagicAcceleration = 15;

        climbMotor.getConfigurator().apply(climbConfig);
    }

    public void setState(ClimbState state) {
        climbState = state;
    }

    public ClimbState getState() {
        return climbState;
    }

    // state for climb up
    public void states(ClimbState state) {

        switch (state) {
            case UP:
                climbMotor.setControl(m_request.withPosition(-100.0));
                break;
            case DOWN:
                climbMotor.setControl(m_request.withPosition(0.0));
                break;
            case HANG:
                climbMotor.setControl(m_request.withPosition(-80.0));
        }
    }

    /* ================= COMMANDS ================= */

    public Command climbUp() {
        return Commands.runOnce(() -> {
            climbMotor.setControl(m_request.withPosition(-100.0));

            // states(ClimbState.UP);
        });
    }

    public Command climbDown() {
        return Commands.runOnce(() -> {
            climbMotor.setControl(m_request.withPosition(0.0));
            // states(ClimbState.DOWN);
        });
    }

    public Command climbHang() {
        return Commands.runOnce(() -> {
            climbMotor.setControl(m_request.withPosition(-80.0));
            // states(ClimbState.HANG);
        });
    }
}