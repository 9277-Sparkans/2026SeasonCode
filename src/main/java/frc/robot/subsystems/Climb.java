package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Climb extends SubsystemBase {
    // CANBus kCANBus = CANBus.roboRIO();
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
        climbConfig.Slot0.kP = ClimbConstants.kClimb_kP;
        climbConfig.Slot0.kI = ClimbConstants.kClimb_kI;
        climbConfig.Slot0.kD = ClimbConstants.kClimb_kD;
        climbConfig.Slot0.kV = ClimbConstants.kClimb_kV;
        climbConfig.Slot0.kS = ClimbConstants.kClimb_kS;
        climbConfig.Slot0.kG = ClimbConstants.kClimb_kG;

        climbConfig.CurrentLimits.StatorCurrentLimit = ClimbConstants.kClimbCurrent_Limit;
        climbConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        /* Default Motion Magic (UP profile) */
        climbConfig.MotionMagic.MotionMagicCruiseVelocity = ClimbConstants.kClimbMaxVelocity;
        climbConfig.MotionMagic.MotionMagicAcceleration = ClimbConstants.kClimbMaxAcceleration;
        climbConfig.Voltage.PeakForwardVoltage = ClimbConstants.kClimbMaxVoltage;
        climbConfig.Voltage.PeakReverseVoltage = -ClimbConstants.kClimbMaxVoltage;

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
                climbMotor.setControl(m_request.withPosition(-100));
                break;
            case DOWN:
                climbMotor.setControl(m_request.withPosition(-5));
                break;
            case HANG:
                climbMotor.setControl(m_request.withPosition(-80));
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
            // climbMotor.setControl(m_request.withPosition(-80.0));
            states(ClimbState.HANG);
        });
    }

    public Command stopCommand() {
        return Commands.runOnce(() -> stop()) ;
    }

    public Command runClimbCommand() {
        return Commands.runOnce(() -> runClimb());
    }

    public void runClimb() {
        climbMotor.set(-0.1);
    }

    public void stop(){
        climbMotor.set(0);

    }
}