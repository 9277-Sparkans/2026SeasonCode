package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.HingeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Hinge extends SubsystemBase {

    private final TalonFX hinge;
    private final TalonFXConfiguration hingeConfig;

    MotionMagicVoltage m_request = new MotionMagicVoltage(0.0).withSlot(0);

    private double degToRotations(double degrees) {
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

        hingeConfig.CurrentLimits.StatorCurrentLimit = HingeConstants.kHingeCurrentLimit;
        hingeConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        hingeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        hingeConfig.Slot0.kP = Constants.HingeConstants.hinge_kP;
        hingeConfig.Slot0.kI = Constants.HingeConstants.hinge_kI;
        hingeConfig.Slot0.kD = Constants.HingeConstants.hinge_kD;
        hingeConfig.Slot0.kV = Constants.HingeConstants.hinge_kV;
        hingeConfig.Slot0.kS = Constants.HingeConstants.hinge_kS;
        // hingeConfig.Slot0.kG = Constants.HingeConstants.hinge_kG;

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

    @Override
    public void periodic() {
        System.out.println("hingePos is " + getPosition());
    }

    public double getPosition() {
        double position = hinge.getPosition().getValueAsDouble() / HingeConstants.hingeGearRatio * 360.0;
        return position;
    }

    // state for climb up
    public void states(HingeState state) {
        setState(state);
        switch (state) {
            case UP:
                hinge.setControl(m_request.withPosition(degToRotations(-200.0)));
                break;
            case DOWN:
                hinge.setControl(m_request.withPosition(degToRotations(-100.0)));
                break;

        }
    }

    /* ================= COMMANDS ================= */

    public Command hingeUp() {
        return Commands.runOnce(() -> {
            states(HingeState.UP);
        });
    }

    public Command hingeDown() {
        return Commands.runOnce(() -> {
            states(HingeState.DOWN);
        });
    }

    // public Command hingeToggle() {
    // return Commands.runOnce(() -> {
    // if (hingeState == HingeState.DOWN) {
    // hingeState = HingeState.UP;
    // } else {
    // hingeState = HingeState.DOWN;
    // }

    // states(hingeState);
    // });
    // }

    public Command hingeStopCommand() {
        return Commands.runOnce(() -> {
            hinge.set(0.0);
        });
    }

}
