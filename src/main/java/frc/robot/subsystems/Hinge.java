package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.HingeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Hinge extends SubsystemBase {

    private final TalonFX hinge;
    private final TalonFXConfiguration hingeConfig;

    private final CANcoder hingeEncoder;
    private final CANcoderConfiguration hingeEncoderConfig;

    MotionMagicVoltage m_request = new MotionMagicVoltage(0.0).withSlot(0);

    double target;

    private double degToRotations(double degrees) {
        return (degrees / 360.0); //* Constants.HingeConstants.hingeGearRatio;
    }

    public enum HingeState {
        UP,
        DOWN
    }

    private HingeState hingeState;

    public Hinge() {
        hinge = new TalonFX(Constants.HingeConstants.kHingeMotorId);
        hingeConfig = new TalonFXConfiguration();

        hingeEncoder = new CANcoder(Constants.HingeConstants.kHingeEncoderId);
        hingeEncoderConfig = new CANcoderConfiguration();

        // hinge.setPosition(0.0);

        hingeConfig.CurrentLimits.StatorCurrentLimit = HingeConstants.kHingeCurrentLimit;
        hingeConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        hingeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hingeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        hingeConfig.Slot0.kP = Constants.HingeConstants.hinge_kP;
        hingeConfig.Slot0.kI = Constants.HingeConstants.hinge_kI;
        hingeConfig.Slot0.kD = Constants.HingeConstants.hinge_kD;
        hingeConfig.Slot0.kV = Constants.HingeConstants.hinge_kV;
        hingeConfig.Slot0.kS = Constants.HingeConstants.hinge_kS;
        hingeConfig.Slot0.kG = Constants.HingeConstants.hinge_kG;

        hingeConfig.Feedback.FeedbackRemoteSensorID = hingeEncoder.getDeviceID();
        hingeConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        hingeConfig.Feedback.RotorToSensorRatio = 1; // 1 motor rotation per 1 encoder rotation
        hingeConfig.Feedback.SensorToMechanismRatio = 1;

		hingeConfig.MotionMagic.MotionMagicAcceleration = Constants.HingeConstants.hingeMaxAcceleration;
		hingeConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.HingeConstants.hingeMaxVelocity;

        hinge.getConfigurator().apply(hingeConfig);        

        hingeEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        hingeEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        hingeEncoderConfig.MagnetSensor.MagnetOffset = -0.70849609375;
        hingeEncoder.getConfigurator().apply(hingeEncoderConfig);

        // target = 0.0;

        SmartDashboard.putData("Hinge", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("Velocity", () -> hinge.getVelocity().getValueAsDouble(), null);
                builder.addDoubleProperty("Absolute Encoder (non-absolute) Position", () -> (hingeEncoder.getPosition().getValueAsDouble()), (double val) -> hingeEncoder.setPosition(val));
                builder.addDoubleProperty("Absolute Encoder Position", () -> (hingeEncoder.getAbsolutePosition().getValueAsDouble()), (double val) -> hingeEncoder.setPosition(val));
                builder.addDoubleProperty("Motor Encoder Position", () -> hinge.getPosition().getValueAsDouble(), (double val) -> hinge.setPosition(val));
                builder.addDoubleProperty("Target Hinge Position", () -> target, (double val) -> target = val);
            }
        });
    }

    public void setState(HingeState state) {
        hingeState = state;
    }

    public HingeState getState() {
        return hingeState;
    }

    
    @Override
    public void periodic() {
        System.out.println(hingeEncoder.getAbsolutePosition().getValueAsDouble());
        // hinge.setPosition(hingeEncoder.getAbsolutePosition().getValueAsDouble());
        // System.out.println("hingePos is " + getPosition());
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
                hinge.setControl(m_request
                    .withPosition(0.25d));
                // target = 0.0;
                break;
             case DOWN:
                hinge.setControl(m_request.withPosition(0.05d));
                // target = 140.0;
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

    public Command hingeStopCommand() {
        return Commands.runOnce(() -> {
            hinge.set(0.0);
        });
    }

//     public void defaultCommand() {
//     // System.out.println("target is " + target);
//     // hinge.setControl(m_request.withPosition(degToRotations(target)));
//   }

//     public Command initDefaultCommand(Hinge hinge) {
//         return Commands.runOnce(() -> defaultCommand(), this);
//     }

}
