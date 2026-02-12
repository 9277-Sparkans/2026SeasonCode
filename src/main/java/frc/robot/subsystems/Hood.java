// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;
import frc.robot.Utils;
import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {
  private final TalonFX hoodMotor;
  private final CANcoder hoodEncoder;
  private final TalonFXConfiguration hoodMotorConfiguration;
  private final MotionMagicVoltage request = new MotionMagicVoltage(0).withSlot(0);

  public double targetHoodPosition = HoodConstants.kMinimumEncoderPos;

  public Hood() {
    hoodMotor = new TalonFX(HoodConstants.kHoodMotorId);
    hoodEncoder = new CANcoder(HoodConstants.kHoodEncoderId);
    hoodMotorConfiguration = new TalonFXConfiguration();

    // hoodMotor.setPosition(hoodEncoder.getAbsolutePosition().getValueAsDouble());

    // Current limiting
    CurrentLimitsConfigs hoodCurrent = new CurrentLimitsConfigs();
    hoodCurrent.StatorCurrentLimit = HoodConstants.kHoodCurrentLimit;
    hoodCurrent.StatorCurrentLimitEnable = true;
    hoodMotor.getConfigurator().apply(hoodCurrent);

    // PID + Gravity
    hoodMotorConfiguration.Slot0.kG = HoodConstants.hood_kG;
    hoodMotorConfiguration.Slot0.kS = HoodConstants.hood_kS;
    hoodMotorConfiguration.Slot0.kV = HoodConstants.hood_kV;
    hoodMotorConfiguration.Slot0.kA = HoodConstants.hood_kA;
    hoodMotorConfiguration.Slot0.kP = HoodConstants.hood_kP;
    hoodMotorConfiguration.Slot0.kI = HoodConstants.hood_kI;
    hoodMotorConfiguration.Slot0.kD = HoodConstants.hood_kD;

    hoodMotorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;

    // Motion Magic
    hoodMotorConfiguration.Voltage.PeakForwardVoltage = HoodConstants.hood_maxVoltage;
    hoodMotorConfiguration.Voltage.PeakReverseVoltage = -HoodConstants.hood_maxVoltage;
    hoodMotorConfiguration.MotionMagic.MotionMagicAcceleration = HoodConstants.hood_maxAcceleration;
    hoodMotorConfiguration.MotionMagic.MotionMagicCruiseVelocity = HoodConstants.hood_maxVelocity;

    // set to brake mode to stop the motor within the deadband
    hoodMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    hoodMotorConfiguration.Feedback.FeedbackRemoteSensorID = hoodEncoder.getDeviceID();
    hoodMotorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    hoodMotorConfiguration.Feedback.RotorToSensorRatio = 1; // 1 motor rotation per 1 encoder rotation
   
    hoodMotor.getConfigurator().apply(hoodMotorConfiguration);

    SmartDashboard.putData("Hood", new Sendable() {
        @Override
        public void initSendable(SendableBuilder builder) {
            builder.addDoubleProperty("Velocity", () -> hoodMotor.getVelocity().getValueAsDouble(), null);
            builder.addDoubleProperty("Absolute Encoder Position", () -> (hoodEncoder.getAbsolutePosition().getValueAsDouble()), (double val) -> hoodEncoder.setPosition(val));
            builder.addDoubleProperty("Motor Encoder Position", () -> (hoodMotor.getPosition().getValueAsDouble()), (double val) -> hoodMotor.setPosition(val));
            builder.addDoubleProperty("Target Hood Position", () -> targetHoodPosition, (double val) -> targetHoodPosition = val);
        }
    });

    // SmartDashboard.putData(hoodMotor);

    Telemetry.telemeterizeMotorWithPID("Hood (PID)", hoodMotor, 1);
  }

  @Override
  public void periodic() {
    // keep the hood motor in sync with the hood encoder absolute position;
    // MotionMagic will NOT work if you tell the motor that it has
    // a remote CANcoder on it (in my testing)
    // hoodMotor.setPosition(hoodEncoder.getAbsolutePosition().getValueAsDouble());
  }

  //motion magic

  private double hoodRotationsToMotor(double hoodRotations) {
    return hoodRotations * HoodConstants.kGearRatio;
  }

  public void moveHoodMotionMagic() {
    // double motorTarget = hoodRotationsToMotor(hoodRotations);
    clampTarget();

    System.out.println("moving to target! " + targetHoodPosition);
    hoodMotor.setControl(request.withPosition(targetHoodPosition * 1/*.6303*/)); // is this a magic number? yes. does it work? maybe
  }

  public void moveHoodToAngle(Angle angle) {
    double degrees = angle.in(Degree);

    double hoodRangeDeg = HoodConstants.kMaximumAngle - HoodConstants.kMinimumAngle;
    double hoodEncoderRange = HoodConstants.kMaximumEncoderPos - HoodConstants.kMinimumEncoderPos;

    double positionRatio = degrees / hoodRangeDeg;
    double position = HoodConstants.kMinimumEncoderPos + (hoodEncoderRange * positionRatio);
    
    targetHoodPosition = position;

    clampTarget();
    moveHoodMotionMagic();
  }

  public void moveHoodWithEncoder(double rotation) {
    // rotation = Utils.clamp(rotation, HoodConstants.kMinimumEncoderPos, HoodConstants.kMaximumEncoderPos);
    // fix `Resource leak: 'pidController' is never closed`
    try (PIDController pidController = new PIDController(HoodConstants.hood_kP, HoodConstants.hood_kI, HoodConstants.hood_kD)) {
      double feedback = pidController.calculate(hoodEncoder.getPosition().getValueAsDouble(), rotation);

      double error = rotation - (hoodEncoder.getPosition().getValueAsDouble());

      if (Math.abs(error) < 0.05) {
        hoodMotor.set(0);
      } else {
        hoodMotor.set(feedback);
      }
    }
  }

  // POV UP move hood to -0.75
  public Command moveHoodToTgtCmd() {
    // return Commands.runOnce(() -> moveHoodMotionMagic(-27)); 
    return Commands.runOnce(() -> {});
  }

  public Command stopHoodCmd() {
    hoodMotor.set(0);
    return Commands.runOnce(() -> {});
  }

  
  public void clampTarget() {
    // targetHoodPosition = Utils.clamp(targetHoodPosition, HoodConstants.kMinimumEncoderPos, HoodConstants.kMaximumEncoderPos);
  }

  public void runHood() {
    // targetHoodPosition += HoodConstants.kHoodSpeed;
    targetHoodPosition = 0.3;
    moveHoodMotionMagic();
  }


  public void runHoodReverse() {
    // targetHoodPosition -= HoodConstants.kHoodSpeed;
    hoodMotor.set(-HoodConstants.kHoodSpeed);
    // moveHoodMotionMagic();
  }

}