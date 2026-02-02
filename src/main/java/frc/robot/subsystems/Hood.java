// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils;
import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {

  private final TalonFX hoodMotor;
  private final CANcoder hoodEncoder;
  private final TalonFXConfiguration hoodMotorConfiguration;

  public double targetHoodPosition = HoodConstants.kMinimumEncoderPos;

  public Hood() {
    hoodMotor = new TalonFX(HoodConstants.kHoodMotorId);
    hoodEncoder = new CANcoder(HoodConstants.kHoodEncoderId);
    hoodMotorConfiguration = new TalonFXConfiguration();

    // Current limiting
    CurrentLimitsConfigs hoodCurrent = new CurrentLimitsConfigs();
    hoodCurrent.StatorCurrentLimit = HoodConstants.kHoodCurrentLimit;
    hoodCurrent.StatorCurrentLimitEnable = true;
    hoodMotor.getConfigurator().apply(hoodCurrent);

    // PID + Gravity
    hoodMotorConfiguration.Slot0.kG = HoodConstants.hood_kG;
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

    hoodMotorConfiguration.Feedback.FeedbackRemoteSensorID = HoodConstants.kHoodEncoderId;
    hoodMotorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    hoodMotorConfiguration.Feedback.RotorToSensorRatio = 1; // 1 motor rotation per 1 encoder rotation
   
    hoodMotor.getConfigurator().apply(hoodMotorConfiguration);

    SmartDashboard.putData("Hood", new Sendable() {
        @Override
        public void initSendable(SendableBuilder builder) {
            builder.addDoubleProperty("Velocity", () -> hoodMotor.getVelocity().getValueAsDouble(), null);
            builder.addDoubleProperty("Position", () -> (hoodEncoder.getPosition().getValueAsDouble()), (double val) -> hoodEncoder.setPosition(val));
            builder.addDoubleProperty("Target Hood Position", () -> targetHoodPosition, (double val) -> targetHoodPosition = val);
        }
    });
  }

  @Override
  public void periodic() {
    // clampTarget();
    // moveHoodWithEncoder(targetHoodPosition);
    // System.out.println("Hood motor rotations: " + hoodMotor.getPosition().getValueAsDouble());
  }

  //motion magic

  private double hoodRotationsToMotor(double hoodRotations) {
    return hoodRotations * HoodConstants.kGearRatio;
  }

  // public void moveHoodMotionMagic(double hoodRotations) {
  //   // double motorTarget = hoodRotationsToMotor(hoodRotations);

  //   MotionMagicVoltage request =
  //       new MotionMagicVoltage(0)
  //           .withSlot(0)
  //           .withPosition(hoodRotations);

  //   hoodMotor.setControl(request);
  // }


  public void moveHoodWithEncoder(double rotation) {
    rotation = Utils.clamp(rotation, HoodConstants.kMinimumEncoderPos, HoodConstants.kMaximumEncoderPos);
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
    targetHoodPosition = Utils.clamp(targetHoodPosition, HoodConstants.kMinimumEncoderPos, HoodConstants.kMaximumEncoderPos);
  }

  public void runHood() {
    targetHoodPosition += HoodConstants.kHoodSpeed;
    clampTarget();
  }


  public void runHoodReverse() {
    targetHoodPosition -= HoodConstants.kHoodSpeed;
    clampTarget();
  }

}