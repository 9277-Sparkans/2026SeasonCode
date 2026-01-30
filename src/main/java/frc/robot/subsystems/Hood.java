// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;
import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {

  private final TalonFX hoodMotor;
  private final TalonFXConfiguration hoodMotorConfiguration;

  public Hood() {
    hoodMotor = new TalonFX(HoodConstants.kHoodMotorId);
    hoodMotorConfiguration = new TalonFXConfiguration();

    hoodMotor.setPosition(0);

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

    // Motion Magic
    hoodMotorConfiguration.Voltage.PeakForwardVoltage = HoodConstants.hood_maxVoltage;
    hoodMotorConfiguration.Voltage.PeakReverseVoltage = -HoodConstants.hood_maxVoltage;
    hoodMotorConfiguration.MotionMagic.MotionMagicAcceleration = HoodConstants.hood_maxAcceleration;
    hoodMotorConfiguration.MotionMagic.MotionMagicCruiseVelocity = HoodConstants.hood_maxVelocity;

   
    hoodMotor.getConfigurator().apply(hoodMotorConfiguration);

    Telemetry.telemeterizeMotor("Hood", hoodMotor, HoodConstants.kGearRatio);
  }

  @Override
  public void periodic() {
    // System.out.println("Hood motor rotations: " + hoodMotor.getPosition().getValueAsDouble());
  }

  

  //motion magic

  private double hoodRotationsToMotor(double hoodRotations) {
    return hoodRotations * HoodConstants.kGearRatio;
  }

  public void moveHoodMotionMagic(double hoodRotations) {
    double motorTarget = hoodRotationsToMotor(hoodRotations);

    MotionMagicVoltage request =
        new MotionMagicVoltage(0)
            .withSlot(0)
            .withPosition(motorTarget);

    hoodMotor.setControl(request);
  }

  

  // POV UP move hood to -0.75
  public Command moveHoodToTgtCmd() {
    return Commands.runOnce(() -> moveHoodMotionMagic(5.0)); 
  }

  public Command stopHoodCmd() {
    //return Commands.runOnce(() -> hoodMotor.set(0));
    hoodMotor.set(0);
    return Commands.runOnce(() -> {});
  }

  

  public void runHood() {
    hoodMotor.set(HoodConstants.kHoodSpeed);
  }

  public void runHoodReverse() {
    hoodMotor.set(-HoodConstants.kHoodSpeed);
  }

}