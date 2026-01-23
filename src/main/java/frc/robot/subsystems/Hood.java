// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.channels.SelectableChannel;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Limelight;
import frc.robot.Constants.HoodConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;

public class Hood extends SubsystemBase {
  public double tgtAngle = HoodConstants.kMaximumAngle;
  private final TalonFX hoodMotor;
  private final TalonFXConfiguration hoodMotorConfiguration;
// -33
  public Hood() {
    hoodMotor = new TalonFX(HoodConstants.kHoodMotorId);
    hoodMotorConfiguration = new TalonFXConfiguration();
    hoodMotor.setPosition(0);

    CurrentLimitsConfigs hoodConfigs = new CurrentLimitsConfigs();
    hoodConfigs.StatorCurrentLimit = HoodConstants.kHoodCurrentLimit;
    hoodConfigs.StatorCurrentLimitEnable = true;
    hoodMotor.getConfigurator().apply(hoodConfigs);

    hoodMotorConfiguration.Slot0.kG = HoodConstants.hood_kG;
    hoodMotorConfiguration.Slot0.kP = HoodConstants.hood_kP;
    hoodMotorConfiguration.Slot0.kI = HoodConstants.hood_kI;
    hoodMotorConfiguration.Slot0.kD = HoodConstants.hood_kD; 

    hoodMotorConfiguration.Voltage.PeakForwardVoltage = HoodConstants.hood_maxVoltage;
    hoodMotorConfiguration.Voltage.PeakReverseVoltage = -HoodConstants.hood_maxVoltage;
    hoodMotorConfiguration.MotionMagic.MotionMagicAcceleration = HoodConstants.hood_maxAcceleration;
    hoodMotorConfiguration.MotionMagic.MotionMagicCruiseVelocity = HoodConstants.hood_maxVelocity;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println("Hood Angle: " + GetCurrentHoodAngle());
    System.out.println("Target angle: " + tgtAngle);
    //System.out.println("Target hood angle: " + GetTargetHoodAngle());
  }

  public Command runHoodCmd() {
        return Commands.runOnce(() -> runHood());
    }

  public Command runHoodReverseCmd() {
        return Commands.runOnce(() -> runHoodReverse());
    }

  public Command stopHoodCmd() {
        return Commands.runOnce(() -> stopHood());
    }
    public Command moveHoodUpCmd(){
      if (GetCurrentHoodAngle() < HoodConstants.kMaximumAngle - 0.5)
      {
        runHood();
      }
      else
      {
        stopHood();
      }
      return Commands.runOnce(() -> {});
    }
  

    public Command moveHoodDownCmd(){
      if (GetCurrentHoodAngle() > HoodConstants.kMinimumAngle + 0.5)
      {
        runHoodReverse();
      }
      else
      {
        stopHood();
      }
      return Commands.runOnce(() -> {});
    }

    // hood
    public void moveHoodToAngle(double theta){
      theta = tgtAngle;
      // MotionMagicVoltage hoodRequest = new MotionMagicVoltage(-theta).withSlot(0);
      // hoodMotor.setControl(hoodRequest.withPosition(GetCurrentHoodAngle()));

      PIDController pidController = new PIDController(HoodConstants.hood_kP, HoodConstants.hood_kI, HoodConstants.hood_kD);

      double feedbackVoltage = pidController.calculate(GetCurrentHoodAngle(), tgtAngle);
      hoodMotor.setVoltage(feedbackVoltage);
    }
    
    public void runHood() {
        hoodMotor.set(HoodConstants.kHoodSpeed);
    }

    public void runHoodReverse() {
        hoodMotor.set(-HoodConstants.kHoodSpeed);
    }

    public void stopHood() {
        hoodMotor.set(0);
    }

    public void moveHoodDown()
    {
        if (tgtAngle - HoodConstants.kHoodIncrement >= HoodConstants.kMinimumAngle + 0.5)
        {
            tgtAngle -= HoodConstants.kHoodIncrement;
        }
    }

    public void moveHoodUp()
    {
        if (tgtAngle + HoodConstants.kHoodIncrement <= HoodConstants.kMaximumAngle - 0.5)
        {
            tgtAngle += HoodConstants.kHoodIncrement;
        }
    }

    public double GetTargetHoodAngle()
    {
        Translation2d position = Limelight.GetDistance();
        double distance = Math.sqrt(position.getX() * position.getX() + position.getY() * position.getY());

        // field length 158.6  inches or 4.02844 meters
        double angle = (distance / 4.02844) * (HoodConstants.kMaximumAngle - HoodConstants.kMinimumAngle);

        return HoodConstants.kMinimumAngle + angle; // replace with actual math later
    }

    public double GetCurrentHoodAngle()
    {
        double currentHoodPosition = hoodMotor.getPosition().getValueAsDouble(); // rotations
        double hoodSpace = currentHoodPosition / HoodConstants.kGearRatio;
        double hoodAngle = hoodSpace * 360;

        return Math.max(HoodConstants.kMaximumAngle + hoodAngle, HoodConstants.kMinimumAngle);
    }
}
