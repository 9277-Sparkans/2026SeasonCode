// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;

public class Roller extends SubsystemBase {
  private final TalonFX rollerMotor;
  private final TalonFXConfiguration rollerMotorConfig;

  /** Creates a new Roller. */
  public Roller() {
    rollerMotor = new TalonFX(RollerConstants.roller_motorId);
    rollerMotorConfig = new TalonFXConfiguration(); 
    rollerMotor.setPosition(0);

    rollerMotorConfig.Slot0.kG = RollerConstants.roller_kG;
    rollerMotorConfig.Slot0.kP = RollerConstants.roller_kP;
    rollerMotorConfig.Slot0.kI = RollerConstants.roller_kI;
    rollerMotorConfig.Slot0.kD = RollerConstants.roller_kD;

    rollerMotorConfig.Voltage.PeakForwardVoltage = RollerConstants.roller_maxVoltage;
    rollerMotorConfig.Voltage.PeakReverseVoltage = -RollerConstants.roller_maxVoltage;
    rollerMotorConfig.MotionMagic.MotionMagicAcceleration = RollerConstants.roller_maxAcceleration;
    rollerMotorConfig.MotionMagic.MotionMagicCruiseVelocity = RollerConstants.roller_maxVelocity;

    rollerMotor.getConfigurator().apply(rollerMotorConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public Command rollerSpin () {
      return Commands.runOnce(() -> spin());
      
    }


  public void spin(){
      rollerMotor.set(RollerConstants.roller_speed);
      }


  public void stop() {
    rollerMotor.set(0);
    }

}


