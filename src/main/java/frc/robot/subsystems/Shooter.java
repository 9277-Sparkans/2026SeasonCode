// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// 

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.ControlRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private final TalonFX shooterMotor;
  private final TalonFXConfiguration shooterMotorConfig;

  final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);

  /** Creates a new Shooter. */
  public Shooter() {
    shooterMotor = new TalonFX(ShooterConstants.kShooterMotorId);
    shooterMotorConfig = new TalonFXConfiguration(); 
    shooterMotor.setPosition(0);

    shooterMotorConfig.CurrentLimits.StatorCurrentLimit = Constants.ShooterConstants.kShooterCurrentLimit;
    shooterMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    shooterMotorConfig.Slot0.kS = ShooterConstants.shooter_kS;
    shooterMotorConfig.Slot0.kV = ShooterConstants.shooter_kV;
    shooterMotorConfig.Slot0.kA = ShooterConstants.shooter_kA;
    shooterMotorConfig.Slot0.kP = ShooterConstants.shooter_kP;
    shooterMotorConfig.Slot0.kI = ShooterConstants.shooter_kI;
    shooterMotorConfig.Slot0.kD = ShooterConstants.shooter_kD;

    shooterMotorConfig.Voltage.PeakForwardVoltage = ShooterConstants.kShooterMaxVoltage;
    shooterMotorConfig.Voltage.PeakReverseVoltage = -ShooterConstants.kShooterMaxVoltage;

    shooterMotorConfig.MotionMagic.MotionMagicAcceleration = ShooterConstants.kShooterMaxAcceleration;
    shooterMotorConfig.MotionMagic.MotionMagicJerk = ShooterConstants.kShooterMaxJerk;

    shooterMotor.getConfigurator().apply(shooterMotorConfig);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command shooterSpin() {
    return Commands.runOnce(() -> spin());
  }

  public Command shooterStop() {
    return Commands.runOnce(() -> stop());
  }

  public void spin() {
    shooterMotor.set(ShooterConstants.kShooterSpeed);
  }

  public void stop() {
    shooterMotor.set(0);
  }

  public void setVel() {
    double tgt = ShooterConstants.kShooterSpeed * ShooterConstants.kShooterGearRatio;
    shooterMotor.setControl(m_request.withVelocity(tgt)); //rps
  }
}