// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
  private final TalonFX indexerMotor;
  private final TalonFXConfiguration indexerMotorConfig;

  public boolean indexerOn = false;

  final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);

  /** Creates a new Indexer. */
  public Indexer() {
    indexerMotor = new TalonFX(IndexerConstants.kIndexerMotorId);
    indexerMotorConfig = new TalonFXConfiguration(); 
    indexerMotor.setPosition(0);

    indexerMotorConfig.CurrentLimits.StatorCurrentLimit = Constants.IndexerConstants.kIndexerCurrentLimit;
    indexerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    indexerMotorConfig.Slot0.kS = IndexerConstants.kIndexer_kS;
    indexerMotorConfig.Slot0.kV = IndexerConstants.kIndexer_kV;
    indexerMotorConfig.Slot0.kA = IndexerConstants.kIndexer_kA;
    indexerMotorConfig.Slot0.kP = IndexerConstants.kIndexer_kP;
    indexerMotorConfig.Slot0.kI = IndexerConstants.kIndexer_kI;
    indexerMotorConfig.Slot0.kD = IndexerConstants.kIndexer_kD;
    
    indexerMotorConfig.MotionMagic.MotionMagicAcceleration = IndexerConstants.kIndexerMaxAcceleration;
    indexerMotorConfig.MotionMagic.MotionMagicJerk = IndexerConstants.kIndexerMaxJerk;

    indexerMotor.getConfigurator().apply(indexerMotorConfig);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command indexerSpin() {
    return Commands.runOnce(() -> spin());
  }

  public Command indexerStop() {
    return Commands.runOnce(() -> stop());
  }

  public Command toggleIndexer() {
    return Commands.runOnce(() -> toggle());
  }
  
  public void toggle() {
    indexerOn = !indexerOn;
    if (indexerOn) {
      setVel();
    } else {
      stop();
    }
  }

  public void spin()
  {
    indexerMotor.set(IndexerConstants.kIndexerSpeed);
    indexerOn = true;
  }

  public void stop() {
    indexerMotor.set(0);
    indexerOn = false;
  }

  public void setVel() {
    double tgt = IndexerConstants.kIndexerSpeed / IndexerConstants.kIndexerGearRatio;
    indexerMotor.setControl(m_request.withVelocity(tgt)); //rps
  }
}