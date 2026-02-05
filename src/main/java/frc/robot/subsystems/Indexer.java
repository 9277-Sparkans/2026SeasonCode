// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;

public class Indexer extends SubsystemBase {
  private final TalonFX indexerMotor;
  private final TalonFXConfiguration indexerMotorConfig;

  public boolean indexerOn = false;

  /** Creates a new Indexer. */
  public Indexer() {
    indexerMotor = new TalonFX(IndexerConstants.kIndexerMotorID);
    indexerMotorConfig = new TalonFXConfiguration(); 
    indexerMotor.setPosition(0);

    indexerMotorConfig.Slot0.kG = IndexerConstants.kIndexer_kG;
    indexerMotorConfig.Slot0.kP = IndexerConstants.kIndexer_kP;
    indexerMotorConfig.Slot0.kI = IndexerConstants.kIndexer_kI;
    indexerMotorConfig.Slot0.kD = IndexerConstants.kIndexer_kD;

    indexerMotorConfig.Voltage.PeakForwardVoltage = IndexerConstants.kIndexerMaxVoltage;
    indexerMotorConfig.Voltage.PeakReverseVoltage = -IndexerConstants.kIndexerMaxVoltage;

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
      spin();
    } else {
      stop();
    }
  }

  public void spin()
  {
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
        indexerMotor.setControl(m_request.withVelocity(IndexerConstants.kIndexerSpeedRPS));
  }

  public void stop() {
    indexerMotor.set(0);
  }
}