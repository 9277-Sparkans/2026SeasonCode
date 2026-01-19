package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.QuickAccessConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.TransferConstants;;


public class Transfer extends SubsystemBase
{
    private final TalonFX transferMotor;
    private final TalonFXConfiguration transferMotorConfig;

    /** Creates a new Turret. */
    public Transfer() {
        transferMotor = new TalonFX(TransferConstants.transferID);
        transferMotorConfig = new TalonFXConfiguration();

        transferMotorConfig.Voltage.PeakForwardVoltage = TransferConstants.transferMaxVoltage;
        transferMotorConfig.Voltage.PeakReverseVoltage = -TransferConstants.transferMaxVoltage;
        transferMotorConfig.MotionMagic.MotionMagicAcceleration = TransferConstants.transferMaxAcceleration;
        transferMotorConfig.MotionMagic.MotionMagicCruiseVelocity = TransferConstants.transferMaxVelocity;

        transferMotor.getConfigurator().apply(transferMotorConfig);
    }

    public Command activateTransferCommand()
    {
        return Commands.run(() -> activateTransfer());
    }

    public Command stopTransferCommand()
    {
        return Commands.run(() -> stop());
    }

    public void activateTransfer()
    {
        transferMotor.set(1);
    }

    public void stop()
    {
        transferMotor.set(0);
    }
}