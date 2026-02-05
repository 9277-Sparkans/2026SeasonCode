package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.TransferConstants;;


public class Transfer extends SubsystemBase
{
    private final TalonFX transferMotor;
    private final TalonFXConfiguration transferMotorConfig;

    public boolean transferOn = false;

    /** Creates a new Transfer. */
    public Transfer() {
        transferMotor = new TalonFX(TransferConstants.transferID);
        transferMotorConfig = new TalonFXConfiguration();

        transferMotorConfig.Slot0.kS = TransferConstants.kTransfer_kS;
        transferMotorConfig.Slot0.kP = TransferConstants.kTransfer_kP;
        transferMotorConfig.Slot0.kI = TransferConstants.kTransfer_kI;
        transferMotorConfig.Slot0.kD = TransferConstants.kTransfer_kD;

        transferMotorConfig.Voltage.PeakForwardVoltage = TransferConstants.transferMaxVoltage;
        transferMotorConfig.Voltage.PeakReverseVoltage = -TransferConstants.transferMaxVoltage;
        transferMotorConfig.MotionMagic.MotionMagicAcceleration = TransferConstants.transferMaxAcceleration;
        transferMotorConfig.MotionMagic.MotionMagicCruiseVelocity = TransferConstants.transferMaxVelocity;

        transferMotor.getConfigurator().apply(transferMotorConfig);

        Telemetry.telemeterizeMotor("Transfer", transferMotor);
    }

    public Command activateTransferCommand() {
        activateTransfer();
        return Commands.runOnce(() -> {});
    }

    public Command stopTransferCommand() {
        stop();
        return Commands.run(() -> {});
    }

    public Command toggleTransferCommand() {
        toggleTransfer();
        return Commands.runOnce(() -> {});
    }

    public void toggleTransfer() {
        transferOn = !transferOn;

        if (transferOn) {
            activateTransfer();
        } else {
            stop();
        }
    }

    public void activateTransferOLD() {
        transferMotor.set(-0.5);
    }

    public void activateTransfer()
    {
        final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
            transferMotor.setControl(m_request.withVelocity(TransferConstants.kTargetTransferRPS));
    }

    public void stop() {
        transferMotor.set(0);
    }
}