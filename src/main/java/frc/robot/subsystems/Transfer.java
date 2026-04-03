package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;
import frc.robot.Constants.TransferConstants;;

public class Transfer extends SubsystemBase {
    private final TalonFX transferMotor;
    private final TalonFXConfiguration transferMotorConfig;
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    public boolean transferOn = false;

    /** Creates a new Transfer. */
    public Transfer() {
        transferMotor = new TalonFX(TransferConstants.transferID);
        transferMotorConfig = new TalonFXConfiguration();

        transferMotorConfig.Slot0.kS = TransferConstants.kTransfer_kS;
        transferMotorConfig.Slot0.kV = TransferConstants.kTransfer_kV;
        transferMotorConfig.Slot0.kA = TransferConstants.kTransfer_kA;
        transferMotorConfig.Slot0.kP = TransferConstants.kTransfer_kP;
        transferMotorConfig.Slot0.kI = TransferConstants.kTransfer_kI;
        transferMotorConfig.Slot0.kD = TransferConstants.kTransfer_kD;

        transferMotorConfig.CurrentLimits.StatorCurrentLimit = TransferConstants.kTransferCurrent_Limit;
        transferMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        transferMotorConfig.MotionMagic.MotionMagicAcceleration = TransferConstants.transferMaxAcceleration;
        transferMotorConfig.MotionMagic.MotionMagicCruiseVelocity = TransferConstants.transferMaxVelocity;

        transferMotor.getConfigurator().apply(transferMotorConfig);

        Telemetry.telemeterizeMotor("Transfer", transferMotor);

        SmartDashboard.putBoolean("Transfer/Status", transferOn);


    }

    
    @Override
    public void periodic() {
        // activateTransfer();
    }

    public Command toggleTransferCommand() {
        return Commands.runOnce(() -> toggleTransfer());
    }

    public void toggleTransfer() {
        transferOn = !transferOn;
        if (transferOn) {
            activateTransfer();
        } else {
            stop();
        }
    }

    public void activateTransfer() {
        double tgt = (TransferConstants.kTargetTransferRps / TransferConstants.kTransferGearRatio); // convert to rpm at motor
        transferMotor.setControl(m_request.withVelocity(tgt));
    }

    public void stop() {
        transferMotor.set(0);
    }
}