package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Limelight;
import frc.robot.Telemetry;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {

  private final TalonFX turretMotor;
  private final TalonFXConfiguration turretMotorConfig;
  public double turretOffset = 0.0;

  final MotionMagicVoltage m_request = new MotionMagicVoltage(0.0);



  /** Creates a new Turret. */
  public Turret() {
    turretMotor = new TalonFX(TurretConstants.turret_motorId);
    turretMotorConfig = new TalonFXConfiguration();
    turretMotor.setPosition(0);

    turretMotorConfig.Slot0.kS = TurretConstants.turret_kS;
    turretMotorConfig.Slot0.kV = TurretConstants.turret_kV;
    turretMotorConfig.Slot0.kA = TurretConstants.turret_kA;
    turretMotorConfig.Slot0.kP = TurretConstants.turret_kP;
    turretMotorConfig.Slot0.kI = TurretConstants.turret_kI;
    turretMotorConfig.Slot0.kD = TurretConstants.turret_kD;

    turretMotorConfig.Voltage.PeakForwardVoltage = TurretConstants.turret_maxVoltage;
    turretMotorConfig.Voltage.PeakReverseVoltage = -TurretConstants.turret_maxVoltage;
    turretMotorConfig.MotionMagic.MotionMagicAcceleration = TurretConstants.turret_maxAcceleration;
    turretMotorConfig.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.turret_maxVelocity;
    turretMotorConfig.MotionMagic.MotionMagicJerk = TurretConstants.turret_maxJerk;

    turretMotor.getConfigurator().apply(turretMotorConfig);

    Telemetry.telemeterizeMotor("Turret", turretMotor, (108.0 / 15.0));


  }

  @Override
  public void periodic() {

  }

  public double getPosition() {
    double position = turretMotor.getPosition().getValueAsDouble() / TurretConstants.kGearRatio;
    return position * 360; 
  }

  public double getTurretCurrent() {
    double turretCurrent = turretMotor.getSupplyCurrent().getValueAsDouble();
    return turretCurrent;
  }

  public Command turretPos () {
      return Commands.runOnce(() -> spinPositive());
    }

  public Command turretNeg () {
      return Commands.runOnce(() -> spinNegative());
    }


  public double getTurretAngle() {
    double position = getTurretCurrent(); // turns
    return position * 360;
  }

  public double getVelocity() {
    double turretVelocity = turretMotor.getVelocity().getValueAsDouble();
    return (turretVelocity);
  }

  public void spinPositive() {
    turretMotor.set(TurretConstants.turret_speed);
  }

  public void spinNegative() {
    turretMotor.set(-TurretConstants.turret_speed);
  }

  public void turretMoveTgt(double llAngle){

    boolean isAtTarget = Math.abs(turretMotor.getClosedLoopError().getValue()) < 1.5;
    double tgt = (-llAngle * 10 * TurretConstants.kGearRatio) / 360;

    System.out.println(llAngle * 10);

    turretMotor.setControl(m_request.withPosition(tgt)); //motor rotations
    
  }


  public void stop() {
    turretMotor.set(0);
  }

}