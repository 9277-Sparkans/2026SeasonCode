package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Telemetry;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants;
import frc.robot.Constants;

public class Turret extends SubsystemBase {

  private final TalonFX turretMotor;
  private final TalonFXConfiguration turretMotorConfig;
  public double turretOffset = 0.0;

  public double target;

  final MotionMagicVoltage m_request = new MotionMagicVoltage(0.0);

  NeutralModeValue brake = NeutralModeValue.Brake;

  // final DigitalInput turret_forwardLimit = new DigitalInput(0);
  // final DigitalInput turret_reverseLimit = new DigitalInput(1);

  // final DutyCycleOut turret_dutyCycle = new DutyCycleOut(0.0);

  /** Creates a new Turret. */
  public Turret() {
    turretMotor = new TalonFX(TurretConstants.turret_motorId);
    turretMotorConfig = new TalonFXConfiguration();
    turretMotor.setPosition(0);

    // turretMotor.setControl(
    // turret_dutyCycle.withOutput(0.5)
    //     .withLimitForwardMotion(turret_forwardLimit.get())
    //     .withLimitReverseMotion(turret_reverseLimit.get())
    // );

    turretMotorConfig.CurrentLimits.StatorCurrentLimit = TurretConstants.turret_currentLimit;
    turretMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    turretMotorConfig.CurrentLimits.SupplyCurrentLimit = TurretConstants.turret_currentLimit;
    turretMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    turretMotorConfig.MotorOutput.NeutralMode = brake;
  

    turretMotorConfig.Slot0.kS = TurretConstants.turret_kS;
    turretMotorConfig.Slot0.kV = TurretConstants.turret_kV;
    turretMotorConfig.Slot0.kA = TurretConstants.turret_kA;
    turretMotorConfig.Slot0.kP = TurretConstants.turret_kP;
    turretMotorConfig.Slot0.kI = TurretConstants.turret_kI;
    turretMotorConfig.Slot0.kD = TurretConstants.turret_kD;

    turretMotorConfig.Slot1.kS = TurretConstants.turret_kS1;
    turretMotorConfig.Slot1.kV = TurretConstants.turret_kV1;
    turretMotorConfig.Slot1.kA = TurretConstants.turret_kA1;
    turretMotorConfig.Slot1.kP = TurretConstants.turret_kP1;
    turretMotorConfig.Slot1.kI = TurretConstants.turret_kI1;
    turretMotorConfig.Slot1.kD = TurretConstants.turret_kD1;

    turretMotorConfig.Slot2.kS = TurretConstants.turret_kS2;
    turretMotorConfig.Slot2.kV = TurretConstants.turret_kV2;
    turretMotorConfig.Slot2.kA = TurretConstants.turret_kA2;
    turretMotorConfig.Slot2.kP = TurretConstants.turret_kP2;
    turretMotorConfig.Slot2.kI = TurretConstants.turret_kI2;
    turretMotorConfig.Slot2.kD = TurretConstants.turret_kD2;

    // last tested with this
    // turretMotorConfig.Voltage.PeakForwardVoltage = TurretConstants.turret_maxVoltage;
    // turretMotorConfig.Voltage.PeakReverseVoltage = -TurretConstants.turret_maxVoltage;
    turretMotorConfig.MotionMagic.MotionMagicAcceleration = TurretConstants.turret_maxAcceleration;
    turretMotorConfig.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.turret_maxVelocity;
    turretMotorConfig.MotionMagic.MotionMagicJerk = TurretConstants.turret_maxJerk;

    // last tested without this    
    turretMotorConfig.CurrentLimits.SupplyCurrentLimit = TurretConstants.kTurretCurrentLimit;
    turretMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      
    turretMotorConfig.CurrentLimits.StatorCurrentLimit = TurretConstants.kTurretCurrentLimit;
    turretMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    turretMotor.getConfigurator().apply(turretMotorConfig);

    Telemetry.telemeterizeMotorWithPID("Turret", turretMotor, (108.0 / 15.0));

    target = 0.0;
  }

  @Override
  public void periodic() {
    defaultCommand();
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
    return getPosition();
  }

  public double getVelocity() {
    double turretVelocity = turretMotor.getVelocity().getValueAsDouble();
    return (turretVelocity);
  }

  public void spinPositive() {
    // turretMotor.set(TurretConstants.turret_speed);
    target += 10.0;
  }

  public void spinNegative() {
    turretMotor.set(-TurretConstants.turret_speed);
    target -= 10.0;
  }

  public void turretMoveTgt(){
    double tgt = (0.0); // angle
    target = tgt;
  }

  public void stop() {
    turretMotor.set(0);
  }

  public void defaultCommand() {
    // System.out.println("target is " + target);

    int index = 2;

    if (Math.abs(target) < 30.0)
    {
      index = 0;
    }
    else if (Math.abs(target) < 60.0)
    {
      index = 1;
    } 

    if (target > TurretConstants.kMaximumAngle) {
      turretMotor.setControl(m_request.withPosition(TurretConstants.kMaximumAngle / 360.0 * TurretConstants.kGearRatio).withSlot(index));
    }
    else if (target < TurretConstants.kMinimumAngle) {
      turretMotor.setControl(m_request.withPosition(TurretConstants.kMinimumAngle / 360.0 * TurretConstants.kGearRatio).withSlot(index));
    }
    else {
      turretMotor.setControl(m_request.withPosition(target / 360.0 * TurretConstants.kGearRatio).withSlot(index));
    }
  }

  public Command initDefaultCommand(Turret turret) {
    return Commands.runOnce(() -> defaultCommand(), this);
  }
}