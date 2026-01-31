package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.QuickAccessConstants;
import frc.robot.Constants.TurretConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import frc.robot.Limelight;

public class Turret extends SubsystemBase {

  private final TalonFX turretMotor;
  private final TalonFXConfiguration turretMotorConfig;
  public double turretOffset = 0.0;

  public double targetHoodAngle = HoodConstants.kMaximumAngle;


  /** Creates a new Turret. */
  public Turret() {
    turretMotor = new TalonFX(TurretConstants.turret_motorId);
    turretMotorConfig = new TalonFXConfiguration();
    // SoftwareLimitSwitchConfigs softwareLimitConfigs = new SoftwareLimitSwitchConfigs();
    turretMotor.setPosition(0);




    // softwareLimitConfigs.ForwardSoftLimitThreshold = TurretConstants.kMaximumAngle;
    // softwareLimitConfigs.ReverseSoftLimitThreshold = TurretConstants.kMinimumAngle;
    // softwareLimitConfigs.ForwardSoftLimitEnable = true;
    // softwareLimitConfigs.ReverseSoftLimitEnable = true;
    // turretMotor.getConfigurator().apply(softwareLimitConfigs);

    turretMotorConfig.Slot0.kS = TurretConstants.turret_kS;
    turretMotorConfig.Slot0.kV = TurretConstants.turret_kV;
    turretMotorConfig.Slot0.kA = TurretConstants.turret_kA;
    turretMotorConfig.Slot0.kP = TurretConstants.turret_kP;
    turretMotorConfig.Slot0.kI = TurretConstants.turret_kI;
    turretMotorConfig.Slot0.kD = TurretConstants.turret_kD;
    turretMotorConfig.Slot0.kG = TurretConstants.turret_kG;


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
    double turretCurrent = turretMotor.getSupplyCurrent().getValueAsDouble() / TurretConstants.kGearRatio;
    return (turretCurrent);
  }

  public Command turretPos () {
      return Commands.runOnce(() -> spinPositive());
    }

  public Command turretNeg () {
      return Commands.runOnce(() -> spinNegative());
    }


  public double getTurretAngle()
  {
    double position = getTurretCurrent(); // turns
    return position * 360;
  }

  public double getVelocity() {
    double turretVelocity = turretMotor.getVelocity().getValueAsDouble();
    return (turretVelocity);
  }

  public void spinPositive(){
      turretMotor.set(TurretConstants.turret_speed);
  }

  public void spinNegative(){
      turretMotor.set(-TurretConstants.turret_speed);
  }

  public void turretMoveTgt(double llAngle){
    double tgt = (-llAngle * TurretConstants.kGearRatio) / 360;

    final MotionMagicVoltage m_request = new MotionMagicVoltage(tgt);

    turretMotor.setControl(m_request); //motor rotations
    //System.out.println(tgt);
  }


  public void stop() {
    turretMotor.set(0);
  }

}