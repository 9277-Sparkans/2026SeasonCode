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

public class Turret extends SubsystemBase {

  private final TalonFX turretMotor;
  private final TalonFXConfiguration turretMotorConfig;
  public double turretOffset = 0.0;

  public double targetHoodAngle = HoodConstants.kMaximumAngle;

  public double currentTargetTurret = 0;

  public boolean positivePressed = false;
  public boolean negativePressed = false;

  /** Creates a new Turret. */
  public Turret() {
    turretMotor = new TalonFX(TurretConstants.turret_motorId);
    turretMotorConfig = new TalonFXConfiguration(); 
    SoftwareLimitSwitchConfigs softwareLimitConfigs = new SoftwareLimitSwitchConfigs();
    turretMotor.setPosition(0);

    softwareLimitConfigs.ForwardSoftLimitThreshold = TurretConstants.kMaximumAngle;
    softwareLimitConfigs.ReverseSoftLimitThreshold = TurretConstants.kMinimumAngle;
    softwareLimitConfigs.ForwardSoftLimitEnable = true;
    softwareLimitConfigs.ReverseSoftLimitEnable = true;
    turretMotor.getConfigurator().apply(softwareLimitConfigs);

    turretMotorConfig.Slot0.kG = TurretConstants.turret_kG;
    turretMotorConfig.Slot0.kP = TurretConstants.turret_kP;
    turretMotorConfig.Slot0.kI = TurretConstants.turret_kI;
    turretMotorConfig.Slot0.kD = TurretConstants.turret_kD;

    turretMotorConfig.Voltage.PeakForwardVoltage = TurretConstants.turret_maxVoltage;
    turretMotorConfig.Voltage.PeakReverseVoltage = -TurretConstants.turret_maxVoltage;
    turretMotorConfig.MotionMagic.MotionMagicAcceleration = TurretConstants.turret_maxAcceleration;
    turretMotorConfig.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.turret_maxVelocity;

    turretMotor.getConfigurator().apply(turretMotorConfig);


    Telemetry.telemeterizeMotor("Turret", turretMotor, (1.0 / (15.0 / 108.0)));

  }

  @Override
  public void periodic() {
    System.out.println("Is positive: " + positivePressed);
    System.out.println("Is negative: " + negativePressed);
    System.out.println("is position: " + getTurretAngle());

    if (positivePressed)
    {
      // 30 degrees per second
      currentTargetTurret += 0.02 * 30; 

      if (currentTargetTurret >= TurretConstants.kMaximumAngle - 2.0)
      {
        currentTargetTurret = TurretConstants.kMaximumAngle - 2.0;
      }

      // if (getTurretAngle() <= TurretConstants.kMaximumAngle - 2.0) 
      // {
      //   stop();
      // }
      // else
      // {
      //   spinPositive();
      // }
    }
    else if (negativePressed)
    {
      // 30 degrees per second
      currentTargetTurret -= 0.02 * 30; 

      if (currentTargetTurret <= TurretConstants.kMinimumAngle + 2.0)
      {
        currentTargetTurret = TurretConstants.kMinimumAngle + 2.0;
      }

      // if (getTurretAngle() >= TurretConstants.kMinimumAngle + 2.0) 
      // {
      //   stop();
      // }
      // else
      // {
      //   spinNegative();
      // } 
    }

    setTurretToAngle(currentTargetTurret);
  }

  public double getPosition() {
    double position = turretMotor.getPosition().getValueAsDouble() / TurretConstants.kGearRatio;
    return position * 360; 
  }

  public double getTurretCurrent() {
    double turretCurrent = turretMotor.getSupplyCurrent().getValueAsDouble() / TurretConstants.kGearRatio;
    return (turretCurrent);
  }

  // public Command turretPos () {
  //   posi
  //   return Commands.runOnce(() -> {});
  //   // if (currentTurretAngle >= TurretConstants.kMaximumAngle - 2.0) {
  //   //   return Commands.runOnce(() -> stop());
  //   // }
  //   // else {
  //   //   return Commands.runOnce(() -> spinPositive());
  //   // }
  // }

  // public Command turretNeg () {
  //   if (currentTurretAngle <= TurretConstants.kMinimumAngle + 2.0) {
  //     return Commands.runOnce(() -> stop());
  //   }
  //   else {
  //     return Commands.runOnce(() -> spinNegative());
  //   }
  //   }

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

  
  public void setTurretToAngle(double angle)
  {
    // angle = targetHoodAngle;
    // if (angle > TurretConstants.kMaximumAngle)
    // {
    //   angle = TurretConstants.kMaximumAngle;
    // }
    // if (angle < TurretConstants.kMinimumAngle)
    // {
    //   angle = TurretConstants.kMinimumAngle;
    // }

    MotionMagicVoltage angleTgt = new MotionMagicVoltage(angle).withSlot(0);
    turretMotor.setControl(angleTgt.withPosition(getTurretAngle()));
  }

  public void PositiveTrue()
  {
    positivePressed = true;
  }

  public void PositiveFalse()
  {
    positivePressed = false;
  }

  public void NegativeTrue()
  {
    negativePressed = true;
  }

  public void NegativeFalse()
  {
    negativePressed = false;
  } 

  public void stop() {
    turretMotor.set(0);
  }
}