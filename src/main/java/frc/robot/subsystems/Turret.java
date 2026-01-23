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
import frc.robot.Constants.QuickAccessConstants;
import frc.robot.Constants.TurretConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Turret extends SubsystemBase {

  private final TalonFX turretMotor;
  private final TalonFXConfiguration turretMotorConfig;
  //private GenericEntry sb_turretEncoder;
  public double turretOffset = 0.0;

  public Command turretPos () {
      spinPositive(); // okay so this works, Commands.runOnce throws a tantrum
      return Commands.runOnce(() -> {});
    }

    public Command turretNeg () {
      spinNegative(); // okay so this works, Commands.runOnce throws a tantrum
      return Commands.runOnce(() -> {});
    }

  public Timer timer;

  /** Creates a new Turret. */
  public Turret() {
    turretMotor = new TalonFX(TurretConstants.turret_motorId);
    turretMotorConfig = new TalonFXConfiguration(); 
    turretMotor.setPosition(0);

    turretMotorConfig.Slot0.kG = TurretConstants.turret_kG;
    turretMotorConfig.Slot0.kP = TurretConstants.turret_kP;
    turretMotorConfig.Slot0.kI = TurretConstants.turret_kI;
    turretMotorConfig.Slot0.kD = TurretConstants.turret_kD;

    turretMotorConfig.Voltage.PeakForwardVoltage = TurretConstants.turret_maxVoltage;
    turretMotorConfig.Voltage.PeakReverseVoltage = -TurretConstants.turret_maxVoltage;
    turretMotorConfig.MotionMagic.MotionMagicAcceleration = TurretConstants.turret_maxAcceleration;
    turretMotorConfig.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.turret_maxVelocity;

    turretMotor.getConfigurator().apply(turretMotorConfig);

    //sb_turretEncoder.setDouble(turretMotor.getPosition().getValue().magnitude());


    timer = new Timer();
  }

  @Override
  public void periodic() {
    //System.out.println("Turret Position: " + getPosition());
    // This method will be called once per scheduler run
    System.out.println(turretMotor.getSupplyCurrent().getValueAsDouble());
  }

  public double getPosition() {
    return turretMotor.getPosition().getValueAsDouble();
  }

  public double getTurretCurrent() {
    double turretCurrent = turretMotor.getSupplyCurrent().getValueAsDouble() / TurretConstants.kGearRatio;
    return (turretCurrent);
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
    turretMotor.set(0.15);
  }

  public void spinNegative(){
    turretMotor.set(-0.15);
  }

  public void setTurretToAngle(double angle)
  {
    if (angle > TurretConstants.kMaximumAngle)
    {
      angle = TurretConstants.kMaximumAngle;
    }
    if (angle < TurretConstants.kMinimumAngle)
    {
      angle = TurretConstants.kMinimumAngle;
    }

    MotionMagicVoltage angleTgt = new MotionMagicVoltage(angle).withSlot(0);
    turretMotor.setControl(angleTgt.withPosition(getTurretAngle()));
  }


  public void stop() {
    turretMotor.set(0);
  }
}