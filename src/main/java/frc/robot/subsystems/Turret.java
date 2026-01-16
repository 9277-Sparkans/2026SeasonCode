package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.QuickAccessConstants;
import frc.robot.Constants.TurretConstants;


public class Turret extends SubsystemBase {

  private final TalonFX turretMotor;
  private final TalonFXConfiguration turretMotorConfig;

  private boolean isBlue = false;

  public Command turretPos () {
      return Commands.runOnce(() -> spinPositive());
    }

    public Command turretNeg () {
      return Commands.runOnce(() -> spinNegative());
    }

  public Timer timer;

  /** Creates a new Turret. */
  public Turret() {

    isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;

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

    timer = new Timer();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getPosition() {
    return turretMotor.getPosition().getValueAsDouble();
  }

  public double getElevatorCurrent() {
    double turretCurrent = turretMotor.getSupplyCurrent().getValueAsDouble();
    return (turretCurrent);
  }


  public double getVelocity() {
    double turretVelocity = turretMotor.getVelocity().getValueAsDouble();
    return (turretVelocity);
  }

  

  public void spinPositive(){
    turretMotor.set(0.5);
  }

  public void spinNegative(){
    turretMotor.set(-0.5);
  }


  public void stop() {
    turretMotor.set(0);
  }

  public double GetTx()
  {
    Long id = NetworkTableInstance.getDefault().getTable("limelight").getEntry("id").getInteger(0);
    if (isBlue)
    {
      return (id == 26 || id == 25) ? NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0) : 0; 
    }
    else
    {
      return (id == 9 || id == 10) ? NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0) : 0; 
    }
  }
}