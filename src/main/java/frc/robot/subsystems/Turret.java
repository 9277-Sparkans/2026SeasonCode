package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {

  private final TalonFX turretMotor;
  private final TalonFXConfiguration turretMotorConfig;
  // private GenericEntry sb_turretEncoder;
  // public double turretOffset = 0.0;

  public Command turretPos() {
    spinPositive(); // okay so this works, Commands.runOnce throws a tantrum
    return Commands.runOnce(() -> {
    });
  }

  public Command turretNeg() {
    spinNegative(); // okay so this works, Commands.runOnce throws a tantrum
    return Commands.runOnce(() -> {
    });
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

    // sb_turretEncoder.setDouble(turretMotor.getPosition().getValue().magnitude());

    timer = new Timer();
  }

  @Override
  public void periodic() {
    // System.out.println("Turret Position: " + getPosition());
    // This method will be called once per scheduler run
    System.out.println(turretMotor.getSupplyCurrent().getValueAsDouble());
  }

  public double getPosition() {
    return turretMotor.getPosition().getValueAsDouble();
  }

  public double getVelocity() {
    double turretVelocity = turretMotor.getVelocity().getValueAsDouble();
    return (turretVelocity);
  }

  public void spinPositive() {
    turretMotor.set(0.15);
  }

  public void spinNegative() {
    turretMotor.set(-0.15);
  }

  public double getTurretRotations() {
    return turretMotor.getPosition().getValueAsDouble();
  }

  public double getTurretAngle() {
    double position = getTurretRotations(); // rotations
    // double turretRotations = position * TurretConstants.kGearRatio; // This seems
    // backwards based on constants.
    // In Constants: kGearRatio = 1.0 / (18.0 / 105.0) approx 5.83.
    // If motor spins 5.83 times, turret spins 1 time?
    // Usually gear ratio is Motor / Output.
    // If kGearRatio is 5.83, then motor rot = turret rot * 5.83.
    // So turret rot = motor rot / 5.83.
    // Wait, let's check the constant definition again.
    // kGearRatio = 1.0 / (18.0 / 105.0) = 1 / 0.1714 = 5.833
    // So if I have motor rotations, I should DIVIDE by kGearRatio to get turret
    // rotations?
    // Or did previous code multiply?
    // Old code: turretCurrent / kGearRatio.
    // Let's stick to logical deduction:
    // 18 teeth driving 105 -> reduction is 18/105 = 0.1714. output spins 0.1714 per
    // input.
    // So GearRatio should be 105/18 = 5.833 (Input per Output).
    // So MotorRotations / 5.833 = TurretRotations.
    // kGearRatio is defined as 5.833.
    // So TurretRotations = MotorRotations / kGearRatio.

    // In previous code:
    // double turretCurrent = turretMotor.getSupplyCurrent().getValueAsDouble() /
    // TurretConstants.kGearRatio;
    // return (turretCurrent);

    // So it was dividing. I will do the same.

    return (position / TurretConstants.kGearRatio) * 360;
  }

  public void setTurretToAngle(double angle) {
    if (angle > TurretConstants.kMaximumAngle) {
      angle = TurretConstants.kMaximumAngle;
    }
    if (angle < TurretConstants.kMinimumAngle) {
      angle = TurretConstants.kMinimumAngle;
    }

    // Convert angle to motor rotations
    // degrees / 360 = turret rotations
    // turret rotations * gear ratio = motor rotations
    double rotations = (angle / 360.0) * TurretConstants.kGearRatio;

    MotionMagicVoltage angleTgt = new MotionMagicVoltage(rotations).withSlot(0);
    turretMotor.setControl(angleTgt);
  }

  public void stop() {
    turretMotor.set(0);
  }
}