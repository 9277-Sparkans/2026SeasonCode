// // // Copyright (c) FIRST and other WPILib contributors.
// // // Open Source Software; you can modify and/or share it under the terms of
// // // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Telemetry;
import frc.robot.Utils;
import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {
  private final TalonFX hoodMotor;
  private final CANcoder hoodEncoder;
  private final TalonFXConfiguration hoodMotorConfiguration;

  public double targetHoodPosition = -0.25d;
  public double targetHoodAngle = 5;
  public double convertedHoodPos = -1d;

  private final MotionMagicVoltage request = new MotionMagicVoltage(0.0);

  private final VoltageOut sysIdControl = new VoltageOut(0);
  private final SysIdRoutine sysIdRoutine;

  /** Creates a new Hood. */
  public Hood() {
    hoodMotor = new TalonFX(HoodConstants.kHoodMotorId);
    hoodEncoder = new CANcoder(HoodConstants.kHoodEncoderId);
    hoodMotorConfiguration = new TalonFXConfiguration();

    // hoodMotor.setPosition(hoodEncoder.getAbsolutePosition().getValueAsDouble());

    // Current limiting
    CurrentLimitsConfigs hoodCurrent = new CurrentLimitsConfigs();
    hoodCurrent.StatorCurrentLimit = HoodConstants.kHoodCurrentLimit;
    hoodCurrent.StatorCurrentLimitEnable = true;
    hoodMotor.getConfigurator().apply(hoodCurrent);

    // PID + Gravity
    hoodMotorConfiguration.Slot0.kG = HoodConstants.hood_kG;
    hoodMotorConfiguration.Slot0.kS = HoodConstants.hood_kS;
    hoodMotorConfiguration.Slot0.kV = HoodConstants.hood_kV;
    hoodMotorConfiguration.Slot0.kA = HoodConstants.hood_kA;
    hoodMotorConfiguration.Slot0.kP = HoodConstants.hood_kP;
    hoodMotorConfiguration.Slot0.kI = HoodConstants.hood_kI;
    hoodMotorConfiguration.Slot0.kD = HoodConstants.hood_kD;

    hoodMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    hoodMotorConfiguration.Voltage.PeakForwardVoltage = HoodConstants.hood_maxVoltage;
    hoodMotorConfiguration.Voltage.PeakReverseVoltage = -HoodConstants.hood_maxVoltage;
    hoodMotorConfiguration.MotionMagic.MotionMagicAcceleration = HoodConstants.hood_maxAcceleration;
    hoodMotorConfiguration.MotionMagic.MotionMagicCruiseVelocity = HoodConstants.hood_maxVelocity;

    hoodMotor.getConfigurator().apply(hoodMotorConfiguration);

    // hoodMotorConfiguration.Feedback.FeedbackRemoteSensorID = hoodEncoder.getDeviceID();
    // hoodMotorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    // hoodMotorConfiguration.Feedback.RotorToSensorRatio = 1; // 1 motor rotation per 1 encoder rotation
    // hoodMotorConfiguration.Feedback.SensorToMechanismRatio = ;
   
    SmartDashboard.putData("Hood", new Sendable() {
        @Override
        public void initSendable(SendableBuilder builder) {
            builder.addDoubleProperty("Velocity", () -> hoodMotor.getVelocity().getValueAsDouble(), null);
            builder.addDoubleProperty("Absolute Encoder Position", () -> (hoodEncoder.getAbsolutePosition().getValueAsDouble()), (double val) -> hoodEncoder.setPosition(val));
            builder.addDoubleProperty("Motor Encoder Position", () -> (hoodMotor.getPosition().getValueAsDouble()), (double val) -> hoodMotor.setPosition(val));
            builder.addDoubleProperty("Target Hood Position", () -> targetHoodPosition, (double val) -> targetHoodPosition = val);
            builder.addDoubleProperty("Target Angle Position", () -> targetHoodAngle, (double val) -> targetHoodAngle = val);
            builder.addDoubleProperty("Supposed Hood Position", () -> convertedHoodPos, null);
        }
    });

    // SmartDashboard.putData(hoodMotor);

    Telemetry.telemeterizeMotorWithPID("Hood (PID)", hoodMotor, 1);

    sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
        Volts.of(0.1).per(Second) , // Quasi - increases by 0.1 V per sec
        Volts.of(0.65), // Dynamic - jumps to 0.5 V
        Seconds.of(10) // maxes at 10s
      ),
      new SysIdRoutine.Mechanism (
        (Voltage volts) -> {
          hoodMotor.setControl(sysIdControl.withOutput(volts.in(Volts)));
        },
        (SysIdRoutineLog log) -> {
          log.motor("Hood-Motor")
            .voltage(Volts.of(hoodMotor.getMotorVoltage().
              getValueAsDouble()))
            .angularPosition (Rotations.of(hoodMotor.getPosition().
              getValueAsDouble()))
            .angularVelocity(RotationsPerSecond.of(hoodMotor.
              getVelocity().getValueAsDouble()));
        },
        this
      )
    );
  }

  @Override
  public void periodic() {
    // System.out.println(hoodMotor.getPosition().getValueAsDouble());
    // hoodMotor.set(0.0);
  }

  public double getPosition() {
    double position = hoodMotor.getPosition().getValueAsDouble() / HoodConstants.kGearRatio;
    return position * 360; 
  }

  public void moveHoodMotionMagic() {
    // double motorTarget = hoodRotationsToMotor(hoodRotations);
    // clampTarget();

    System.out.println("target: " + targetHoodPosition);
    System.out.println("idk man constant: " + HoodConstants.kIdkManConstant);
    convertedHoodPos = HoodConstants.kIdkManConstant * targetHoodPosition;
    System.out.println("theoretical absolute position: " + convertedHoodPos);
    hoodMotor.setControl(request.withPosition(targetHoodPosition));
  }

  public void moveHoodToAngle(double degrees) {
    // double hoodRangeDeg = HoodConstants.kMaximumAngle - HoodConstants.kMinimumAngle;
    // double hoodEncoderRange = HoodConstants.kMaximumEncoderPos - HoodConstants.kMinimumEncoderPos;

    // double positionRatio = degrees / hoodRangeDeg;
    // double position = HoodConstants.kMinimumEncoderPos + (hoodEncoderRange * positionRatio);

    targetHoodPosition = -((degrees + 1) / 360.f) * HoodConstants.kGearRatio;
    System.out.println("target angle to hood: " + targetHoodPosition);
    System.out.println("target degrees to hood: " + degrees);

    // clampTarget();
    moveHoodMotionMagic();
  }

  // POV UP move hood to -0.75
  public Command moveHoodToTgtCmd() {
    // return Commands.runOnce(() -> moveHoodMotionMagic(-27)); 
    return Commands.runOnce(() -> {});
  }

  public Command stopHoodCmd() {
    hoodMotor.set(0);
    return Commands.runOnce(() -> {});
  }

  
  public void clampTarget() {
    targetHoodPosition = Utils.clamp(targetHoodPosition, HoodConstants.kMinimumEncoderPos, HoodConstants.kMaximumEncoderPos);
  }

  public void runHood() {
    // targetHoodPosition += HoodConstants.kHoodSpeed;
    targetHoodPosition = 0.4;
    hoodMotor.set(HoodConstants.kHoodSpeed);
    // moveHoodMotionMagic();
  }


  public void runHoodReverse() {
    // targetHoodPosition -= HoodConstants.kHoodSpeed;
    hoodMotor.set(-HoodConstants.kHoodSpeed);
    // moveHoodMotionMagic();
  }

  public Command sysIdQuasistatic ( SysIdRoutine . Direction direction ) {
    return sysIdRoutine . quasistatic ( direction ) ;
  }
 
  public Command sysIdDynamic ( SysIdRoutine . Direction direction ) {
    return sysIdRoutine . dynamic ( direction ) ;
  }

}