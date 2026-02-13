// // // Copyright (c) FIRST and other WPILib contributors.
// // // Open Source Software; you can modify and/or share it under the terms of
// // // the WPILib BSD license file in the root directory of this project.
// package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.Degrees;
// import static edu.wpi.first.units.Units.Rotations;

// import static edu.wpi.first.units.Units.Degree;

// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.controls.PositionDutyCycle;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.units.AngleUnit;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.util.sendable.Sendable;
// import edu.wpi.first.util.sendable.SendableBuilder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.util.sendable.Sendable;
// import edu.wpi.first.util.sendable.SendableBuilder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Utils;
// import frc.robot.Telemetry;
// import frc.robot.Utils;
// import frc.robot.Constants.HoodConstants;
// import frc.robot.Constants.ShooterConstants;

// public class Hood extends SubsystemBase {
//   private final TalonFX hoodMotor;
//   private final CANcoder hoodEncoder;
//   private final TalonFXConfiguration hoodMotorConfiguration;
//   private final MotionMagicVoltage request = new MotionMagicVoltage(0).withSlot(0);

//   public double targetHoodPosition = HoodConstants.kMinimumEncoderPos;

//   public Hood() {
//     hoodMotor = new TalonFX(HoodConstants.kHoodMotorId);
//     hoodEncoder = new CANcoder(HoodConstants.kHoodEncoderId);
//     hoodMotorConfiguration = new TalonFXConfiguration();

//     hoodMotor.setPosition(0);

//     // Current limiting
//     CurrentLimitsConfigs hoodCurrent = new CurrentLimitsConfigs();
//     hoodCurrent.StatorCurrentLimit = HoodConstants.kHoodCurrentLimit;
//     hoodCurrent.StatorCurrentLimitEnable = true;
//     hoodMotor.getConfigurator().apply(hoodCurrent);

//     // PID + Gravity
//     hoodMotorConfiguration.Slot0.kP = HoodConstants.hood_kP;
//     hoodMotorConfiguration.Slot0.kI = HoodConstants.hood_kI;
//     hoodMotorConfiguration.Slot0.kD = HoodConstants.hood_kD;

//     hoodMotorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;

//     hoodMotorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;

//     // Motion Magic
//     hoodMotorConfiguration.Voltage.PeakForwardVoltage = HoodConstants.hood_maxVoltage;
//     hoodMotorConfiguration.Voltage.PeakReverseVoltage = -HoodConstants.hood_maxVoltage;
//     hoodMotorConfiguration.MotionMagic.MotionMagicAcceleration = HoodConstants.hood_maxAcceleration;
//     hoodMotorConfiguration.MotionMagic.MotionMagicCruiseVelocity = HoodConstants.hood_maxVelocity;

//     // set to brake mode to stop the motor within the deadband
//     hoodMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

//     // hoodMotorConfiguration.Feedback.FeedbackRemoteSensorID = HoodConstants.kHoodEncoderId;
//     // hoodMotorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
//     // hoodMotorConfiguration.Feedback.RotorToSensorRatio = 1; // 1 motor rotation per 1 encoder rotation
   
//     hoodMotor.getConfigurator().apply(hoodMotorConfiguration);

//     SmartDashboard.putData("Hood", new Sendable() {
//         @Override
//         public void initSendable(SendableBuilder builder) {
//             builder.addDoubleProperty("Velocity", () -> hoodMotor.getVelocity().getValueAsDouble(), null);
//             builder.addDoubleProperty("Absolute Encoder Position", () -> (hoodEncoder.getAbsolutePosition().getValueAsDouble()), (double val) -> hoodEncoder.setPosition(val));
//             builder.addDoubleProperty("Motor Encoder Position", () -> (hoodMotor.getPosition().getValueAsDouble()), (double val) -> hoodEncoder.setPosition(val));
//             builder.addDoubleProperty("Target Hood Position", () -> targetHoodPosition, (double val) -> targetHoodPosition = val);
//         }
//     });
//   }

//   @Override
//   public void periodic() {
//     // keep the hood motor in sync with the hood encoder absolute position;
//     // MotionMagic will NOT work if you tell the motor that it has
//     // a remote CANcoder on it (in my testing)
//     // hoodMotor.setPosition(hoodEncoder.getAbsolutePosition().getValueAsDouble());
//     //moveHoodWithEncoder(targetHoodPosition);
//     // System.out.println("Hood motor rotations: " + hoodMotor.getPosition().getValueAsDouble());
//   }

//   //motion magic

//   private double hoodRotationsToMotor(double hoodRotations) {
//     return hoodRotations * HoodConstants.kGearRatio;
//   }

//   public void moveHoodMotionMagic() {
//     // double motorTarget = hoodRotationsToMotor(hoodRotations);
//     clampTarget();

//     System.out.println("moving to target! " + targetHoodPosition);
//     hoodMotor.setControl(request.withPosition(targetHoodPosition));
//   }

//   public void moveHoodToAngle(Angle angle) {
//     double degrees = angle.in(Degree);

//     double hoodRangeDeg = HoodConstants.kMaximumAngle - HoodConstants.kMinimumAngle;
//     double hoodEncoderRange = HoodConstants.kMaximumEncoderPos - HoodConstants.kMinimumEncoderPos;

//     double positionRatio = degrees / hoodRangeDeg;
//     double position = HoodConstants.kMinimumEncoderPos + (hoodEncoderRange * positionRatio);
    
//     targetHoodPosition = position;

//     clampTarget();
//     moveHoodMotionMagic();
//   }

//   public void moveHoodWithEncoder(double rotation) {
//     rotation = Utils.clamp(rotation, HoodConstants.kMinimumEncoderPos, HoodConstants.kMaximumEncoderPos);
//     // fix `Resource leak: 'pidController' is never closed`
//     try (PIDController pidController = new PIDController(HoodConstants.hood_kP, HoodConstants.hood_kI, HoodConstants.hood_kD)) {
//       double feedback = pidController.calculate(hoodEncoder.getPosition().getValueAsDouble(), rotation);

//       double error = rotation - (hoodEncoder.getPosition().getValueAsDouble());

//       if (Math.abs(error) < 0.05) {
//         hoodMotor.set(0);
//       } else {
//         hoodMotor.set(feedback);
//       }
//     }
//   }

//   // POV UP move hood to -0.75
//   public Command moveHoodToTgtCmd() {
//     // return Commands.runOnce(() -> moveHoodMotionMagic(-27)); 
//     return Commands.runOnce(() -> {});
//   }

//   public Command stopHoodCmd() {
//     hoodMotor.set(0);
//     return Commands.runOnce(() -> {});
//   }

  
//   public void clampTarget() {
//     targetHoodPosition = Utils.clamp(targetHoodPosition, HoodConstants.kMinimumEncoderPos, HoodConstants.kMaximumEncoderPos);
//   }

//   public void runHood() {
//     // targetHoodPosition += HoodConstants.kHoodSpeed;
//     targetHoodPosition = 0.4;
//     moveHoodMotionMagic();
//   }


//   public void runHoodReverse() {
//     targetHoodPosition -= HoodConstants.kHoodSpeed;
//     hoodMotor.set(-HoodConstants.kHoodSpeed);
//     moveHoodMotionMagic();
//   }

//   public double GetHoodAngle()
//   {
//     return 0; // TEMP VALUE
//   }
// }

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Limelight;
import frc.robot.Telemetry;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.TurretConstants;

public class Hood extends SubsystemBase {

  private final TalonFX hoodMotor;
  private final TalonFXConfiguration hoodMotorConfig;

  final MotionMagicVoltage m_request = new MotionMagicVoltage(0.0);

  double tgt = 0.0;

  /** Creates a new Hood. */
  public Hood() {
    hoodMotor = new TalonFX(HoodConstants.kHoodMotorId);
    hoodMotorConfig = new TalonFXConfiguration();
    hoodMotor.setPosition(0);

    hoodMotorConfig.Slot0.kS = HoodConstants.hood_kS;
    hoodMotorConfig.Slot0.kV = HoodConstants.hood_kV;
    hoodMotorConfig.Slot0.kA = HoodConstants.hood_kA;
    hoodMotorConfig.Slot0.kP = HoodConstants.hood_kP;
    hoodMotorConfig.Slot0.kI = HoodConstants.hood_kI;
    hoodMotorConfig.Slot0.kD = HoodConstants.hood_kD;

    hoodMotorConfig.CurrentLimits.StatorCurrentLimit = HoodConstants.kHoodCurrentLimit;
    hoodMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    hoodMotorConfig.Voltage.PeakForwardVoltage = HoodConstants.hood_maxVoltage;
    hoodMotorConfig.Voltage.PeakReverseVoltage = -HoodConstants.hood_maxVoltage;
    hoodMotorConfig.MotionMagic.MotionMagicAcceleration = HoodConstants.hood_maxAcceleration;
    hoodMotorConfig.MotionMagic.MotionMagicCruiseVelocity = HoodConstants.hood_maxVelocity;

    hoodMotor.getConfigurator().apply(hoodMotorConfig);

    Telemetry.telemeterizeMotorWithPID("Hood", hoodMotor, (15.0 / 210.0), hoodMotorConfig);


  }

  @Override
  public void periodic() {
    System.out.println(hoodMotor.getPosition().getValueAsDouble());
  }

  public double getPosition() {
    double position = hoodMotor.getPosition().getValueAsDouble() / HoodConstants.kGearRatio;
    return position * 360; 
  }

  public double getHoodCurrent() {
    double hoodCurrent = hoodMotor.getSupplyCurrent().getValueAsDouble();
    return hoodCurrent;
  }

  public Command hoodPos () {
      return Commands.runOnce(() -> spinPositive());
    }

  public Command hoodNeg () {
      return Commands.runOnce(() -> spinNegative());
    }


  public double getHoodAngle() {
    double position = getPosition(); // turns
    return position * 360;
  }

  public void spinPositive() {
    hoodMotor.set(HoodConstants.kHoodSpeed);
  }

  public void spinNegative() {
    hoodMotor.set(-HoodConstants.kHoodSpeed);
  }

  public void hoodMoveTgt(){

    // double tgt = (-20.0 * HoodConstants.kGearRatio) / 360.0;
    // double tgt = -1.0 * 2.209277238403452;
    // double tgt = -0.5;// * 2.209277238403452;
    // double tgt = -0.75;
    // tgt = -0.3;

    
    hoodMotor.setControl(m_request.withPosition(tgt)); //motor rotations
    
  }

  public void setHoodTgt(double input)
  {
    tgt = input;
  }

  public void stop() {
    hoodMotor.set(0);
  }

}