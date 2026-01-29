// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import java.nio.channels.SelectableChannel;

// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.ShooterConstants;
// import frc.robot.Limelight;
// import frc.robot.Constants.HoodConstants;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.controller.PIDController;

// public class Hood extends SubsystemBase {
//   public double tgtAngle = 0;
//   private final TalonFX hoodMotor;
//   private final TalonFXConfiguration hoodMotorConfiguration;
// // -33
//   public Hood() {
//     hoodMotor = new TalonFX(HoodConstants.kHoodMotorId);
//     hoodMotorConfiguration = new TalonFXConfiguration();
//     hoodMotor.setPosition(0);

//     CurrentLimitsConfigs hoodConfigs = new CurrentLimitsConfigs();
//     hoodConfigs.StatorCurrentLimit = HoodConstants.kHoodCurrentLimit;
//     hoodConfigs.StatorCurrentLimitEnable = true;
//     hoodMotor.getConfigurator().apply(hoodConfigs);

//     hoodMotorConfiguration.Slot0.kG = HoodConstants.hood_kG;
//     hoodMotorConfiguration.Slot0.kP = HoodConstants.hood_kP;
//     hoodMotorConfiguration.Slot0.kI = HoodConstants.hood_kI;
//     hoodMotorConfiguration.Slot0.kD = HoodConstants.hood_kD; 

//     hoodMotorConfiguration.Voltage.PeakForwardVoltage = HoodConstants.hood_maxVoltage;
//     hoodMotorConfiguration.Voltage.PeakReverseVoltage = -HoodConstants.hood_maxVoltage;
//     hoodMotorConfiguration.MotionMagic.MotionMagicAcceleration = HoodConstants.hood_maxAcceleration;
//     hoodMotorConfiguration.MotionMagic.MotionMagicCruiseVelocity = HoodConstants.hood_maxVelocity;

//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     System.out.println("Hood Angle: " + GetCurrentHoodAngle());
//     System.out.println("Target angle: " + tgtAngle);
//     System.out.println("Encoder position: " + hoodMotor.getPosition());
//     //System.out.println("Target hood angle: " + GetTargetHoodAngle());
//   }

  // public Command runHoodCmd() {
  //       return Commands.runOnce(() -> runHood());
  //   }

  // public Command runHoodReverseCmd() {
  //       return Commands.runOnce(() -> runHoodReverse());
  //   }

  // public Command stopHoodCmd() {
  //       return Commands.runOnce(() -> stopHood());
  //   }
  //   public Command moveHoodUpCmd(){
  //     if (GetCurrentHoodAngle() < HoodConstants.kMaximumAngle - 0.5)
  //     {
  //       runHood();
  //     }
  //     else
  //     {
  //       stopHood();
  //     }
  //     return Commands.runOnce(() -> {});
  //   }
  

  //   public Command moveHoodDownCmd(){
  //     if (GetCurrentHoodAngle() > HoodConstants.kMinimumAngle + 0.5)
  //     {
  //       runHoodReverse();
  //     }
  //     else
  //     {
  //       stopHood();
  //     }
  //     return Commands.runOnce(() -> {});
  //   }

//     // hood
//     public void moveHoodToAngle(double theta){
//       theta = tgtAngle;

//       PIDController pidController = new PIDController(HoodConstants.hood_kP, HoodConstants.hood_kI, HoodConstants.hood_kD);

//       double normalizedAngle = (HoodConstants.kMaximumAngle - HoodConstants.kMinimumAngle) / theta;
//       double targetPosition = normalizedAngle * HoodConstants.maxEncoderValue;

//       System.out.println("target position: " + targetPosition);

//       double feedbackVoltage = pidController.calculate(hoodMotor.getPosition().getValueAsDouble(), targetPosition);
//       hoodMotor.setVoltage(feedbackVoltage/500);
//     }
    
//     public void runHood() {
//         hoodMotor.set(HoodConstants.kHoodSpeed);
//     }

//     public void runHoodReverse() {
//         hoodMotor.set(-HoodConstants.kHoodSpeed);
//     }

//     public void stopHood() {
//         hoodMotor.set(0);
//     }

//     public void moveHoodDown()
//     {
//         if (tgtAngle - HoodConstants.kHoodIncrement >= 0.5)
//         {
//             tgtAngle -= HoodConstants.kHoodIncrement;
//         }
//     }

//     public void moveHoodUp()
//     {
//         if (tgtAngle + HoodConstants.kHoodIncrement <= HoodConstants.kMaximumAngle - HoodConstants.kMinimumAngle - 0.5)
//         {
//             tgtAngle += HoodConstants.kHoodIncrement;
//         }
//     }

//     public double GetTargetHoodAngle()
//     {
//         Translation2d position = Limelight.GetDistance();
//         double distance = Math.sqrt(position.getX() * position.getX() + position.getY() * position.getY());

//         // field length 158.6  inches or 4.02844 meters
//         double angle = (distance / 4.02844) * (HoodConstants.kMaximumAngle - HoodConstants.kMinimumAngle);

//         return HoodConstants.kMinimumAngle + angle; // replace with actual math later
//     }

//     public double GetCurrentHoodAngle()
//     {
//         double currentHoodPosition = hoodMotor.getPosition().getValueAsDouble(); // rotations
//         double hoodSpace = currentHoodPosition / HoodConstants.kGearRatio;
//         double hoodAngle = hoodSpace * 360;

//         return Math.max(HoodConstants.kMaximumAngle + hoodAngle, HoodConstants.kMinimumAngle);
//     }
// }
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;
import frc.robot.Telemetry;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;

public class Hood extends SubsystemBase {

  private final TalonFX hoodMotor;
  private final CANcoder hoodEncoder;
  private final TalonFXConfiguration hoodMotorConfiguration;

  public Hood() {
    hoodMotor = new TalonFX(HoodConstants.kHoodMotorId);
    hoodEncoder = new CANcoder(HoodConstants.kHoodEncoderId);
    hoodMotorConfiguration = new TalonFXConfiguration();

    // hoodEncoder.setPosition(0);

    // Current limiting
    CurrentLimitsConfigs hoodCurrent = new CurrentLimitsConfigs();
    hoodCurrent.StatorCurrentLimit = HoodConstants.kHoodCurrentLimit;
    hoodCurrent.StatorCurrentLimitEnable = true;
    hoodMotor.getConfigurator().apply(hoodCurrent);

    // PID + Gravity
    hoodMotorConfiguration.Slot0.kG = HoodConstants.hood_kG;
    hoodMotorConfiguration.Slot0.kP = HoodConstants.hood_kP;
    hoodMotorConfiguration.Slot0.kI = HoodConstants.hood_kI;
    hoodMotorConfiguration.Slot0.kD = HoodConstants.hood_kD;

    hoodMotorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;

    // Motion Magic
    hoodMotorConfiguration.Voltage.PeakForwardVoltage = HoodConstants.hood_maxVoltage;
    hoodMotorConfiguration.Voltage.PeakReverseVoltage = -HoodConstants.hood_maxVoltage;
    hoodMotorConfiguration.MotionMagic.MotionMagicAcceleration = HoodConstants.hood_maxAcceleration;
    hoodMotorConfiguration.MotionMagic.MotionMagicCruiseVelocity = HoodConstants.hood_maxVelocity;

    // set to brake mode to stop the motor within the deadband
    hoodMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    hoodMotorConfiguration.Feedback.FeedbackRemoteSensorID = HoodConstants.kHoodEncoderId;
    hoodMotorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    hoodMotorConfiguration.Feedback.RotorToSensorRatio = 1; // 1 motor rotation per 1 encoder rotation
   
    hoodMotor.getConfigurator().apply(hoodMotorConfiguration);

    // Telemetry.telemeterizeMotor("Hood", hoodMotor);
    SmartDashboard.putData("Hood", new Sendable() {
        @Override
        public void initSendable(SendableBuilder builder) {
            builder.addDoubleProperty("Velocity", () -> hoodMotor.getVelocity().getValueAsDouble(), null);
            builder.addDoubleProperty("Position", () -> (hoodEncoder.getPosition().getValueAsDouble()), (double val) -> hoodEncoder.setPosition(val));
        }
    });
  }

  @Override
  public void periodic() {
    moveHoodWithEncoder(0);
    // System.out.println("Hood motor rotations: " + hoodMotor.getPosition().getValueAsDouble());
  }

  

  //motion magic

  private double hoodRotationsToMotor(double hoodRotations) {
    return hoodRotations * HoodConstants.kGearRatio;
  }

  public void moveHoodMotionMagic(double hoodRotations) {
    // double motorTarget = hoodRotationsToMotor(hoodRotations);

    MotionMagicVoltage request =
        new MotionMagicVoltage(0)
            .withSlot(0)
            .withPosition(hoodRotations);

    hoodMotor.setControl(request);
  }


  public void moveHoodWithEncoder(double rotation) {
    rotation = 0.25;
    PIDController pidController = new PIDController(HoodConstants.hood_kP, HoodConstants.hood_kI, HoodConstants.hood_kD);

    double feedback = pidController.calculate(hoodEncoder.getPosition().getValueAsDouble(), rotation);

    double error = rotation - (hoodEncoder.getPosition().getValueAsDouble());

    System.out.println("error: " + error);
    System.out.println("encoder position: " + hoodEncoder.getPosition().getValueAsDouble());
    System.out.println("feedback: " + feedback);

    if (Math.abs(error) < 0.05)
    {
      hoodMotor.set(0);
    }
    else
    {
      hoodMotor.set(feedback);
    }
  }

  // POV UP move hood to -0.75
  public Command moveHoodToTgtCmd() {
    return Commands.runOnce(() -> moveHoodMotionMagic(-27)); 
  }

  public Command stopHoodCmd() {
    //return Commands.runOnce(() -> hoodMotor.set(0));
    hoodMotor.set(0);
    return Commands.runOnce(() -> {});
  }

  

  public void runHood() {
    hoodMotor.set(HoodConstants.kHoodSpeed);
  }

  public void runHoodReverse() {
    hoodMotor.set(-HoodConstants.kHoodSpeed);
  }

}