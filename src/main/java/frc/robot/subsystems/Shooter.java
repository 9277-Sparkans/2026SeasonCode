// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// 

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static edu . wpi . first . units . Units . Rotations ;
import static edu . wpi . first . units . Units . RotationsPerSecond ;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class Shooter extends SubsystemBase {
  private final TalonFX shooterMotor;
  private final TalonFXConfiguration shooterMotorConfig;

  final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);

  private final VoltageOut sysIdControl = new VoltageOut(0);
  private final SysIdRoutine sysIdRoutine;

  public double targetVel;

  /** Creates a new Shooter. */
  public Shooter() {
    shooterMotor = new TalonFX(ShooterConstants.kShooterMotorId);
    shooterMotorConfig = new TalonFXConfiguration(); 
    shooterMotor.setPosition(0);



    shooterMotorConfig.CurrentLimits.StatorCurrentLimit = Constants.ShooterConstants.kShooterCurrentLimit;
    shooterMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    shooterMotorConfig.Slot0.kS = ShooterConstants.shooter_kS;
    shooterMotorConfig.Slot0.kV = ShooterConstants.shooter_kV;
    shooterMotorConfig.Slot0.kA = ShooterConstants.shooter_kA;
    shooterMotorConfig.Slot0.kP = ShooterConstants.shooter_kP;
    shooterMotorConfig.Slot0.kI = ShooterConstants.shooter_kI;
    shooterMotorConfig.Slot0.kD = ShooterConstants.shooter_kD;

    shooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;



    shooterMotorConfig.MotionMagic.MotionMagicAcceleration = ShooterConstants.kShooterMaxAcceleration;
    shooterMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 200.0;

    shooterMotorConfig.MotionMagic.MotionMagicJerk = ShooterConstants.kShooterMaxJerk;

    shooterMotor.getConfigurator().apply(shooterMotorConfig);
    
    sysIdRoutine = new SysIdRoutine (
      new SysIdRoutine . Config (
        Volts . of (1) . per ( Second ) , // Quasi - increases by 1V per sec
        Volts . of (7) , // Dynamic - jumps to 7V
        Seconds . of (10) // maxes at 10s
      ) ,
      new SysIdRoutine . Mechanism (
      ( Voltage volts ) -> {
        shooterMotor.setControl (sysIdControl.withOutput (volts.in (
      Volts ) ) ) ;
      } ,
      ( SysIdRoutineLog log ) -> {
        log.motor ("Shooter-Motor")
        .voltage ( Volts.of (shooterMotor.getMotorVoltage () .
        getValueAsDouble () ) )
        .angularPosition ( Rotations . of ( shooterMotor . getPosition () .
        getValueAsDouble () ) )
        .angularVelocity ( RotationsPerSecond.of( shooterMotor .
        getVelocity () . getValueAsDouble () ) ) ;
      } ,
        this
      )
    ) ;


    // SmartDashboard.putData("Shooter]]]", new Sendable() {
    //     @Override
    //     public void initSendable(SendableBuilder builder) {
    //         builder.addDoubleProperty("Speed", () -> targetVel, (val) -> targetVel = val);
    //     }
    // });

    targetVel = 0.0;
  }


  @Override
  public void periodic() {
    setVel();
    System.out.println(targetVel);
    // This method will be called once per scheduler run
  }

  public Command shooterSpin() {
    return Commands.runOnce(() -> spin());
  }

  public Command shooterStop() {
    return Commands.runOnce(() -> stop());
  }

  public void spin() {
    shooterMotor.set(ShooterConstants.kShooterSpeed);
  }

  public void stop() {
    shooterMotor.set(0);
  }

  public void increaseSpeed () {
    if (targetVel + ShooterConstants.kRpmIncrement <= ShooterConstants.kMaxRPM) {
        targetVel += ShooterConstants.kRpmIncrement;
    }
  }

  public void decreaseSpeed () {
    if (targetVel - ShooterConstants.kRpmIncrement >= ShooterConstants.kMinRPM) {
        targetVel -= ShooterConstants.kRpmIncrement;
    }
  }

  public void setVel() {
    if (targetVel == 0) {
        shooterMotor.set(0.0);
    }
    else {
        shooterMotor.setControl(m_request.withVelocity(4450.0 / 60.0)); //rpm 
    }
  }

  public Command sysIdQuasistatic ( SysIdRoutine . Direction direction ) {
    return sysIdRoutine . quasistatic ( direction ) ;
  }
 
  public Command sysIdDynamic ( SysIdRoutine . Direction direction ) {
    return sysIdRoutine . dynamic ( direction ) ;
  }
}