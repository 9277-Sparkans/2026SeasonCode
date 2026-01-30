// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Limelight;
import frc.robot.Telemetry;
import frc.robot.Constants.HoodConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class Shooter extends SubsystemBase {
    public int targetRPM;
    private final TalonFX shooterMotor;
    private final TalonFXConfiguration ShooterMotorConfiguration;

    private boolean shooting = false;


    public int shooterRPM; // rpm

    /** Creates a new Shooter. */
    public Shooter() {

        shooterMotor = new TalonFX(ShooterConstants.kShooterMotorId);
        ShooterMotorConfiguration = new TalonFXConfiguration();

        CurrentLimitsConfigs shooterConfigs = new CurrentLimitsConfigs();
        shooterConfigs.StatorCurrentLimit = ShooterConstants.kShooterCurrentLimit;
        shooterConfigs.StatorCurrentLimitEnable = true;
        shooterMotor.getConfigurator().apply(shooterConfigs);

        ShooterMotorConfiguration.Slot0.kG = ShooterConstants.shooter_kG;
        ShooterMotorConfiguration.Slot0.kP = ShooterConstants.shooter_kP;
        ShooterMotorConfiguration.Slot0.kI = ShooterConstants.shooter_kI;
        ShooterMotorConfiguration.Slot0.kD = ShooterConstants.shooter_kD;

        
        ShooterMotorConfiguration.Slot0.kS = ShooterConstants.shooter_kS;
        ShooterMotorConfiguration.Slot0.kV = ShooterConstants.shooter_kV;
        ShooterMotorConfiguration.Slot0.kA = ShooterConstants.shooter_kA;


        shooterMotor.getConfigurator().apply(ShooterMotorConfiguration);

        shooterRPM = 0;

        Telemetry.telemeterizeMotor("Shooter", shooterMotor);

    }

    public Command shootCmd() {
        toggleShoot();
        return Commands.runOnce(() -> {});
    }
    
    
    // shoot
    private void toggleShoot() {
        shooting = !shooting;

        // TODO: adjust this based on distance to hub/alliance zone
        if (shooting) {
            shooterMotor.set(ShooterConstants.kShooterSpeed);
        } else {
            shooterMotor.set(0);
        }
    }

    public void fireAtRPM()
    {
        // MotionMagicVelocityVoltage velocityTgt = new MotionMagicVelocityVoltage(rpm).withSlot(0);
        // shooterMotor.setControl(velocityTgt.withVelocity(GetShooterVelocity()));

        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ShooterConstants.shooter_kS, ShooterConstants.shooter_kV, ShooterConstants.shooter_kA);

        PIDController pidController = new PIDController(ShooterConstants.shooter_kP, ShooterConstants.shooter_kI, ShooterConstants.shooter_kD);

        double setpointVelocity = shooterRPM / 60;
        double feedforwardVoltage = feedforward.calculate(setpointVelocity);
        double feedbackVoltage = pidController.calculate(shooterMotor.getVelocity().getValueAsDouble(), setpointVelocity);

        shooterMotor.setVoltage(feedforwardVoltage + feedbackVoltage);
    }
    
    // getters
    public int GetCorrectRPM()
    {
        return shooterRPM; // replace with the actual math later
    }

    // rps
    public double GetShooterVelocity()
    {
        return shooterMotor.getVelocity().getValueAsDouble();
    }

    public void increaseSpeed()
    {
        if (shooterRPM + ShooterConstants.kRpmIncrement < ShooterConstants.kMaxRPM)
        {
            shooterRPM += ShooterConstants.kRpmIncrement;
        }
    }

    public void decreaseSpeed()
    {
        if (shooterRPM - ShooterConstants.kRpmIncrement > -10)
        {
            shooterRPM -= ShooterConstants.kRpmIncrement;
        }
    }

    public void setShooterRPM(int input)
    {
        shooterRPM = input;
    }  

    @Override
    public void periodic() 
    {
        fireAtRPM();
    }
}
