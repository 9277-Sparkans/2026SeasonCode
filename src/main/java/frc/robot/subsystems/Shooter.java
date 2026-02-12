// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Telemetry;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

public class Shooter extends SubsystemBase {
    public int targetRPM;
    private final TalonFX shooterMotor;
    private final TalonFXConfiguration ShooterMotorConfiguration;
    final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);
    private boolean shooting = false;


    public int shooterRpm; // rpm

    /** Creates a new Shooter. */
    public Shooter() {

        shooterMotor = new TalonFX(ShooterConstants.kShooterMotorId);
        ShooterMotorConfiguration = new TalonFXConfiguration();

        CurrentLimitsConfigs shooterConfigs = new CurrentLimitsConfigs();
        shooterConfigs.StatorCurrentLimit = ShooterConstants.kShooterCurrentLimit;
        shooterConfigs.StatorCurrentLimitEnable = true;
        shooterMotor.getConfigurator().apply(shooterConfigs);

        ShooterMotorConfiguration.Slot0.kP = ShooterConstants.shooter_kP;
        ShooterMotorConfiguration.Slot0.kI = ShooterConstants.shooter_kI;
        ShooterMotorConfiguration.Slot0.kD = ShooterConstants.shooter_kD;

        ShooterMotorConfiguration.Slot0.kS = ShooterConstants.shooter_kS;
        ShooterMotorConfiguration.Slot0.kV = ShooterConstants.shooter_kV;
        ShooterMotorConfiguration.Slot0.kA = ShooterConstants.shooter_kA;

        shooterMotor.getConfigurator().apply(ShooterMotorConfiguration);

        shooterRpm = 0;

        Telemetry.telemeterizeMotor("Shooter", shooterMotor);

    }

    public Command shootCmd() {
        return Commands.runOnce(() -> {});
    }

    public void setTgtRpm(int rpm) {
        this.shooterRpm = rpm;
    }


    // shoot
    public void fireAtRpm()
    {
        double tgt = (shooterRpm / ShooterConstants.kShooterGearRatio) * 60.0; // convert to rpm at motor
        shooterMotor.setControl(m_request.withVelocity(tgt));
    }

    public double GetShooterRPM()
    {
        return shooterMotor.getVelocity().getValueAsDouble() * 60;
    }

    public void increaseSpeed() {
        if (shooterRpm + ShooterConstants.kRpmIncrement < ShooterConstants.kMaxRPM)
        {
            shooterRpm += ShooterConstants.kRpmIncrement;
        }
    }

    public void decreaseSpeed()
    {
        if (shooterRpm - ShooterConstants.kRpmIncrement > -10)
        {
            shooterRpm -= ShooterConstants.kRpmIncrement;
        }
    }

    

    public double GetShooterVelocity()
    {
        return shooterMotor.getVelocity().getValueAsDouble();
    }

    

    @Override
    public void periodic() {
        fireAtRpm();
    }
}
