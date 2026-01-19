// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.HoodConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX hoodMotor;
    private final TalonFXConfiguration hoodMotorConfiguration;

    private final TalonFX shooterMotor;

    private boolean shooting;

    /** Creates a new Shooter. */
    public ShooterSubsystem() {
        // TODO: probably set current limits
        hoodMotor = new TalonFX(HoodConstants.kHoodMotorId);
        hoodMotorConfiguration = new TalonFXConfiguration();

        CurrentLimitsConfigs hoodConfigs = new CurrentLimitsConfigs();
        hoodConfigs.StatorCurrentLimit = Constants.ShooterConstants.kHoodCurrentLimit;
        hoodConfigs.StatorCurrentLimitEnable = true;
        hoodMotor.getConfigurator().apply(hoodConfigs);

        hoodMotorConfiguration.Slot0.kG = HoodConstants.hood_kG;
		hoodMotorConfiguration.Slot0.kP = HoodConstants.hood_kP;
		hoodMotorConfiguration.Slot0.kI = HoodConstants.hood_kI;
		hoodMotorConfiguration.Slot0.kD = HoodConstants.hood_kD; 

		hoodMotorConfiguration.Voltage.PeakForwardVoltage = HoodConstants.hood_maxVoltage;
		hoodMotorConfiguration.Voltage.PeakReverseVoltage = -HoodConstants.hood_maxVoltage;
		hoodMotorConfiguration.MotionMagic.MotionMagicAcceleration = HoodConstants.hood_maxAcceleration;
		hoodMotorConfiguration.MotionMagic.MotionMagicCruiseVelocity = HoodConstants.hood_maxVelocity;

        shooterMotor = new TalonFX(HoodConstants.kShooterMotorId);
    }

    public Command shootCmd() {
        return Commands.runOnce(() -> toggleShoot());
    }

    public void moveHoodToAngle(double theta)
    {

    }

    public double getHoodAngle()
    {
        double currentHoodPosition = hoodMotor.getPosition(); // rotations
        double hoodSpace = currentHoodPosition / HoodConstants.kGearRatio;
        double hoodAngle = hoodSpace * (maximumAngle-minimumAngle);

        return Math.max(HoodConstants.maximumAngle - hoodAngle, HoodConstants.minimumAngle);
    }

    public void runHood() {
        hoodMotor.set(HoodConstants.kHoodSpeed);
    }

    public void runHoodReverse() {
        hoodMotor.set(-HoodConstants.kHoodSpeed);
    }

    public void stopHood() {
        hoodMotor.set(0);
    }
    
    public Command runHoodCmd() {
        return Commands.runOnce(() -> runHood());
    }

    public Command runHoodReverseCmd() {
        return Commands.runOnce(() -> runHoodReverse());
    }

    public Command stopHoodCmd() {
        return Commands.runOnce(() -> stopHood());
    }

    private void toggleShoot() {
        shooting = !shooting;

        // TODO: adjust this based on distance to hub/alliance zone
        if (shooting) {
            shooterMotor.set(ShooterConstants.kShooterSpeed);
        } else {
            shooterMotor.set(0);
        }
    }

    public void fireAtRPM(int rpm)
    {
        MotionMagicVelocityVoltage velocityTgt = new MotionMagicVelocityVoltage(rpm).withSlot(0);
        shooterMotor.setControl(velocityTgt);
    }

    public void autoFire()
    {
        int tgtRPM = getCorrectRPM();
        fireAtRPM(tgtRPM);
    }

    public int getCorrectRPM()
    {
        return 1000; // replace with the actual math later
    }

    public double GetShooterVelocity()
    {
        return shooterMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {}
}
