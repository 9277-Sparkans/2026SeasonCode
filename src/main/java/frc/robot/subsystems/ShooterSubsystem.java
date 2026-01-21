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
import frc.robot.Constants.HoodConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX hoodMotor;
    private final TalonFXConfiguration hoodMotorConfiguration;

    private final TalonFX shooterMotor;

    private boolean shooting;

    public double tgtAngle = HoodConstants.kMaximumAngle;

    public int shooterRPM = 0; // rpm

    /** Creates a new Shooter. */
    public ShooterSubsystem() {
        // TODO: probably set current limits
        hoodMotor = new TalonFX(HoodConstants.kHoodMotorId);
        hoodMotorConfiguration = new TalonFXConfiguration();

        CurrentLimitsConfigs hoodConfigs = new CurrentLimitsConfigs();
        hoodConfigs.StatorCurrentLimit = HoodConstants.kHoodCurrentLimit;
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

        shooterMotor = new TalonFX(HoodConstants.kHoodMotorId);
    }

    public Command shootCmd() {
        return Commands.runOnce(() -> toggleShoot());
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

    public Command moveHoodUpCmd()
    {
        return Commands.runOnce(() -> moveHoodUp());
    }

    public Command moveHoodDownCmd()
    {
        return Commands.runOnce(() -> moveHoodDown());
    }

    // hood
    public void moveHoodToAngle(double theta)
    {
        MotionMagicVoltage hoodRequest = new MotionMagicVoltage(theta).withSlot(0);
		hoodMotor.setControl(hoodRequest.withPosition(GetHoodAngle()));
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

    public void moveHoodUp()
    {
        if (tgtAngle - HoodConstants.kHoodIncrement >= HoodConstants.kMinimumAngle)
        {
            tgtAngle -= HoodConstants.kHoodIncrement;
        }
    }

    public void moveHoodDown()
    {
        if (tgtAngle + HoodConstants.kHoodIncrement <= HoodConstants.kMaximumAngle)
        {
            tgtAngle += HoodConstants.kHoodIncrement;
        }
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

    public void fireAtRPM(int rpm)
    {
        MotionMagicVelocityVoltage velocityTgt = new MotionMagicVelocityVoltage(rpm).withSlot(0);
        shooterMotor.setControl(velocityTgt);
    }

    public void autoFire()
    {
        int tgtRPM = GetCorrectRPM();
        tgtAngle = GetCorrectHoodAngle();
        fireAtRPM(tgtRPM);
        moveHoodToAngle(tgtAngle);
    }

    // getters
    public int GetCorrectRPM()
    {
        return shooterRPM; // replace with the actual math later
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
        if (shooterRPM - ShooterConstants.kRpmIncrement > 0)
        {
            shooterRPM -= ShooterConstants.kRpmIncrement;
        }
    }

    public double GetCorrectHoodAngle()
    {
        Translation2d position = Limelight.GetDistance();
        double distance = Math.sqrt(position.getX() * position.getX() + position.getY() * position.getY());

        // field length 158.6  inches or 4.02844 meters
        double angle = (distance / 4.02844) * (HoodConstants.kMaximumAngle - HoodConstants.kMinimumAngle);

        return HoodConstants.kMinimumAngle + angle; // replace with actual math later
    }

    public double GetShooterVelocity()
    {
        return shooterMotor.getVelocity().getValueAsDouble();
    }

    public double GetHoodAngle()
    {
        double currentHoodPosition = hoodMotor.getPosition().getValueAsDouble(); // rotations
        double hoodSpace = currentHoodPosition / HoodConstants.kGearRatio;
        double hoodAngle = hoodSpace * 360;

        return Math.max(HoodConstants.kMaximumAngle - hoodAngle, HoodConstants.kMinimumAngle);
    }

    @Override
    public void periodic() {}
}
