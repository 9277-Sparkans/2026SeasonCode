// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX hoodMotor;
    private final TalonFX shooterMotor;

    private boolean shooting;

    /** Creates a new Shooter. */
    public ShooterSubsystem() {
        // TODO: probably set current limits
        hoodMotor = new TalonFX(Constants.ShooterConstants.kHoodMotorId);
        shooterMotor = new TalonFX(Constants.ShooterConstants.kShooterMotorId);

        // hoodMotor.setPosition(Angle.ofBaseUnits(120, Degrees));
    }

    public Command shootCmd() {
        return Commands.runOnce(() -> toggleShoot());
    }

    public double getHoodPos() {
        return hoodMotor.getMotorVoltage().getValueAsDouble();
    }

    public void adjustHood(double delta) {
        hoodMotor.setPosition(Angle.ofBaseUnits(delta, Degrees));
    }

    public void runHood() {
        hoodMotor.set(1);
    }

    public void stopHood() {
        hoodMotor.set(0);
    }
    
    public Command runHoodCmd() {
        return Commands.run(() -> runHood()).handleInterrupt(() -> stopHood());
    }

    public Command stopHoodCmd() {
        return Commands.run(() -> stopHood());
    }

    private void toggleShoot() {
        shooting = !shooting;

        // TODO: adjust this based on distance to hub/alliance zone
        if (shooting) {
            shooterMotor.set(1);
        } else {
            shooterMotor.set(0);
        }
    }

    @Override
    public void periodic() {}
}
