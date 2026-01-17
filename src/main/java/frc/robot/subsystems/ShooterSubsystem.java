// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

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

    public void runHood() {
        hoodMotor.set(ShooterConstants.kHoodSpeed);
    }

    public void runHoodReverse() {
        hoodMotor.set(-ShooterConstants.kHoodSpeed);
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

    @Override
    public void periodic() {}
}
