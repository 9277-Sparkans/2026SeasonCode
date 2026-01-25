package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import frc.robot.Constants.ClimbConstants;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
        // public enum ClimbState { RAISE, LOWER, ASCEND, DESCEND, HANG }

        CANBus kCANBus = CANBus.roboRIO();
        private final TalonFX climbMotor;
        private final TalonFXConfiguration climbConfig;

        private ClimbState climbState;

        public enum ClimbState {
                DOWN,
                UP
        }

        public Climb() {

                climbMotor = new TalonFX(Constants.ClimbConstants.kClimbMotorID, kCANBus);
                climbConfig = new TalonFXConfiguration();

                /* PID */
                climbConfig.Slot0.kP = 1.5;
                climbConfig.Slot0.kI = 0.0;
                climbConfig.Slot0.kD = 0.1;

                /* Default Motion Magic (UP profile) */
                climbConfig.MotionMagic.MotionMagicCruiseVelocity = 30;
                climbConfig.MotionMagic.MotionMagicAcceleration = 15;

                climbMotor.getConfigurator().apply(climbConfig);

                //var climbmotorConfigurator = climbMotor.getConfigurator();
                //CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();

                //currentLimits.SupplyCurrentLimit = 20; // Amps
                //currentLimits.SupplyCurrentLimitEnable = true;
                //climbmotorConfigurator.apply(currentLimits);

                // Brake mode

        }

        public void setState(ClimbState state) {
                climbState = state;
        }

        public ClimbState getState() {
                return climbState;
        }

        // state for climb up
        public void states(ClimbState state) {

                switch (state) {
                        case UP:
                                MotionMagicVoltage climbUP = new MotionMagicVoltage(5).withSlot(0);
                                climbMotor.setControl(climbUP.withPosition(-228));
                                break;
                        case DOWN:
                                MotionMagicVoltage DOWN = new MotionMagicVoltage(5).withSlot(0);
                                climbMotor.setControl(DOWN.withPosition(0.0));
                                break;

                }
        }

        // public double goingdown(ClimbState state) {

        // switch (state) {
        // case DOWN:
        // MotionMagicVoltage DOWN = new MotionMagicVoltage(5).withSlot(0);
        // climbMotor.setControl(DOWN.withPosition(0.0));
        // default:
        // return 0.0;
        // }

        // }
        // state for climb down

  

        /* ================= COMMANDS ================= */

        public Command climbUp() {
                return Commands.runOnce(() -> {
                        System.out.println("climb UP command running");
                        states(ClimbState.UP);
                });
        }

        public Command climbDown() {
                return Commands.runOnce(() -> {
                        System.out.println("climb DOWN command running");
                        states(ClimbState.DOWN);
                });
        }

}
