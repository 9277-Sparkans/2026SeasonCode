package frc.robot;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.Timer;

public class Constants {
    public static final class QuickAccessConstants {
        public static final boolean swerveEnabled = true;
        public static final boolean manipulatorsEnabled = true;
        public static final boolean autoControlsEnabled = true;
        
        public static final ControlTypes controlType = ControlTypes.DRIVER_STICKS;
        public static enum ControlTypes {
            DEV,
            DRIVER_STICKS,
            DRIVER_CONTROLLER
        };
        public static final boolean usingKeyboard = true;
    }

    public static final class OIConstants {

        // Ports
        public static final int kOperatorControllerPort = 0;
        public static final int kDriverControllerPort = 1;
        public static final int kDriverTranslateStickPort = 2;
        public static final int kDriverRotateStickPort = 3;
        public static final int kBackupOperatorControllerPort = 4;
        public static final double kDeadband = 0.04;

        // Joysticks
        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 2;

        // Buttons
        public static final int kController_x = 1;
        public static final int kController_a = 2;
        public static final int kController_b = 3;
        public static final int kController_y = 4;

        public static final int kController_back = 9;
        public static final int kController_start = 10;
        public static final int kController_leftStickButton = 11;
        public static final int kController_rightStickButton = 12;

        // Triggers [CONTROLLER ONLY]
        public static final int kController_leftBumper = 5;
        public static final int kController_rightBumper = 6;
        public static final int kController_leftTrigger = 7;
        public static final int kController_rightTrigger = 8;

        
        // Flight Sticks
        public static final int kSticks_trigger = 1;
        public static final int kSticks_centerHandle = 2;
        public static final int kSticks_leftHandle = 3;
        public static final int kSticks_rightHandle = 4;

        // Flight Sticks [LEFT HANDED] (from bird's eye view)
        public static final int kLeftSticks_leftGrid_topLeft = 11;
        public static final int kLeftSticks_leftGrid_topMid = 12;
        public static final int kLeftSticks_leftGrid_topRight = 13;
        public static final int kLeftSticks_leftGrid_bottomLeft = 16;
        public static final int kLeftSticks_leftGrid_bottomMid = 15;
        public static final int kLeftSticks_leftGrid_bottomRight = 14;
        public static final int kLeftSticks_rightGrid_topLeft = 7;
        public static final int kLeftSticks_rightGrid_topMid = 6;
        public static final int kLeftSticks_rightGrid_topRight = 5;
        public static final int kLeftSticks_rightGrid_bottomLeft = 8;
        public static final int kLeftSticks_rightGrid_bottomMid = 9;
        public static final int kLeftSticks_rightGrid_bottomRight = 10;

        // Flight Sticks [RIGHT HANDED] (from bird's eye view)
        public static final int kRightSticks_leftGrid_topLeft = 5;
        public static final int kRightSticks_leftGrid_topMid = 6;
        public static final int kRightSticks_leftGrid_topRight = 7;
        public static final int kRightSticks_leftGrid_bottomLeft = 10;
        public static final int kRightSticks_leftGrid_bottomMid = 9;
        public static final int kRightSticks_leftGrid_bottomRight = 8;
        public static final int kRightSticks_rightGrid_topLeft = 13;
        public static final int kRightSticks_rightGrid_topMid = 12;
        public static final int kRightSticks_rightGrid_topRight = 11;
        public static final int kRightSticks_rightGrid_bottomLeft = 14;
        public static final int kRightSticks_rightGrid_bottomMid = 15;
        public static final int kRightSticks_rightGrid_bottomRight = 16;
    } 
    

    public static final class TurretConstants {

        public static final double turret_maxVelocity = 40; // rotations per second
        public static final double turret_maxAcceleration = 20; // rotations per second^2
        public static final double turret_maxVoltage = 5;// kraken x44 max voltage
        public static final int turret_motorId = 34; // change this value

        public static final double turret_kG = 0.03;
        public static final double turret_kS = 0.01;
        public static final double turret_kP = 4.5;
        public static final double turret_kI = 0.0;
        public static final double turret_kD = 0.3;

        public static int kMaximumAngle = 70;
        public static int kMinimumAngle = -70;

        public static double kGearRatio = 1.0 / (18.0 / 105.0);
    }
    
    public static class ShooterConstants {
        public static final int kShooterMotorId = 33;

        public static final int kShooterCurrentLimit = 30;

        // subject to change if we end up automating these
        public static final double kShooterSpeed = 0.7;

        public static final int kRpmLenience = 200;

        public static final int kRpmIncrement = 10;

        public static final int kMaxRPM = 5000;

        public static final double shooter_kG = 0.0;
        public static final double shooter_kS = 0.0;
        public static final double shooter_kP = 0.1;
        public static final double shooter_kI = 0.0;
        public static final double shooter_kD = 0.0;
        public static final double shooter_kV = 0.0;
        public static final double shooter_kA = 0.0;

    }

    public static class HoodConstants
    {
        public static final int kHoodMotorId = 30;

        public static final double hood_maxVelocity = 125; // rotations per second
        public static final double hood_maxAcceleration = 80; // rotations per second^2
        public static final double hood_maxVoltage = 5;// kraken x44 max voltage

        public static final double hood_kG = 0.03;
        public static final double hood_kS = 0.01;
        public static final double hood_kP = 4.5;
        public static final double hood_kI = 0.0;
        public static final double hood_kD = 0.3;

        public static final double kHoodSpeed = 1;

        public static final double kMinimumAngle = 15;
        public static final double kMaximumAngle = 45;

        public static final double kGearRatio = 1.0 / (15.0 / 210.0);

        public static final double kHoodCurrentLimit = 35;
        public static final double kShooterCurrentLimit = 35;

        public static final double kHoodIncrement = 5;
    }

    public static class IntakeConstants
    {
        // deployment
        public static final int deploymentID = 0;

        public static final double deploymentKS = 0.01;
        public static final double deploymentKP = 10;
        public static final double deploymentKI = 0;
        public static final double deploymentKD = 0.1;

        public static final double deploymentMaxVoltage = 4;
        public static final double deploymentMaxAcceleration = 40;
        public static final double deploymentMaxVelocity = 100; // rps

        public static final int deploymentCountsPerRevolution = 2048; // for kraken x60
        public static final int deploymentGearRatio = 1 / (1 / 1);

        public static final int deploymentMaxDeg = 115;
    
        // rollers
        public static final int rollerID = 1;

        public static final double rollerKS = 0.01;
        public static final double rollerKP = 10;
        public static final double rollerKI = 0;
        public static final double rollerKD = 0.1;

        public static final double rollerMaxVoltage = 4; // can change if not needed
        public static final double rollerMaxAcceleration = 40;
        public static final double rollerMaxVelocity = 100; // rps
    }

    public static class ClimbConstants
    {
        public static final int kClimbMotorID = 37;

        public static final double kClimbMaxVelocity = 90; // rps
        public static final int kClimbCURRENT_LIMIT = 30;   
        public static final double kClimb_SPEED = .3; 


        public static final int kClimbGearRatio = 9 / 1;
    }

    public static class TransferConstants
    {
        public static final int transferID = 31; // change later

        public static final double transferMaxVoltage = 4; // can change if not needed
        public static final double transferMaxAcceleration = 40;
        public static final double transferMaxVelocity = 100; // rps
    }
    
}
