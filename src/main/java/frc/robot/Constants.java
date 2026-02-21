package frc.robot;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.Filesystem;
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
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
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

        // Keyboard
        // 87777877ui00o831.00
        // NOTE: IDs here are 1-indexed as opposed to the keyboard firmware's 0-indexing
        // Add one to your ID or else you'll be binding to a completely different button
        public static final int kKeyboard_lockModeLeft = 2;
        public static final int kKeyboard_lockModeCenter = 3;
        public static final int kKeyboard_lockModeRight = 4;
        // public static final int kKeyboard_lockModeFire = 5;
        public static final int kKeyboard_climbUp = 6;
        public static final int kKeyboard_climbDown = 7;
        public static final int kKeyboard_autoFire = 8;

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
        public static final double turret_maxJerk = 100; // rotations per second^3
        public static final int turret_motorId = 32; // change this value
        public static final double turret_speed = 0.2;

        public static double turret_kV = 0.25;
        public static double turret_kA = 0.0;
        public static double turret_kS = 0.01;
        public static double turret_kP = 7.5;
        public static double turret_kI = 0.0;
        public static double turret_kD = 0.5;


        public static double kMaximumAngle = 60.0;
        public static double kMinimumAngle = -60.0;
        
        public static double kGearRatio = 105.0 / 18.0;
    }
    
    public static class ShooterConstants {
        public static final int kShooterMotorId = 33; //33

        public static final int kShooterCurrentLimit = 50;
        public static final double kShooterGearRatio = 36.0 / 30.0;

        public static final double kShooterMaxVoltage = 5;// kraken x44 max voltage
        public static final double kShooterMaxAcceleration = 100; // rotations per second^2
        public static final double kShooterMaxJerk = 1000; // rotations per second^3

        // subject to change if we end up automating these
        public static double kShooterSpeed = 30.0; //rpm

        public static double kMaxVelocity = 100.0;

        public static final int kRpmLenience = 200;

        public static final double kRpmIncrement = 5.0/3.0;

        public static final int kMinRPM = 0;
        public static final int kMaxRPM = 6000;

        public static final int kMinFlywheelRPM = 2400;
        public static final int kMaxFlywheelRPM = 7200;

        public static final double shooter_kS = 0.17909;
        public static final double shooter_kP = 0.018595;
        public static final double shooter_kI = 0.0;
        public static final double shooter_kD = 0.0;
        public static final double shooter_kV = 0.11753;
        public static final double shooter_kA = 0.0038935;
        // public static final double shooter_kA = 0.004;



        public static double autoshootDistanceRange = 10;
        public static double autoshootAngleWeight = 0.1; // idk tbh
        public static String lookupTablePath = "";



    }

    public static class HoodConstants
    {
        public static final int kHoodMotorId = 34;
        public static final int kHoodEncoderId = 41;

        public static final double hood_maxVelocity = 20; // rotations per second; was 1.0, setting it to this to avoid grinding the gear again!
        public static final double hood_maxAcceleration = 80; // rotations per second^2
        public static final double hood_maxVoltage = 15;// kraken x44 max voltage

        public static final double hood_kG = 0.00;
        public static final double hood_kS = 0.01;
        public static final double hood_kV = 0.095;
        public static final double hood_kA = 0.01;
        public static final double hood_kP = 8.85;
        public static final double hood_kI = 0.0;
        public static final double hood_kD = 0.5;

        public static final double kHoodSpeed = 0.1;

        public static final double kMinimumAngle = 0;
        public static final double kMaximumAngle = 12;

        public static final double kMinimumEncoderPos = 0;
        public static final double kMaximumEncoderPos = -0.67236328125;

        public static final double maxEncoderValue = -1.258789; // test for this

        public static final double kGearRatio = 210.0 / 15.0;

        public static final double kHoodCurrentLimit = 35;

        public static final double kHoodIncrement = 17.0/2.0;

        public static final double kIdkManConstant = 1.209757239732467f;
    }

    public static class IntakeConstants
    {
        public static final int intakeMotorId = 38; //38
        public static final double kIntakeGearRatio = 12.0 / 18.0;

        public static final double kIntakeCurrentLimit = 80.0;

        public static final double intake_kS = 0.18572;
        public static final double intake_kV = 0.11754;
        public static final double intake_kA = 0.0048972;
        public static final double intake_kP = 0.030667;
        public static final double intake_kI = 0.0;
        public static final double intake_kD = 0.0;

        public static final double intakeMaxVoltage = 5; // can change if not needed
        public static final double intakeMaxAcceleration = 100;
        public static final double intakeMaxVelocity = 500; // rps
        public static final double intakeSpeed = 100.0; //rps
    }

    public static class HingeConstants {

        public static final int kHingeMotorId = 39;

        public static final double hinge_kS = 0.01;        
        public static final double hinge_kP = 10.0;
        public static final double hinge_kI = 0.0;
        public static final double hinge_kD = 0.1;
        public static final double hinge_kV = 0.12;
        public static final double hingeMaxVoltage = 4;
        public static final double hingeMaxAcceleration = 10;
        public static final double hingeMaxVelocity = 10; // rps
        public static final double kHingeCurrentLimit = 50;

        public static final int hingeCountsPerRevolution = 2048; // for kraken x60
        public static final double hingeGearRatio = 45.0 / 1.0;

        public static final double hingeMaxDeg = -100.0;
    }

    public static class ClimbConstants
    {
        public static final int kClimbMotorID = 37;

        public static final double kClimbMaxVelocity = 90; // rps
        public static final double kClimbMaxAcceleration = 30; // rps^2
        public static final int kClimbCurrent_Limit = 80;   
        public static final double kClimb_Speed = 0.3; 

        public static final double kClimb_kS = 0.01;
        public static final double kClimb_kV = 0.12;
        public static final double kClimb_kA = 0.01;
        public static final double kClimb_kP = 10;
        public static final double kClimb_kI = 0;
        public static final double kClimb_kD = 0.1;
        public static final double kClimb_kG = 0.12;

        public static final int kClimbUp = 80;
        public static final int kClimbDown = 5;
        public static final int kClimbHang = 60;

        public static final double kClimbGearRatio = 6.0 / 1.0;
    }

    public static class TransferConstants
    {
        public static final int transferID = 31; // change later 31

        public static final double transferMaxVoltage = 4; // can change if not needed
        public static final double transferMaxAcceleration = 40;
        public static final double transferMaxVelocity = 100; // rps
        public static final int kTransferCurrent_Limit = 80;
        public static final double kTransferGearRatio = 30.0 / 24.0;

        public static final double kTransfer_kS = 0.18572;
        public static final double kTransfer_kV = 0.11754;
        public static final double kTransfer_kA = 0.0048972;
        public static final double kTransfer_kP = 0.030667;
        public static final double kTransfer_kI = 0.0;
        public static final double kTransfer_kD = 0.0;

        public static final double kTargetTransferRps = -100.0;
    }

    public static final class IndexerConstants {
        public static final double kIndexerMaxVoltage = 5;// kraken x44 max voltage
        public static final double kIndexerCurrentLimit = 100;
        public static final int kIndexerMotorId = 35; // 35
        public static final double kIndexerSpeed = 100.0; //rps
        public static final double kIndexerGearRatio = 12.0 / 15.0;
        public static final double indexerSpeed = 30.0; // old

        public static final double kIndexer_kS = 0.18572;  
        public static final double kIndexer_kV = 0.11754;
        public static final double kIndexer_kA = 0.0048972;
        public static final double kIndexer_kP = 0.030667;
        public static final double kIndexer_kI = 0.0;
        public static final double kIndexer_kD = 0.0;
        public static final double kIndexerMaxAcceleration = 400;
        public static final double kIndexerMaxJerk = 4000;
    }

    public static final class LimelightConstants {
        public static final String limelightName = "limelight-a";
    }

    public static final class LockModeConstants
    {
        public static final int kLockModeRPM = 3300;

        public static final double kHoodLeft = 15;
        public static final double kHoodCenter = 15;
        public static final double kHoodRight = 15;
        
        public static final double kTurretLeft = 45;
        public static final double kTurretCenter = 0;
        public static final double kTurretRight = -45;
    }
}