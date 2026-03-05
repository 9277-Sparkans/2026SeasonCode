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

import edu.wpi.first.apriltag.AprilTagFieldLayout;

public class Constants {
    public static final class QuickAccessConstants {
        public static final boolean swerveEnabled = true;
        public static final boolean manipulatorsEnabled = true;
        public static final boolean autoControlsEnabled = true;

        // change between DRIVER_CONTROLLER and DRIVER_STICKS for controller and sticks,
        // erm i guess
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
        public static final double kDeadband = 0.02;

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
        public static final int kKeyboard_modeToggle = 5;
        public static final int kKeyboard_trackToggle = 3;

        public static final int kKeyboard_lockModeToggle = 2;
        public static final int kKeyboard_lockModeLeft = 11;
        public static final int kKeyboard_lockModeRight = 13;
        public static final int kKeyboard_lockModeCenter = 12;
        public static final int kKeyboard_lockModeTrenchLeft = 8;
        public static final int kKeyboard_lockModeTrenchRight = 15;

        public static final int kKeyboard_climbUp = 7;
        public static final int kKeyboard_climbDown = 10;
        public static final int kKeyboard_climbHang = 9;
        
        public static final int kKeyboard_intakeDeploy = 16;
        public static final int kKeyboard_intakeRetract = 6;

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
        public static final int turret_motorId = 32; // 32
        public static final int kTurretEncoderId = 40;
        public static final double kTurretEncoderOffset = 0.0; // Rotations
        public static final double turret_speed = 0.2;

        public static double turret_kV = 0.095;
        public static double turret_kA = 0.0;
        public static double turret_kS = 0.6;
        public static double turret_kP = 13.5;
        public static double turret_kI = 0.07;
        public static double turret_kD = 0.25;

        public static double kMaximumAngle = 70.0;
        public static double kMinimumAngle = -70.0;
        
        public static double kGearRatio = 105.0 / 18.0;

        public static final edu.wpi.first.math.geometry.Transform3d ROBOT_TO_TURRET_TRANSFORM = new edu.wpi.first.math.geometry.Transform3d(
                new edu.wpi.first.math.geometry.Translation3d(0.0, 0.0, 0.4572), // 18 inches
                new edu.wpi.first.math.geometry.Rotation3d());
    }

    public static class ShooterConstants {
        public static final int kShooterMotorId = 33; //33

        public static final int kShooterCurrentLimit = 80;
        public static final double kShooterGearRatio = 36.0 / 30.0;

        public static final double kShooterMaxVoltage = 5;// kraken x44 max voltage
        public static final double kShooterMaxAcceleration = 100; // rotations per second^2
        public static final double kShooterMaxJerk = 1000; // rotations per second^3

        // subject to change if we end up automating these
        public static double kShooterSpeed = 30.0; //rpm

        public static double kMaxVelocity = 100.0;

        public static final int kRpmLenience = 200;

        public static final double kRpmIncrement = 50.0;

        public static final int kMinRPM = 0;
        public static final int kMaxRPM = 6000;

        public static double kMinOperationalRPM = 3000;

        public static final int kMinFlywheelRPM = 2400;
        public static final int kMaxFlywheelRPM = 7200;

        public static final double shooter_kS = 0.17909;
        public static final double shooter_kP = 0.018595;
        public static final double shooter_kI = 0.0;
        public static final double shooter_kD = 0.0;
        public static final double shooter_kV = 0.11753;
        public static final double shooter_kA = 0.0038935;

        public static final String lookupTablePath = Filesystem.getDeployDirectory().getPath() + "/ShooterLookupTable/shooter-lookup.csv"; // Generated with github.com/DanielR723/shooter
        public static final double autoshootDistanceRange = 0.1; // Range of distances to check to choose the best shot (Least hood rotation and flywheel RPM change)
        public static final double maxShotError = 0.001; // The maximum allowed error for a shot to occur

        // Tuning constants for autofire
        public static final double rpmOffset = 50.0;
        public static final double distancePower = 1.4;
        public static final double speedPower = 1.0;
        
        // Weights of each value when calculating an optimal shot, higher value means higher priority to minimize
        public static final double botVelocityWeight = 1.0;
        public static final double shooterRPMWeight = 0.2;
        public static final double hoodAngleWeight = 0.0;
    }

    public static class HoodConstants
    {
        public static final int kHoodMotorId = 34; //34
        public static final int kHoodEncoderId = 41; //41

        public static final double hood_maxVelocity = 20; // rotations per second; was 1.0, setting it to this to avoid grinding the gear again!
        public static final double hood_maxAcceleration = 80; // rotations per second^2
        public static final double hood_maxVoltage = 15;// kraken x44 max voltage

        public static final double hood_kG = 0.00;
        public static final double hood_kS = 0.39;
        public static final double hood_kV = 0.095;
        public static final double hood_kA = 0.01;
        public static final double hood_kP = 13.5; // 7.23, 11
        public static final double hood_kI = 0.07; // 0.01
        public static final double hood_kD = 0.25; //0.38

        public static final double kHoodSpeed = 0.1;

        public static final double kMinimumAngle = 0;
        public static final double kMaximumAngle = 12;

        public static final double kMinimumEncoderPos = 0;
        public static final double kMaximumEncoderPos = -0.67236328125;

        public static final double maxEncoderValue = -1.258789; // test for this

        public static final double kGearRatio = 210.0 / 15.0; //210.0 / 15.0;

        public static final double kHoodCurrentLimit = 45;

        public static final double kHoodIncrement = 17.0/2.0;

        public static final double kIdkManConstant = 1.209757239732467f;

        public static final double hoodOffset = 0.205; // Offset from center of the bot (m)
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
        public static final double intakeMaxAcceleration = 500;
        public static final double intakeMaxVelocity = 500; // rps
        public static final double intakeSpeed = 150.0; //rps max 66.6 rps 400.0 
    }

    public static class HingeConstants {

        public static final int kHingeMotorId = 39; //39
        public static final int deploymentMaxDeg = 115;

        // rollers
        public static final int rollerID = 1;

        public static final double hinge_kS = 0.24;        
        public static final double hinge_kP = 10.0;
        public static final double hinge_kI = 0.0;
        public static final double hinge_kD = 0.1;
        public static final double hinge_kV = 0.12;
        public static final double hinge_kG = 2.5; //0.25
        public static final double hingeMaxAcceleration = 20.0;
        public static final double hingeMaxVelocity = 60.0; // rps
        public static final double kHingeCurrentLimit = 120;

        public static final int hingeCountsPerRevolution = 2048; // for kraken x60
        public static final double hingeGearRatio = 45.0 / 1.0; //carter: 8:1 tyler: 45:1

        public static final double hingeMaxDeg = 50.0; //-100.0;
    }

    public static class ClimbConstants
    {
        public static final int kClimbMotorID = 37; //37

        public static final double kClimbMaxVelocity = 90; // rps
        public static final double kClimbMaxAcceleration = 30; // rps^2
        public static final int kClimbCurrent_Limit = 120;   
        public static final double kClimb_Speed = 0.3; 

        public static final double kClimb_kS = 0.05;
        public static final double kClimb_kV = 0.12;
        public static final double kClimb_kA = 0.05;
        public static final double kClimb_kP = 9.0;
        public static final double kClimb_kI = 0;
        public static final double kClimb_kD = 0.1;
        public static final double kClimb_kG = 0.12;

        public static final int kClimbUp = 105;
        public static final int kClimbDown = 0;
        public static final int kClimbHang = 20;

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

    /**
     * DEPRECATED: Use frc.robot.Vision.VisionConstants.aprilTagLayout instead.
     * This ensures all code uses the same AprilTag field layout as PhotonVision.
     */
    @Deprecated(since = "2026")
    public static final class VisionConstants {

        /**
         * DEPRECATED: Use frc.robot.Vision.VisionConstants.aprilTagLayout instead.
         */
        @Deprecated(since = "2026")
        public static final AprilTagFieldLayout TAG_LAYOUT = frc.robot.Vision.VisionConstants.aprilTagLayout;
    }

    public static class IndexerConstants {
        public static final double kIndexerMaxVoltage = 5;// kraken x44 max voltage
        public static final double kIndexerCurrentLimit = 100;
        public static final int kIndexerMotorId = 35; // 35
        public static final double kIndexerSpeed = 60.0; //rps
        public static final double kIndexerGearRatio = 12.0 / 15.0;
        public static final double indexerSpeed = 30.0; // rps

        // tomorrow - use the best tuning and then increase ka and kv cuz the recorrect is kinda messed up
        public static final double kIndexer_kS = 0.18572; //0.01;
        public static final double kIndexer_kV = 0.12; //0.11754;
        public static final double kIndexer_kA = 0.01; //0.0048972;
        public static final double kIndexer_kP = 0.030667; //1.0;
        public static final double kIndexer_kI = 0.01; //0.005; 
        public static final double kIndexer_kD = 0.0; //0.05;
        public static final double kIndexerMaxAcceleration = 400;
        public static final double kIndexerMaxJerk = 4000;
    }

    public static class FieldConstants {
        public static final double BLUE_HUB_X = 4.625594; // meters
        public static final double BLUE_HUB_Y = 4.034536; // meters
        public static final double RED_HUB_X = 11.915394; // meters 
        public static final double RED_HUB_Y = 4.034536; // meters
        public static final edu.wpi.first.math.geometry.Translation3d HUB_BLUE = new edu.wpi.first.math.geometry.Translation3d(
                BLUE_HUB_X, BLUE_HUB_Y, 1.4); // 1.4m height approx
        public static final edu.wpi.first.math.geometry.Translation3d HUB_RED = new edu.wpi.first.math.geometry.Translation3d(
                RED_HUB_X, RED_HUB_Y, 1.4);
    }

    public static final class CanBusConstants {
        // setting the canivore
        public static final String kCANivore = "DriveTrain";
    }

    public static final class LockModeConstants
    {
        //LEFT
        public static final int kRPMLeft = 4450;
        public static final double kHoodLeft = 5.0;
        public static final double kTurretLeft = 44.0;

        //CENTER
        public static final int kRPMCenter = 4050;
        public static final double kHoodCenter = 7.0;
        public static final double kTurretCenter = 10.0;

        //RIGHT
        public static final int kRPMRight = 4450;
        public static final double kHoodRight = 5.0;
        public static final double kTurretRight = -44.0;

        //TRENCH LEFT
        public static final int kRPMTrenchLeft = 4350;
        public static final double kHoodTrenchLeft = 6.0;
        public static final double kTurretTrenchLeft = 62.0;

        //TRENCH RIGHT
        public static final int kRPMTrenchRight = 4350;
        public static final double kHoodTrenchRight = 6.0;
        public static final double kTurretTrenchRight = -62.0;

        //LOCK
        public static final int kRPMLock = 3700;
        public static final double kHoodLock = 6.0;
        public static final double kTurretLock = 0.0;
    }
}
