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
import frc.robot.Constants.ShooterConstants.ShotData;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

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
        public static final double kDeadband = 0.065;

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
        public static final int kKeyboard_trackToggle = 3; //3
        public static final int kKeyboard_duck = 1; //1

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
        public static final int kKeyboard_AutoTrackToggle = 14;

        public static final int kKeyboard_fudgeFactorShooterIncrease = 17;
        public static final int kKeyboard_fudgeFactorShooterDecrease = 18;
        public static final int kKeyboard_fudgeFactorTurretIncrease = 19;
        public static final int kKeyboard_fudgeFactorTurretDecrease = 20;


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

    public static final class LedConstants {
        public static final int kCandleId = 50; // 50

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
        public static final double turret_currentLimit = 40.0; // amps

        public static double turret_kV = 0.095;
        public static double turret_kA = 0.0;
        public static double turret_kS = 0.0; // 0.6 1.0 // handled by ffmap
        public static double turret_kP = 1.5; // 13.5 //12.0 //1.5
        public static double turret_kI = 0.0; // 0.07
        public static double turret_kD = 0.35; // 0.25 //0.35

        public static double turret_kV1 = 0.095;
        public static double turret_kA1 = 0.0;
        public static double turret_kS1 = 0.50; // 0.6 1.0
        public static double turret_kP1 = 12.0; // 13.5
        public static double turret_kI1 = 0.07;
        public static double turret_kD1 = 0.25;

        public static double turret_kV2 = 0.095;
        public static double turret_kA2 = 0.0;
        public static double turret_kS2 = 0.9; // 0.6 1.0
        public static double turret_kP2 = 12.0; // 13.5
        public static double turret_kI2 = 0.07; // 0.07
        public static double turret_kD2 = 0.25;

        public static double kMaximumAngle = 90.0;
        public static double kMinimumAngle = -90.0;

        public static double kGearRatio = 105.0 / 18.0;

        public static double kTurretCurrentLimit = 50.0;

        public static final InterpolatingDoubleTreeMap FF_MAP = new InterpolatingDoubleTreeMap();

        static {
            FF_MAP.put(-10.0, 0.4); // 0.55
            FF_MAP.put(-20.0, 0.7); // 0.65
            FF_MAP.put(-35.0, 0.5); // 0.7
            FF_MAP.put(-40.0, 0.8);
            FF_MAP.put(-60.0, 0.4);
            FF_MAP.put(-70.0, 0.7);

            // turret angle, voltage required
            FF_MAP.put(0.0, 0.3); // 0.5
            FF_MAP.put(10.0, 0.3); // 0.55
            FF_MAP.put(20.0, 0.35); // 0.85
            FF_MAP.put(30.0, 0.55); // 1.15
            FF_MAP.put(35.0, 0.7); // 0.83
            FF_MAP.put(44.0, 1.0);
            FF_MAP.put(55.0, 1.35);
            FF_MAP.put(65.0, 1.4);
            FF_MAP.put(70.0, 2.1);
            FF_MAP.put(80.0, 2.1);
        }

        public static final edu.wpi.first.math.geometry.Transform3d ROBOT_TO_TURRET_TRANSFORM = new edu.wpi.first.math.geometry.Transform3d(
                new edu.wpi.first.math.geometry.Translation3d(0.12543, 0.0, 0.2897), // updated
                new edu.wpi.first.math.geometry.Rotation3d());
    }

    public static class ShooterConstants {
        public static final int kShooterMotorId = 33; // 33

        public static final int kShooterCurrentLimit = 100;
        public static final double kShooterGearRatio = 36.0 / 30.0; // 36/ 30

        public static final double kShooterMaxVoltage = 5;// kraken x44 max voltage
        public static final double kShooterMaxAcceleration = 600; // rotations per second^2
        public static final double kShooterMaxJerk = 1200; // rotations per second^3

        // subject to change if we end up automating these
        public static double kShooterSpeed = 30.0; // rpm

        public static double kMaxVelocity = 100.0;

        public static final int kRpmLenience = 200;

        public static final double kRpmIncrement = 50.0;

        public static final int kMinRPM = 3500; // 0
        public static final int kMaxRPM = 6000; // 6000

        public static double kMinOperationalRPM = 3000;

        public static final int kMinFlywheelRPM = 2400;
        public static final int kMaxFlywheelRPM = 7200;
        public static final double kFlywheelRadius = edu.wpi.first.math.util.Units.inchesToMeters(1.75);

        public static final double shooter_kS = 0.17909;
        public static final double shooter_kP = 0.018595;
        public static final double shooter_kI = 0.0;
        public static final double shooter_kD = 0.0;
        public static final double shooter_kV = 0.11753;
        public static final double shooter_kA = 0.0038935;

        public static final String lookupTablePath = Filesystem.getDeployDirectory().getPath()
                + "/ShooterLookupTable/shooter-lookup.csv"; // Generated with github.com/DanielR723/shooter
        public static final double autoshootDistanceRange = 0.1; // Range of distances to check to choose the best shot
                                                                 // (Least hood rotation and flywheel RPM change)
        public static final double maxShotError = 0.001; // The maximum allowed error for a shot to occur

        // Tuning constants for autofire
        public static final double rpmOffset = 50.0;
        public static final double distancePower = 1.4;
        public static final double speedPower = 1.0;
        public static final double autoFireDriveSpeed = 0.25;
        public static final double hysteresisDeadband = 0.0;

        // Weights of each value when calculating an optimal shot, higher value means
        // higher priority to minimize
        public static final double botVelocityWeight = 0.0;
        public static final double shooterRPMWeight = 0.1;
        public static final double hoodAngleWeight = 1.0;

        // quadratic regression tests
        public record ShotData(double rpm, double hoodAngle) {
        }

        // https://www.desmos.com/calculator/aitlewjs62
<<<<<<< HEAD
        public static final double RPM_A = 4.8105;
        public static final double RPM_B = 394.1445;
        public static final double RPM_C = 2732.58665;

=======
        // public static final double RPM_A = 29.29341;
        // public static final double RPM_B = 161.21136;
        // public static final double RPM_C = 3221.91989;

        // public static final double HOOD_A = -0.0538813;
        // public static final double HOOD_B = -1.96426;
        // public static final double HOOD_C = 15.05734;

        public static final double RPM_A = 4.8105;
        public static final double RPM_B = 394.1445;
        public static final double RPM_C = 2732.58665;

>>>>>>> 4bf1340b1e3562c5ae148d9319ea75d216872662
        public static final double HOOD_A = 0.035756;
        public static final double HOOD_B = -2.748184;
        public static final double HOOD_C = 16.707595;

        public static final double TOF_A = -0.0742554;
        public static final double TOF_B = 0.800318;
        public static final double TOF_C = -0.956608;

        public static ShotData getShotData(double distMeters) {
            double rpm = RPM_A * distMeters * distMeters + RPM_B * distMeters + RPM_C;
            double hood = HOOD_A * distMeters * distMeters + HOOD_B * distMeters + HOOD_C;
            rpm = Math.max(3800.0, Math.min(6000.0, rpm));
            hood = Math.max(2.0, Math.min(10.0, hood));
            return new ShotData(rpm, hood);
        }

        public static double getTOF(double distMeters) {
            double tof = TOF_A * distMeters * distMeters + TOF_B * distMeters + TOF_C;
            return Math.max(0.98, Math.min(1.39, tof));
        }

        public static final double[] shotAutoAlignPositions = new double[] { 2.5, 3.0, 3.5, 4.0, 4.5, 4.75, 5.0, 5.25, 5.5 };

        // // autofire treemapping with interpolation
        // public record ShotData(double rpm, double hoodAngle) implements Interpolatable<ShotData> {
        //     @Override
        //     public ShotData interpolate(ShotData endValue, double t) {
        //         return new ShotData(
        //                 edu.wpi.first.math.MathUtil.interpolate(this.rpm, endValue.rpm, t),
        //                 edu.wpi.first.math.MathUtil.interpolate(this.hoodAngle, endValue.hoodAngle, t));
        //     }
        // }

        // public static final InterpolatingTreeMap<Double, ShotData> SHOT_MAP = new InterpolatingTreeMap<Double, ShotData>(
        //         InverseInterpolator.forDouble(), (start, end, t) -> start.interpolate(end, t));

        // public static final InterpolatingDoubleTreeMap TOF_MAP = new InterpolatingDoubleTreeMap();

        // static {

        // // data points (distance from turret, rpm, hood angle)
        // // time of flight (distance from turret, time of flight)
        // SHOT_MAP.put(2.378, new ShotData(3800.0, 10.0));
        // SHOT_MAP.put(2.428, new ShotData(3800.8, 9.92));
        // SHOT_MAP.put(2.478, new ShotData(3803.3, 9.84));
        // SHOT_MAP.put(2.528, new ShotData(3807.1, 9.75));
        // SHOT_MAP.put(2.578, new ShotData(3812.3, 9.67));
        // SHOT_MAP.put(2.628, new ShotData(3818.6, 9.58));
        // SHOT_MAP.put(2.678, new ShotData(3826.0, 9.48));
        // SHOT_MAP.put(2.728, new ShotData(3834.3, 9.39));
        // SHOT_MAP.put(2.778, new ShotData(3843.3, 9.29));
        // SHOT_MAP.put(2.828, new ShotData(3853.0, 9.18));
        // SHOT_MAP.put(2.878, new ShotData(3863.1, 9.08));
        // SHOT_MAP.put(2.928, new ShotData(3873.7, 8.97));
        // SHOT_MAP.put(2.978, new ShotData(3888.6, 8.86));
        // SHOT_MAP.put(3.028, new ShotData(3908.5, 8.75));
        // SHOT_MAP.put(3.078, new ShotData(3932.5, 8.62));
        // SHOT_MAP.put(3.128, new ShotData(3959.7, 8.5));
        // SHOT_MAP.put(3.178, new ShotData(3989.1, 8.37));
        // SHOT_MAP.put(3.228, new ShotData(4019.8, 8.24));
        // SHOT_MAP.put(3.278, new ShotData(4050.8, 8.11));
        // SHOT_MAP.put(3.328, new ShotData(4081.2, 7.98));
        // SHOT_MAP.put(3.378, new ShotData(4110.0, 7.85));
        // SHOT_MAP.put(3.428, new ShotData(4136.3, 7.72));
        // SHOT_MAP.put(3.478, new ShotData(4160.1, 7.59));
        // SHOT_MAP.put(3.528, new ShotData(4183.9, 7.47));
        // SHOT_MAP.put(3.578, new ShotData(4207.6, 7.34));
        // SHOT_MAP.put(3.628, new ShotData(4230.9, 7.22));
        // SHOT_MAP.put(3.678, new ShotData(4253.7, 7.09));
        // SHOT_MAP.put(3.728, new ShotData(4275.6, 6.97));
        // SHOT_MAP.put(3.778, new ShotData(4296.4, 6.84));
        // SHOT_MAP.put(3.828, new ShotData(4315.9, 6.72));
        // SHOT_MAP.put(3.878, new ShotData(4333.7, 6.59));
        // SHOT_MAP.put(3.928, new ShotData(4349.6, 6.47));
        // SHOT_MAP.put(3.978, new ShotData(4364.0, 6.34));
        // SHOT_MAP.put(4.028, new ShotData(4377.1, 6.22));
        // SHOT_MAP.put(4.078, new ShotData(4389.2, 6.1));
        // SHOT_MAP.put(4.128, new ShotData(4400.8, 5.97));
        // SHOT_MAP.put(4.178, new ShotData(4412.0, 5.85));
        // SHOT_MAP.put(4.228, new ShotData(4423.3, 5.72));
        // SHOT_MAP.put(4.278, new ShotData(4435.0, 5.6));
        // SHOT_MAP.put(4.328, new ShotData(4447.4, 5.48));
        // SHOT_MAP.put(4.378, new ShotData(4460.8, 5.36));
        // SHOT_MAP.put(4.428, new ShotData(4475.6, 5.23));
        // SHOT_MAP.put(4.478, new ShotData(4492.0, 5.11));
        // SHOT_MAP.put(4.528, new ShotData(4509.5, 4.99));
        // SHOT_MAP.put(4.578, new ShotData(4527.8, 4.87));
        // SHOT_MAP.put(4.628, new ShotData(4547.0, 4.75));
        // SHOT_MAP.put(4.678, new ShotData(4567.0, 4.63));
        // SHOT_MAP.put(4.728, new ShotData(4588.0, 4.51));
        // SHOT_MAP.put(4.778, new ShotData(4609.7, 4.39));
        // SHOT_MAP.put(4.828, new ShotData(4632.3, 4.27));
        // SHOT_MAP.put(4.878, new ShotData(4655.8, 4.15));
        // SHOT_MAP.put(4.928, new ShotData(4680.0, 4.03));
        // SHOT_MAP.put(4.978, new ShotData(4705.0, 3.91));
        // SHOT_MAP.put(5.028, new ShotData(4730.9, 3.79));
        // SHOT_MAP.put(5.078, new ShotData(4757.5, 3.67));
        // SHOT_MAP.put(5.128, new ShotData(4784.9, 3.55));
        // SHOT_MAP.put(5.178, new ShotData(4813.0, 3.43));
        // SHOT_MAP.put(5.228, new ShotData(4841.9, 3.31));
        // SHOT_MAP.put(5.278, new ShotData(4871.5, 3.19));
        // SHOT_MAP.put(5.328, new ShotData(4901.8, 3.08));
        // SHOT_MAP.put(5.378, new ShotData(4932.8, 2.96));
        // SHOT_MAP.put(5.424, new ShotData(4962.0, 2.85));

        // // SHOT_MAP.put(2.265, new ShotData(4000.0, 10.0));
        // // SHOT_MAP.put(2.765, new ShotData(4050.0, 10.0));
        // // SHOT_MAP.put(3.265, new ShotData(4200.0, 9.0));
        // // SHOT_MAP.put(3.765, new ShotData(4400.0, 8.0));
        // // SHOT_MAP.put(4.265, new ShotData(4500.0, 6.0));
        // // SHOT_MAP.put(4.765, new ShotData(4700.0, 5.0));
        // // SHOT_MAP.put(5.265, new ShotData(5150.0, 5.0));
        // // SHOT_MAP.put(6.223, new ShotData(5900.0, 2.0));

        // // distance, time of flight
        // // TOF_MAP.put(2.265, 0.7940);
        // // TOF_MAP.put(2.765, 0.8145);
        // // TOF_MAP.put(3.265, 0.8444);
        // // TOF_MAP.put(3.765, 0.8798);
        // // TOF_MAP.put(4.265, 0.8543);
        // // TOF_MAP.put(4.765, 0.8719);
        // // TOF_MAP.put(5.265, 0.9375);

        // // TOF_MAP.put(2.765, 0.956875);
        // // TOF_MAP.put(3.265, 0.9975);
        // // TOF_MAP.put(3.765, 1.165);
        // // TOF_MAP.put(4.265, 1.1375);
        // // TOF_MAP.put(4.765, 1.28375);
        // // TOF_MAP.put(5.265, 1.22875);
        // }

        // dumping tree map
        public static final InterpolatingTreeMap<Double, ShotData> DUMP_MAP = new InterpolatingTreeMap<Double, ShotData>(
                InverseInterpolator.forDouble(), (start, end, t) -> new ShotData(
                        edu.wpi.first.math.MathUtil.interpolate(start.rpm(), end.rpm(), t),
                        edu.wpi.first.math.MathUtil.interpolate(start.hoodAngle(), end.hoodAngle(), t)));
        public static final InterpolatingDoubleTreeMap DUMP_TOF_MAP = new InterpolatingDoubleTreeMap();

        static {
            // data points (distance from turret, rpm, hood angle)
            // time of flight (distance from turret, time of flight)
            DUMP_MAP.put(4.00, new ShotData(3307.8, 4.0));
            DUMP_MAP.put(4.50, new ShotData(3603.5, 3.3));
            DUMP_MAP.put(5.00, new ShotData(3903.7, 3.6));
            DUMP_MAP.put(5.50, new ShotData(4304.9, 3.4));
            DUMP_MAP.put(6.00, new ShotData(4700.0, 4.5));
            DUMP_MAP.put(6.50, new ShotData(5200.0, 6.3));
            DUMP_MAP.put(7.00, new ShotData(5600.0, 10.0));
            DUMP_MAP.put(7.50, new ShotData(5700.0, 10.0));

            DUMP_TOF_MAP.put(4.00, 1.206);
            DUMP_TOF_MAP.put(4.50, 1.313);
            DUMP_TOF_MAP.put(5.00, 1.399);
            DUMP_TOF_MAP.put(5.50, 1.501);
            DUMP_TOF_MAP.put(6.00, 1.563);
            DUMP_TOF_MAP.put(6.50, 1.605);
            DUMP_TOF_MAP.put(7.00, 1.582);
            DUMP_TOF_MAP.put(7.50, 1.587);




        }
    }

    public static class HoodConstants {
        public static final int kHoodMotorId = 34; // 34
        public static final int kHoodEncoderId = 41; // 41

        public static final double hood_maxVelocity = 20; // rotations per second; was 1.0, setting it to this to avoid
                                                          // grinding the gear again!
        public static final double hood_maxAcceleration = 10; // rotations per second^2
        public static final double hood_maxVoltage = 15;// kraken x44 max voltage

        public static final double hood_kG = 0.00;
        public static final double hood_kS = 0.09;
        public static final double hood_kV = 0.095;
        public static final double hood_kA = 0.01;
        public static final double hood_kP = 13.5; // 7.23, 11
        public static final double hood_kI = 0.07; // 0.01
        public static final double hood_kD = 0.0; // 0.38 0.25

        public static final double kHoodSpeed = 0.1;

        public static final double kMinimumAngle = 3; // 0
        public static final double kMaximumAngle = 10; // 12

        public static final double kMinimumEncoderPos = -0.44849609375;
        public static final double kMaximumEncoderPos = 0;

        public static final double maxEncoderValue = -1.258789; // test for this

        public static final double kGearRatio = 18.f / 210.f; // 210.0 / 15.0;

        public static final double kHoodCurrentLimit = 45;

        public static final double kHoodIncrement = 17.0 / 2.0;

        public static final double kIdkManConstant = 1.209757239732467f;

        public static final double hoodOffset = 0.205; // Offset from center of the bot (m)
    }

    public static class IntakeConstants {
        public static final int intakeMotorId = 38; // 38
        public static final double kIntakeGearRatio = 2.0 / 3.0;

        public static final double kIntakeCurrentLimit = 80.0;

        public static final double intake_kS = 0.15572; // 0.18572
        public static final double intake_kV = 0.11754;
        public static final double intake_kA = 0.0048972;
        public static final double intake_kP = 0.030667;
        public static final double intake_kI = 0.0;
        public static final double intake_kD = 0.0;

        public static final double intakeMaxVoltage = 5; // can change if not needed
        public static final double intakeMaxAcceleration = 100;
        // public static final double intakeMaxJerk = 600;
        public static final double intakeMaxVelocity = 500; // rps
        public static final double intakeSpeed = 50.0; // 45
        public static final double intakeAutoSpeed = 66.0; // 66   rps max 66.6 rps 400.0
        public static final double intakeAgitate = 20.0;
    }

    public static class HingeConstants {

        public static final int kHingeMotorId = 39; // 39
        public static final int kHingeEncoderId = 42;
        public static final int deploymentMaxDeg = 115;

        public static final double hinge_kS = 0.24;
        public static final double hinge_kP = 3.0;
        public static final double hinge_kI = 0.0;
        public static final double hinge_kD = 0.03;
        public static final double hinge_kV = 0.12;
        public static final double hinge_kG = 2.5; // 0.25
        public static final double hingeMaxAcceleration = 10.0;
        public static final double hingeMaxVelocity = 100.0; // rps
        public static final double kHingeCurrentLimit = 50.0;

        public static final int hingeCountsPerRevolution = 2048; // for kraken x60
        public static final double hingeGearRatio = 112.5 / 1.0; // carter: 8:1 tyler: 45:1

        public static final double hingeMaxDeg = 50.0; // -100.0;

        public static final double kHingeDeploymentPosition = 0.0;
        public static final double kHingeAgitatePosition = 0.28;
        public static final double kHingeRetractedPosition = 0.33;
        // public static final double kHingeDeadband = 0.025;
    }

    public static class ClimbConstants {
        public static final int kClimbMotorID = 37; // 37

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

        public static final int kClimbUp = 175;
        public static final int kClimbDown = 0;
        public static final int kClimbHang = 10;

        public static final double kClimbGearRatio = 10.0 / 1.0;
    }

    public static class TransferConstants {
        public static final int transferID = 31; // change later 31

        public static final double transferMaxVoltage = 4; // can change if not needed
        public static final double transferMaxAcceleration = 200.0;
        public static final double transferMaxJerk = 400.0;
        public static final double transferMaxVelocity = 100.0; // rps
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

        // currently 30
        public static final double kIndexerSpeed = 30.0; // rps 60 before
        public static final double kIndexerAgitateSpeed = -30.0; // rps 60 before
        public static final double indexerSpeed = 30.0; // rps

        public static final double kIndexerGearRatio = 12.0 / 15.0;

        // tomorrow - use the best tuning and then increase ka and kv cuz the recorrect
        // is kinda messed up
        public static final double kIndexer_kS = 0.18572; // 0.01;
        public static final double kIndexer_kV = 0.12; // 0.11754;
        public static final double kIndexer_kA = 0.01; // 0.0048972;
        public static final double kIndexer_kP = 0.030667; // 1.0;
        public static final double kIndexer_kI = 0.01; // 0.005;
        public static final double kIndexer_kD = 0.0; // 0.05;
        public static final double kIndexerMaxAcceleration = 150;
        public static final double kIndexerMaxJerk = 500;

        public static final double kIndexerStallCurrent = 0;
        public static final double kIndexerStallVelocity = 0;

    }

    public static class RobotDimensions {
        public static final double FULL_WIDTH = 0.838;
        public static final double FULL_LENGTH = 0.838;
        public static final double BUMPER_HEIGHT = 0.19;
    }

    public static class FieldConstants {
        public static final double FIELD_LENGTH = Units.inchesToMeters(650.12);
        public static final double FIELD_WIDTH = Units.inchesToMeters(316.64);
        public static final double BLUE_HUB_X = 4.625594; // meters
        public static final double BLUE_HUB_Y = 4.034536; // meters
        public static final double RED_HUB_X = 11.915394; // meters
        public static final double RED_HUB_Y = 4.034536; // meters

        public static final edu.wpi.first.math.geometry.Translation3d HUB_BLUE = new edu.wpi.first.math.geometry.Translation3d(
                BLUE_HUB_X, BLUE_HUB_Y, 1.4);
        public static final edu.wpi.first.math.geometry.Translation3d HUB_RED = new edu.wpi.first.math.geometry.Translation3d(
                RED_HUB_X, RED_HUB_Y, 1.4);

        public static final double BLUE_DUMP_RIGHT_X = 1.164;
        public static final double BLUE_DUMP_RIGHT_Y = 1.253326;
        public static final double BLUE_DUMP_LEFT_X = 1.7;
        public static final double BLUE_DUMP_LEFT_Y = 6.816;

        public static final double RED_DUMP_RIGHT_X = 15.376; // 16.540988 - 1.7
        public static final double RED_DUMP_RIGHT_Y = 1.253326;
        public static final double RED_DUMP_LEFT_X = 14.840988;
        public static final double RED_DUMP_LEFT_Y = 6.816;

        public static final double BLUE_LEFT_CLIMB_X = 1.07;
        public static final double BLUE_LEFT_CLIMB_Y = 4.613;
        public static final double BLUE_PRE_ALIGN_LEFT_X = 1.587;
        public static final double BLUE_PRE_ALIGN_LEFT_Y = 5.012;
        public static final double BLUE_PRE_LEFT_CLIMB_X = BLUE_LEFT_CLIMB_X;
        public static final double BLUE_PRE_LEFT_CLIMB_Y = BLUE_LEFT_CLIMB_Y + 0.2;
        public static final double BLUE_LEFT_CLIMB_HEADING = 180.0;

        public static final double BLUE_RIGHT_CLIMB_X = 1.108;
        public static final double BLUE_RIGHT_CLIMB_Y = 2.872;
        public static final double BLUE_PRE_ALIGN_RIGHT_X = 1.6900000000000004;
        public static final double BLUE_PRE_ALIGN_RIGHT_Y = 2.508;
        public static final double BLUE_PRE_RIGHT_CLIMB_X = BLUE_RIGHT_CLIMB_X;
        public static final double BLUE_PRE_RIGHT_CLIMB_Y = BLUE_RIGHT_CLIMB_Y - 0.2;
        public static final double BLUE_RIGHT_CLIMB_HEADING = 0.0;

        public static final double RED_PRE_ALIGN_LEFT_X = FIELD_LENGTH - BLUE_PRE_ALIGN_LEFT_X;
        public static final double RED_PRE_ALIGN_LEFT_Y = FIELD_WIDTH - BLUE_PRE_ALIGN_LEFT_Y;
        public static final double RED_PRE_LEFT_CLIMB_X = FIELD_LENGTH - BLUE_PRE_LEFT_CLIMB_X;
        public static final double RED_PRE_LEFT_CLIMB_Y = FIELD_WIDTH - BLUE_PRE_LEFT_CLIMB_Y;
        public static final double RED_LEFT_CLIMB_X = FIELD_LENGTH - BLUE_LEFT_CLIMB_X;
        public static final double RED_LEFT_CLIMB_Y = FIELD_WIDTH - BLUE_LEFT_CLIMB_Y;
        public static final double RED_LEFT_CLIMB_HEADING = 0.0;

        public static final double RED_PRE_ALIGN_RIGHT_X = FIELD_LENGTH - BLUE_PRE_ALIGN_RIGHT_X;
        public static final double RED_PRE_ALIGN_RIGHT_Y = FIELD_WIDTH - BLUE_PRE_ALIGN_RIGHT_Y;
        public static final double RED_PRE_RIGHT_CLIMB_X = FIELD_LENGTH - BLUE_PRE_RIGHT_CLIMB_X;
        public static final double RED_PRE_RIGHT_CLIMB_Y = FIELD_WIDTH - BLUE_PRE_RIGHT_CLIMB_Y;
        public static final double RED_RIGHT_CLIMB_X = FIELD_LENGTH - BLUE_RIGHT_CLIMB_X;
        public static final double RED_RIGHT_CLIMB_Y = FIELD_WIDTH - BLUE_RIGHT_CLIMB_Y;
        public static final double RED_RIGHT_CLIMB_HEADING = 180.0;

    }

    public static final class CanBusConstants {
        // setting the canivore
        public static final String kCANivore = "DriveTrain";
    }

    public static final class LockModeConstants {
        // LEFT
        public static final int kRPMLeft = 4900; // 4450
        public static final double kHoodLeft = 4.0;
        public static final double kTurretLeft = 45.0;

        // CENTER
        public static final int kRPMCenter = 4600; // 4450
        public static final double kHoodCenter = 5.5; // 7
        public static final double kTurretCenter = 10.0;

        // RIGHT
        public static final int kRPMRight = 4900;
        public static final double kHoodRight = 4.0;
        public static final double kTurretRight = -45.0;

        // TRENCH LEFT
        public static final int kRPMTrenchLeft = 4650;
        public static final double kHoodTrenchLeft = 6.0;
        public static final double kTurretTrenchLeft = 62.0;

        // TRENCH RIGHT
        public static final int kRPMTrenchRight = 4650;
        public static final double kHoodTrenchRight = 6.0;
        public static final double kTurretTrenchRight = -62.0;

        // LOCK
        public static final int kRPMLock = 4550; // 4550
        public static final double kHoodLock = 6.0; // 6
        public static final double kTurretLock = 0.0; //0
    }

    public static final class DriveAssistConstants {
        public static final double TRENCH_BUMP_X = Units.inchesToMeters(181.56);
        public static final double TRENCH_WIDTH = Units.inchesToMeters(49.86);
        public static final double TRENCH_BUMP_LENGTH = Units.inchesToMeters(47.0);
        public static final double TRENCH_BAR_WIDTH = Units.inchesToMeters(4.0);
        public static final double TRENCH_BLOCK_WIDTH = Units.inchesToMeters(12.0);
        public static final double BUMP_WIDTH = Units.inchesToMeters(73.0);
        public static final double TRENCH_CENTER = TRENCH_WIDTH / 2.0;
        public static final double TRENCH_ALIGN_TIME = 0.5;
        public static final double BUMP_ALIGN_TIME = 0.3;
        public static final double TRANSLATION_kP = 8.0;
        public static final double TRANSLATION_kI = 0.0;
        public static final double TRANSLATION_kD = 0.05;
        public static final double TRANSLATION_TOLERANCE = 0.05;
        public static final double ROTATION_kP = 5.0;
        public static final double ROTATION_kI = 0.0;
        public static final double ROTATION_kD = 0.0;
        public static final double ROTATION_TOLERANCE = Units.degreesToRadians(5.0);

        public static final double NORMAL_ACCEL_LIMIT = 15.0;
        public static final double SHOOTING_ACCEL_LIMIT = 1.5;
        public static final double BRAKING_ACCEL_LIMIT = 100.0;
        public static final double SHOOTING_MAX_VELOCITY = 0.5;
        public static final double DUMP_MAX_VELOCITY = 0.8;
        public static final double NORMAL_ROT_ACCEL_LIMIT = 25.0;
        public static final double SHOOTING_ROT_ACCEL_LIMIT = 4.0;
    }
}
