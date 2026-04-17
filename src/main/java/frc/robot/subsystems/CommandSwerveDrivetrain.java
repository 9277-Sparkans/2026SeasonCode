package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.util.Zones;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    public final Pigeon2 pidgey = new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id);

    public final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
            getKinematics(), pidgey.getRotation2d(),
            getState().ModulePositions);

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineRotation;

    private final SwerveRequest.FieldCentric m_fieldCentricDrive = new SwerveRequest.FieldCentric();

    private double m_lastX = 0;
    private double m_lastY = 0;
    private double m_lastOmega = 0;

    private final PIDController m_translationController = new PIDController(
            Constants.DriveAssistConstants.TRANSLATION_kP,
            Constants.DriveAssistConstants.TRANSLATION_kI,
            Constants.DriveAssistConstants.TRANSLATION_kD
    );

    private final PIDController m_rotationController = new PIDController(
            Constants.DriveAssistConstants.ROTATION_kP,
            Constants.DriveAssistConstants.ROTATION_kI,
            Constants.DriveAssistConstants.ROTATION_kD
    );

    @AutoLogOutput
    public DriveAssistMode m_currentAssistMode = DriveAssistMode.NORMAL;

    @AutoLogOutput(key = "DriveAssist/TargetPose")
    private Pose2d m_targetPose = new Pose2d();

    @AutoLogOutput(key = "DriveAssist/TrenchZones")
    public Pose2d[] getTrenchZones() {
        return Zones.TRENCH_ZONES.getCorners();
    }

    @AutoLogOutput(key = "DriveAssist/BumpZones")
    public Pose2d[] getBumpZones() {
        return Zones.BUMP_ZONES.getCorners();
    }

    @AutoLogOutput(key = "DriveAssist/Debug/TrenchSetpoint")
    private double m_debugTrenchSetpoint = 0;

    @AutoLogOutput(key = "DriveAssist/Debug/TrenchClosestHeading")
    private double m_debugTrenchHeading = 0;

    @AutoLogOutput(key = "DriveAssist/Debug/IsTopTrench")
    private boolean m_debugIsTopTrench = false;

    @AutoLogOutput(key = "DriveAssist/Debug/TrenchCurrentY")
    private double m_debugTrenchCurrentY = 0;

    @AutoLogOutput(key = "DriveAssist/Debug/PidOutput")
    private double m_debugPidOutput = 0;

    // --- NT-controllable assist toggles (all default enabled) ---
    @AutoLogOutput(key = "DriveAssist/Enable/All")
    public boolean assistEnabled = true;
    @AutoLogOutput(key = "DriveAssist/Enable/TrenchLock")
    public boolean trenchLockEnabled = true;
    @AutoLogOutput(key = "DriveAssist/Enable/BumpLock")
    public boolean bumpLockEnabled = true;
    @AutoLogOutput(key = "DriveAssist/Enable/ShootingRotLock")
    public boolean shootingRotLockEnabled = true;
    @AutoLogOutput(key = "DriveAssist/Enable/SlewRate")
    public boolean slewRateEnabled = true;
    @AutoLogOutput(key = "DriveAssist/Enable/VelocityCap")
    public boolean velocityCapEnabled = true;

    public enum DriveAssistMode {
        NORMAL,
        TRENCH_LOCK,
        BUMP_LOCK,
        DISABLED
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz
     *                                  on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    // autobuilder
    private void configureAutoBuilder() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    () -> getStateCopy().Pose, // supply position
                    this::resetPose, // consumer for seeding pose against auto
                    () -> getStateCopy().Speeds, // supply speeds
                    // consumer of chassis speeds and feedforwards to drive
                    (speeds, feedforwards) -> setControl(
                            m_pathApplyRobotSpeeds.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                    // .withWheelForceFeedforwardsX(feedforwards,robotRelativeForcesXNewtons())
                    // .withWheelForceFeedforwardsY(feedforwards,robotRelativeForcesYNewtons())
                    ),
                    new PPHolonomicDriveController(
                            new PIDConstants(5, 0, 0), // translation
                            new PIDConstants(5, 0, 0) // rotation
                    ),
                    config,
                    // flip for red vs blue
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this);
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load Pathplanner config and configure autobuilder",
                    ex.getStackTrace());
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision
     *                              camera.
     * @param timestampSeconds      The timestamp of the vision measurement in
     *                              seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        // Convert WPILib FPGA time (from PhotonVision) to Phoenix 6 Epoch time
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement
     *                                 in the form [x, y, theta]ᵀ, with units in
     *                                 meters and radians.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }

    /**
     * Get the current 3D pose of the robot.
     * Uses the 2D odometry pose and the 3D rotation from the Pigeon2.
     * 
     * @return The 3D pose of the robot.
     */
    public Pose3d getPose3d() {
        var pose2d = getStateCopy().Pose;
        var rotation3d = pidgey.getRotation3d();
        return new Pose3d(pose2d.getX(), pose2d.getY(), 0.0, rotation3d);
    }

    public Command driveWithAssist(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier,
            BooleanSupplier isShootingSupplier,
            BooleanSupplier isDumpingSupplier,
            DoubleSupplier desiredTurretAngleSupplier,
            DoubleSupplier MaxSpeed,
            DoubleSupplier MaxAngularRate) {

        m_translationController.setTolerance(Constants.DriveAssistConstants.TRANSLATION_TOLERANCE);
        m_rotationController.setTolerance(Constants.DriveAssistConstants.ROTATION_TOLERANCE);
        m_rotationController.enableContinuousInput(-Math.PI, Math.PI);

        return run(() -> {
            double x = xSupplier.getAsDouble();
            double y = ySupplier.getAsDouble();
            double omega = omegaSupplier.getAsDouble();
            boolean isShooting = isShootingSupplier.getAsBoolean();
            boolean isDumping = isDumpingSupplier.getAsBoolean();

            Pose2d pose = getStateCopy().Pose;
            ChassisSpeeds speeds = getStateCopy().Speeds;

            // zone detection for trench and bump, borrowed from hammerheads
            boolean inTrench = Zones.TRENCH_ZONES
                    .willContain(() -> pose, () -> speeds, Constants.DriveAssistConstants.TRENCH_ALIGN_TIME)
                    .getAsBoolean();
            boolean inBump = Zones.BUMP_ZONES
                    .willContain(() -> pose, () -> speeds, Constants.DriveAssistConstants.BUMP_ALIGN_TIME)
                    .getAsBoolean();

            if (assistEnabled && trenchLockEnabled && inTrench)
                m_currentAssistMode = DriveAssistMode.TRENCH_LOCK;
            else if (assistEnabled && bumpLockEnabled && inBump)
                m_currentAssistMode = DriveAssistMode.BUMP_LOCK;
            else
                m_currentAssistMode = DriveAssistMode.NORMAL;

            double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
            double maxRot = 2.0 * Math.PI;

            // capping velocity while autofiring
            double velCap = (assistEnabled && velocityCapEnabled && isShooting)
                    ? (isDumping ? Constants.DriveAssistConstants.DUMP_MAX_VELOCITY
                                : Constants.DriveAssistConstants.SHOOTING_MAX_VELOCITY)
                    : maxSpeed;
            double targetX = MathUtil.clamp(x * maxSpeed, -velCap, velCap);
            double targetY = MathUtil.clamp(y * maxSpeed, -velCap, velCap);
            double targetOmega = omega * maxRot;

            // asymmetric slew rate limiter for limiting acceleration while allowing to
            // brake instantaneously via ramping
            double dt = 0.02;
            double finalX, finalY, finalOmega;

            if (assistEnabled && slewRateEnabled) {
                double accelLimit = isShooting ? Constants.DriveAssistConstants.SHOOTING_ACCEL_LIMIT
                        : Constants.DriveAssistConstants.NORMAL_ACCEL_LIMIT;
                double rotAccelLimit = isShooting ? Constants.DriveAssistConstants.SHOOTING_ROT_ACCEL_LIMIT
                        : Constants.DriveAssistConstants.NORMAL_ROT_ACCEL_LIMIT;
                double brakingLimit = Constants.DriveAssistConstants.BRAKING_ACCEL_LIMIT;

                double xLimit = (Math.abs(targetX) < Math.abs(m_lastX)) ? brakingLimit : accelLimit;
                finalX = m_lastX + MathUtil.clamp(targetX - m_lastX, -xLimit * dt, xLimit * dt);

                double yLimit = (Math.abs(targetY) < Math.abs(m_lastY)) ? brakingLimit : accelLimit;
                finalY = m_lastY + MathUtil.clamp(targetY - m_lastY, -yLimit * dt, yLimit * dt);

                double omegaLimit = (Math.abs(targetOmega) < Math.abs(m_lastOmega)) ? brakingLimit : rotAccelLimit;
                finalOmega = m_lastOmega + MathUtil.clamp(targetOmega - m_lastOmega, -omegaLimit * dt, omegaLimit * dt);
            } else {
                finalX = targetX;
                finalY = targetY;
                finalOmega = targetOmega;
            }

            // limiting driver rotation so that the bot heading can not surpass the turret
            // range only while autofiring to hub and not when dumping
            if (isShooting && !isDumping && assistEnabled && shootingRotLockEnabled) {
                double desiredTurretAngle = desiredTurretAngleSupplier.getAsDouble();
                double turretMargin = 5.0;
                double minTurret = Constants.TurretConstants.kMinimumAngle + turretMargin;
                double maxTurret = Constants.TurretConstants.kMaximumAngle - turretMargin;

                // if past positive limit, then uses negative omega or clockwise to correct
                if (desiredTurretAngle > maxTurret) {
                    if (finalOmega >= 0) {
                        double targetDirRad = pose.getRotation().getRadians()
                                - Math.toRadians(desiredTurretAngle);
                        m_rotationController.setSetpoint(targetDirRad + Math.toRadians(maxTurret));
                        finalOmega = MathUtil.clamp(
                                m_rotationController.calculate(pose.getRotation().getRadians()),
                                -Math.PI, Math.PI);
                        if (m_rotationController.atSetpoint())
                            finalOmega = 0;
                    }
                    //
                } else if (desiredTurretAngle < minTurret) {
                    // the opposite for this one
                    if (finalOmega <= 0) {
                        double targetDirRad = pose.getRotation().getRadians()
                                - Math.toRadians(desiredTurretAngle);
                        m_rotationController.setSetpoint(targetDirRad + Math.toRadians(minTurret));
                        finalOmega = MathUtil.clamp(
                                m_rotationController.calculate(pose.getRotation().getRadians()),
                                -Math.PI, Math.PI);
                        if (m_rotationController.atSetpoint())
                            finalOmega = 0;
                    }
                }
            }

            // yeahhhh i kinda borrowed this from hammerheads
            switch (m_currentAssistMode) {
                // for trench, it chooses the nearest 90 deg angle (-90, 0, 90, 180) in case we
                // wanna shovel, and locks the y axis (i got the y values from choreo)
                case DISABLED:
                    double curDeg = pose.getRotation().getDegrees();
                    double closest = 0;
                    double minDiff = Math.abs(MathUtil.inputModulus(curDeg - 0, -180, 180));
                    for (double t : new double[] { 90, -90 }) {
                        double d = Math.abs(MathUtil.inputModulus(curDeg - t, -180, 180));
                        if (d < minDiff) {
                            minDiff = d;
                            closest = t;
                        }
                    }
                    m_rotationController.setSetpoint(Math.toRadians(closest));
                    finalOmega = MathUtil.clamp(m_rotationController.calculate(pose.getRotation().getRadians()),
                            -Math.PI, Math.PI);
                    if (m_rotationController.atSetpoint())
                        finalOmega = 0;

                    boolean isTopTrench = pose.getY() >= 4.1055;
                    double trenchY;
                    if (isTopTrench) {
                        if (Math.abs(MathUtil.inputModulus(closest - (-90), -180, 180)) < 1.0) {
                            trenchY = 7.320613861083984; // -90 deg
                        } else if (Math.abs(MathUtil.inputModulus(closest - 90, -180, 180)) < 1.0) {
                            trenchY = 7.531105995178223; // +90 deg
                        } else {
                            trenchY = 7.437553882598877; // 0 or 180 deg
                        }
                    } else {
                        if (Math.abs(MathUtil.inputModulus(closest - (-90), -180, 180)) < 1.0) {
                            trenchY = 0.5396772623062134; // -90 deg
                        } else if (Math.abs(MathUtil.inputModulus(closest - 90, -180, 180)) < 1.0) {
                            trenchY = 0.759850263595581; // +90 deg
                        } else {
                            trenchY = 0.6595473289489746; // 0 or 180 deg
                        }
                    }

                    m_debugTrenchSetpoint = trenchY;
                    m_debugTrenchHeading = closest;
                    m_debugIsTopTrench = isTopTrench;
                    m_debugTrenchCurrentY = pose.getY();

                    m_translationController.setSetpoint(trenchY);
                    double rawPidOutput = m_translationController.calculate(pose.getY());
                    m_debugPidOutput = rawPidOutput;

                    boolean isRedAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                    double yOutput = isRedAlliance ? -rawPidOutput : rawPidOutput;
                    finalY = MathUtil.clamp(yOutput, -1.5, 1.5);

                    if (m_translationController.atSetpoint())
                        finalY = 0;

                    m_targetPose = new Pose2d(pose.getX(), trenchY, Rotation2d.fromDegrees(closest));
                    break;

                // for bump, it chooses the nearest 45 deg angle but doesnt lock translation
                case BUMP_LOCK:
                    double closestBump = Math.round((pose.getRotation().getDegrees() - 45.0) / 90.0) * 90.0 + 45.0;
                    m_rotationController.setSetpoint(Math.toRadians(closestBump));
                    finalOmega = MathUtil.clamp(m_rotationController.calculate(pose.getRotation().getRadians()),
                            -Math.PI, Math.PI);
                    if (m_rotationController.atSetpoint())
                        finalOmega = 0;

                    m_targetPose = new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(closestBump));
                    break;

                default:
                    m_targetPose = pose;
                    break;
            }

            m_lastX = finalX;
            m_lastY = finalY;
            m_lastOmega = finalOmega;

            setControl(m_fieldCentricDrive
                    .withVelocityX(finalX)
                    .withVelocityY(finalY)
                    .withRotationalRate(finalOmega)
                    .withDeadband(MaxSpeed.getAsDouble() * OIConstants.kDeadband)
                    .withRotationalDeadband(MaxAngularRate.getAsDouble() * OIConstants.kDeadband) // Add a 10% deadband
            );
        });
    }

}
