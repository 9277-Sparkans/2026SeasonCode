package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.function.Supplier;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;

import frc.robot.Telemetry;

import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {

  private final TalonFX turretMotor;
  private final CANcoder turretEncoder;
  private final TalonFXConfiguration turretMotorConfig;
  public double turretOffset = 0.0;

  private Supplier<Pose2d> poseSupplier;
  private Supplier<ChassisSpeeds> speedsSupplier;

  private final LinearFilter angleFilter = LinearFilter.singlePoleIIR(0.1, 0.02); // 0.1s time constant, 0.02s loop
  private double filteredTargetAngle = 0;
  private boolean isTracking = false;
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0.0);

  /** Creates a new Turret. */
  public Turret(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier) {
    this.poseSupplier = poseSupplier;
    this.speedsSupplier = speedsSupplier;
    turretMotor = new TalonFX(TurretConstants.turret_motorId);
    turretEncoder = new CANcoder(TurretConstants.kTurretEncoderId);
    turretMotorConfig = new TalonFXConfiguration();

    // Configure CANcoder
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset = TurretConstants.kTurretEncoderOffset;
    turretEncoder.getConfigurator().apply(encoderConfig);

    // Total gear ratio is kGearRatio * 5
    double totalGearRatio = TurretConstants.kGearRatio * 5.0;

    // Seed motor position from absolute encoder
    // Assuming encoder is 1:1 with the turret final axis
    double absolutePosition = turretEncoder.getAbsolutePosition().waitForUpdate(0.1).getValueAsDouble();
    // Normalize absolute position to [-0.5, 0.5] to prevent seeding jumps
    absolutePosition = edu.wpi.first.math.MathUtil.inputModulus(absolutePosition, -0.5, 0.5);
    turretMotor.setPosition(absolutePosition * totalGearRatio);

    turretMotorConfig.Slot0.kS = TurretConstants.turret_kS;
    turretMotorConfig.Slot0.kV = TurretConstants.turret_kV;
    turretMotorConfig.Slot0.kA = TurretConstants.turret_kA;
    turretMotorConfig.Slot0.kP = TurretConstants.turret_kP;
    turretMotorConfig.Slot0.kI = TurretConstants.turret_kI;
    turretMotorConfig.Slot0.kD = TurretConstants.turret_kD;

    turretMotorConfig.Voltage.PeakForwardVoltage = TurretConstants.turret_maxVoltage;
    turretMotorConfig.Voltage.PeakReverseVoltage = -TurretConstants.turret_maxVoltage;
    turretMotorConfig.MotionMagic.MotionMagicAcceleration = TurretConstants.turret_maxAcceleration;
    turretMotorConfig.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.turret_maxVelocity;
    turretMotorConfig.MotionMagic.MotionMagicJerk = TurretConstants.turret_maxJerk;

    // Total gear ratio is kGearRatio * 5 (based on user's original turretMoveTgt
    // math)
    // totalGearRatio is already defined above

    turretMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.kMaximumAngle / 360.0
        * totalGearRatio;
    turretMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    turretMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants.kMinimumAngle / 360.0
        * totalGearRatio;
    turretMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    turretMotor.getConfigurator().apply(turretMotorConfig);

    Telemetry.telemeterizeMotorWithPID("Turret", turretMotor, (1.0 / totalGearRatio), turretMotorConfig);

  }

  @Override
  public void periodic() {

  }

  public double getPosition() {
    // Total gear ratio is kGearRatio * 5
    double totalGearRatio = TurretConstants.kGearRatio * 5.0;
    return turretMotor.getPosition().getValueAsDouble() / totalGearRatio;
  }

  public double getTurretAngle() {
    return getPosition() * 360.0;
  }

  public double getVelocity() {
    return turretMotor.getVelocity().getValueAsDouble();
  }

  /**
   * Automatically track the field target.
   * 
   * @param target The target to track in the field.
   */
  public void trackTarget(Translation3d target) {
    Pose2d currentPose = poseSupplier.get();
    double targetAngleDegrees = TurretCalculator.calculateAzimuthAngle(
        currentPose,
        target,
        getPosition());

    // Smoothing: filter the target angle to prevent jerky movements
    filteredTargetAngle = angleFilter.calculate(targetAngleDegrees);
    isTracking = true;

    // Convert degrees to motor rotations using total gear ratio
    double totalGearRatio = TurretConstants.kGearRatio * 5.0;
    double targetMotorRotations = (filteredTargetAngle * totalGearRatio) / 360.0;

    // Add counter-rotation feedforward
    // Filter the robot's omega to prevent noise from affecting turret stability
    double rawOmegaRps = speedsSupplier.get().omegaRadiansPerSecond / (2 * Math.PI);
    double feedforwardVoltage = -rawOmegaRps * TurretConstants.turret_kV * totalGearRatio;

    turretMotor.setControl(m_request
        .withPosition(targetMotorRotations)
        .withFeedForward(feedforwardVoltage));

    // Logging for verification
    Logger.recordOutput("Turret/IsTracking", isTracking);
    Logger.recordOutput("Turret/RawTargetAngle", targetAngleDegrees);
    Logger.recordOutput("Turret/FilteredTargetAngle", filteredTargetAngle);
    Logger.recordOutput("Turret/DrivetrainPose", currentPose);
  }

  public void stop() {
    isTracking = false;
    angleFilter.reset();
    turretMotor.setControl(new com.ctre.phoenix6.controls.NeutralOut());
  }

}