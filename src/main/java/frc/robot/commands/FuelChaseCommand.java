package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveRequest;

/**
 * Command to chase and drive toward the nearest fuel ball detected by the
 * Jetson.
 * 
 * Expected NetworkTables structure (adjust keys in constants if needed):
 * - SmartDashboard/Fuel/hasTarget (boolean) - whether a ball is detected
 * - SmartDashboard/Fuel/tx (double) - horizontal angle to target in degrees
 * (positive = right)
 * - SmartDashboard/Fuel/ty (double) - vertical angle to target in degrees
 * (positive = up)
 * - SmartDashboard/Fuel/area (double) - optional, area of detection (larger =
 * closer)
 */
public class FuelChaseCommand extends Command {
  // NetworkTables keys - ADJUST THESE to match your Jetson output
  private static final String NT_TABLE = "SmartDashboard";
  private static final String NT_SUBTABLE = "Fuel";
  private static final String NT_HAS_TARGET = "hasTarget";
  private static final String NT_TX = "tx";
  private static final String NT_TY = "ty";
  private static final String NT_AREA = "area";

  // Tuning constants
  private static final double FORWARD_SPEED = 1.5; // m/s base forward speed
  private static final double ROTATION_KP = 0.03; // P gain for rotation toward target
  private static final double AREA_THRESHOLD = 15.0; // Stop when ball is this close (area)
  private static final double TX_TOLERANCE = 2.0; // Degrees, considered "centered"

  private final CommandSwerveDrivetrain drivetrain;
  private final NetworkTable fuelTable;
  private final SwerveRequest.RobotCentric driveRequest;
  private final PIDController rotationController;

  public FuelChaseCommand(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    this.fuelTable = NetworkTableInstance.getDefault()
        .getTable(NT_TABLE)
        .getSubTable(NT_SUBTABLE);
    this.driveRequest = new SwerveRequest.RobotCentric();
    this.rotationController = new PIDController(ROTATION_KP, 0, 0);
    this.rotationController.setTolerance(TX_TOLERANCE);

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    rotationController.reset();
  }

  @Override
  public void execute() {
    boolean hasTarget = fuelTable.getEntry(NT_HAS_TARGET).getBoolean(false);

    if (!hasTarget) {
      // No target - stop or spin to search
      drivetrain.setControl(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
      return;
    }

    double tx = fuelTable.getEntry(NT_TX).getDouble(0);
    // ty and area are available via fuelTable.getEntry(NT_TY) and NT_AREA if needed

    // Calculate rotation to center on target
    double rotationRate = -rotationController.calculate(tx, 0); // Negative because positive tx = turn right

    // Drive forward toward the ball
    // Optionally scale speed based on how centered we are
    double forwardSpeed = FORWARD_SPEED;
    if (Math.abs(tx) > 15) {
      // If far off-center, slow down to let rotation catch up
      forwardSpeed *= 0.5;
    }

    drivetrain.setControl(driveRequest
        .withVelocityX(forwardSpeed)
        .withVelocityY(0)
        .withRotationalRate(rotationRate));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  @Override
  public boolean isFinished() {
    // End when ball is close enough (large area) or target is lost
    double area = fuelTable.getEntry(NT_AREA).getDouble(0);
    boolean hasTarget = fuelTable.getEntry(NT_HAS_TARGET).getBoolean(false);

    return !hasTarget || area >= AREA_THRESHOLD;
  }
}
