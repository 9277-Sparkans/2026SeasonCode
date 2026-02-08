package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem that reads game piece (fuel) detection data from the Jetson's
 * custom YOLO pipeline via NetworkTables.
 * 
 * The Jetson publishes to NetworkTables table "GamePiece" with entries:
 * - targetX: horizontal offset in degrees (~-50 to +50 for 100° FOV)
 * - targetY: vertical offset in degrees
 * - targetArea: area in pixels (larger = closer)
 * - targetValid: true if game piece detected
 * 
 * NOTE: These entry names intentionally differ from Limelight's tx/ty/ta/tv
 * to avoid conflicts if running both systems.
 */
public class FuelVision extends SubsystemBase {

    private static final String TABLE_NAME = "GamePiece";

    private final NetworkTable table;
    private final DoubleSubscriber targetXSubscriber;
    private final DoubleSubscriber targetYSubscriber;
    private final DoubleSubscriber targetAreaSubscriber;
    private final BooleanSubscriber targetValidSubscriber;

    // Cached values updated each periodic cycle
    private double targetX = 0.0;
    private double targetY = 0.0;
    private double targetArea = 0.0;
    private boolean targetValid = false;

    public FuelVision() {
        table = NetworkTableInstance.getDefault().getTable(TABLE_NAME);

        // Create subscribers with default values
        // Using distinct names to avoid collision with Limelight's tx/ty/ta/tv
        targetXSubscriber = table.getDoubleTopic("targetX").subscribe(0.0);
        targetYSubscriber = table.getDoubleTopic("targetY").subscribe(0.0);
        targetAreaSubscriber = table.getDoubleTopic("targetArea").subscribe(0.0);
        targetValidSubscriber = table.getBooleanTopic("targetValid").subscribe(false);
    }

    @Override
    public void periodic() {
        // Update cached values from NetworkTables
        targetX = targetXSubscriber.get();
        targetY = targetYSubscriber.get();
        targetArea = targetAreaSubscriber.get();
        targetValid = targetValidSubscriber.get();

        // Log for debugging via AdvantageKit
        Logger.recordOutput("GamePieceVision/TargetX", targetX);
        Logger.recordOutput("GamePieceVision/TargetY", targetY);
        Logger.recordOutput("GamePieceVision/TargetArea", targetArea);
        Logger.recordOutput("GamePieceVision/HasTarget", targetValid);
    }

    /**
     * @return true if the Jetson sees a valid game piece target
     */
    public boolean hasTarget() {
        return targetValid;
    }

    /**
     * @return horizontal offset in degrees (negative = target to the left)
     */
    public double getTargetX() {
        return targetX;
    }

    /**
     * @return vertical offset in degrees (negative = target below center)
     */
    public double getTargetY() {
        return targetY;
    }

    /**
     * @return target area in pixels (larger = closer)
     */
    public double getTargetArea() {
        return targetArea;
    }
}
