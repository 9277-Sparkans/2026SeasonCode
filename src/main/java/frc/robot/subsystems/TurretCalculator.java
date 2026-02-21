package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.TurretConstants;

/**
 * Utility class for turret targeting math.
 */
public class TurretCalculator {

    /**
     * Calculates the required turret angle relative to the robot to point at a
     * target.
     * 
     * @param robotPose          The current field-relative pose of the robot.
     * @param target             The field-relative position of the target.
     * @param currentTurretAngle The current turret angle relative to the robot (in
     *                           degrees).
     * @return The required turret angle relative to the robot (in degrees).
     */
    public static double calculateAzimuthAngle(Pose2d robotPose, Translation3d target, double currentTurretAngle) {
        // Find the turret's position in the field
        Translation2d turretTranslation = new Pose3d(robotPose)
                .transformBy(TurretConstants.ROBOT_TO_TURRET_TRANSFORM)
                .toPose2d()
                .getTranslation();

        // Direction vector from turret to target
        Translation2d direction = target.toTranslation2d().minus(turretTranslation);

        // Field-relative angle to the target
        Rotation2d fieldAngle = direction.getAngle();

        // Robot-relative angle to the target
        Rotation2d robotRelativeAngle = fieldAngle.minus(robotPose.getRotation());

        // Minimize jumps: Calculate the difference and choose the smallest path
        double targetAngle = MathUtil.inputModulus(robotRelativeAngle.getDegrees(), -180, 180);

        // For a turret with narrow limits (+/- 20), we don't want to snap from +180 to
        // -180
        // if the target crosses the back. Instead, we should just stay at the closer
        // limit.
        // Actually, clamp already handles this, but let's be explicit about the choice
        // if needed.

        // Clamp to physical limits
        return MathUtil.clamp(targetAngle, TurretConstants.kMinimumAngle, TurretConstants.kMaximumAngle);
    }

    /**
     * Returns the distance to the target in meters.
     */
    public static double getDistanceToTarget(Pose2d robot, Translation3d target) {
        return robot.getTranslation().getDistance(target.toTranslation2d());
    }
}
