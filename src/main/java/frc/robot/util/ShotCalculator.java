package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ShotData;
import frc.robot.Constants.TurretConstants;

public class ShotCalculator {

    /**
     * predicts the target position in the future based on robot velocity and time
     * of flight
     * 
     * @param target       center of the hub
     * @param fieldSpeeds  robot speed relative to field
     * @param timeOfFlight time of flight for ball to reach target
     * @return predicted target position relative to the field
     */
    public static Translation3d predictTargetPos(Translation3d target, ChassisSpeeds fieldSpeeds, double timeOfFlight) {
        // subtract the robot's movement from the target's relative position
        double predictedX = target.getX() - fieldSpeeds.vxMetersPerSecond * timeOfFlight;
        double predictedY = target.getY() - fieldSpeeds.vyMetersPerSecond * timeOfFlight;
        return new Translation3d(predictedX, predictedY, target.getZ());
    }

    /**
     * iteratively calculates the optimal shot parameters for a moving robot
     * 
     * @param robotPose   current robot pose
     * @param fieldSpeeds robot speed relative to field
     * @param target      center of the hub
     * @param iterations  number of iterations to run
     * @return the calculated data for rpm and hoodangle, and the predicted target
     */
    public static CalculatedShot calculateIterativeShot(
            Pose2d robotPose,
            ChassisSpeeds fieldSpeeds,
            Translation3d target,
            int iterations) {
        return calculateIterativeShot(robotPose, fieldSpeeds, target, iterations, false);
    }

    public static CalculatedShot calculateIterativeShot(
            Pose2d robotPose,
            ChassisSpeeds fieldSpeeds,
            Translation3d target,
            int iterations,
            boolean isDumping) {

        double distance = getTurretTranslation(robotPose).getDistance(target.toTranslation2d());

        // initial estimate
        ShotData shot = isDumping ? ShooterConstants.DUMP_MAP.get(distance) : ShooterConstants.SHOT_MAP.get(distance);
        double timeOfFlight = isDumping ? ShooterConstants.DUMP_TOF_MAP.get(distance) : ShooterConstants.TOF_MAP.get(distance);
        Translation3d predictedTarget = target;

        // iterative lookahead
        for (int i = 0; i < iterations; i++) {
            predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);
            distance = getTurretTranslation(robotPose).getDistance(predictedTarget.toTranslation2d());
            shot = isDumping ? ShooterConstants.DUMP_MAP.get(distance) : ShooterConstants.SHOT_MAP.get(distance);
            timeOfFlight = isDumping ? ShooterConstants.DUMP_TOF_MAP.get(distance) : ShooterConstants.TOF_MAP.get(distance);
        }

        return new CalculatedShot(shot, predictedTarget);
    }

    public record CalculatedShot(ShotData shot, Translation3d predictedTarget) {
    }
    
    /**
     * Gets the current field-relative translation of the turret.
     */
    public static Translation2d getTurretTranslation(Pose2d robotPose) {
        return robotPose.getTranslation().plus(
            TurretConstants.ROBOT_TO_TURRET_TRANSFORM.getTranslation().toTranslation2d().rotateBy(robotPose.getRotation())
        );
    }

    /**
     * Calculates the field-relative azimuth angle to a 3D target.
     */
    public static Rotation2d getTargetRotation(Pose2d robotPose, Translation3d target) {
        Translation2d direction = target.toTranslation2d().minus(getTurretTranslation(robotPose));
        return direction.getAngle();
    }
}
