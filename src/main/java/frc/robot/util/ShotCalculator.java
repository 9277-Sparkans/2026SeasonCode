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
    public static Translation3d predictTargetPos(Translation3d target, ChassisSpeeds fieldSpeeds, double timeOfFlight, Pose2d robotPose) {
        Translation2d turretFieldOffset = getTurretTranslation(robotPose).minus(robotPose.getTranslation());
        
        double vxRot = -fieldSpeeds.omegaRadiansPerSecond * turretFieldOffset.getY();
        double vyRot = fieldSpeeds.omegaRadiansPerSecond * turretFieldOffset.getX();
        
        double totalVx = fieldSpeeds.vxMetersPerSecond + vxRot;
        double totalVy = fieldSpeeds.vyMetersPerSecond + vyRot;

        double predictedX = target.getX() - totalVx * timeOfFlight;
        double predictedY = target.getY() - totalVy * timeOfFlight;
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

        /*----------------------------------
         *  old fixed point iteration method
         *----------------------------------
        double distance = getTurretTranslation(robotPose).getDistance(target.toTranslation2d());

        // initial estimate
        // ShotData shot = isDumping ? ShooterConstants.DUMP_MAP.get(distance) :

        // lets try dump data points
        // ShotData shot = isDumping ? ShooterConstants.DUMP_MAP.get(distance) :
        // ShooterConstants.getShotData(distance);
        // double timeOfFlight = isDumping ? ShooterConstants.DUMP_TOF_MAP.get(distance)
        // : ShooterConstants.getTOF(distance);

        // if dump data points are bad then just treat it as a hub
        ShotData shot = isDumping ? ShooterConstants.getShotData(distance) : ShooterConstants.getShotData(distance);
        double timeOfFlight = isDumping ? ShooterConstants.getTOF(distance) : ShooterConstants.getTOF(distance);

        Translation3d predictedTarget = target;

        // iterative lookahead
        for (int i = 0; i < iterations; i++) {
            predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);
            distance = getTurretTranslation(robotPose).getDistance(predictedTarget.toTranslation2d());

            // lets try dump data points
            // shot = isDumping ? ShooterConstants.DUMP_MAP.get(distance) :
            // ShooterConstants.getShotData(distance);
            // timeOfFlight = isDumping ? ShooterConstants.DUMP_TOF_MAP.get(distance) :
            // ShooterConstants.getTOF(distance);

            // if dump data points are bad then just treat it as a hub
            shot = isDumping ? ShooterConstants.getShotData(distance) : ShooterConstants.getShotData(distance);
            timeOfFlight = isDumping ? ShooterConstants.getTOF(distance) : ShooterConstants.getTOF(distance);
        }
        */

        // secant method root finding inspired by red rock robotics
        Translation2d turretPos = getTurretTranslation(robotPose);

        double t0 = 0.0;
        double ft0 = isDumping ? ShooterConstants.getTOF(turretPos.getDistance(target.toTranslation2d())) 
                               : ShooterConstants.getTOF(turretPos.getDistance(target.toTranslation2d()));

        double t1 = ft0;
        Translation3d target1 = predictTargetPos(target, fieldSpeeds, t1, robotPose);
        double ft1 = isDumping ? ShooterConstants.getTOF(turretPos.getDistance(target1.toTranslation2d()))
                               : ShooterConstants.getTOF(turretPos.getDistance(target1.toTranslation2d()));

        Translation3d predictedTarget = target1;

        for (int i = 0; i < iterations; i++) {
            if (Math.abs(t1 - t0) < 1e-5) {
                break;
            }

            double f0 = ft0 - t0;
            double f1 = ft1 - t1;

            if (Math.abs(f1 - f0) < 1e-5) {
                break;
            }

            double t2 = t1 - f1 * (t1 - t0) / (f1 - f0);

            t0 = t1;
            ft0 = ft1;
            t1 = t2;

            predictedTarget = predictTargetPos(target, fieldSpeeds, t1, robotPose);
            ft1 = isDumping ? ShooterConstants.getTOF(turretPos.getDistance(predictedTarget.toTranslation2d()))
                            : ShooterConstants.getTOF(turretPos.getDistance(predictedTarget.toTranslation2d()));
        }

        double finalDistance = turretPos.getDistance(predictedTarget.toTranslation2d());
        ShotData shot = isDumping ? ShooterConstants.getShotData(finalDistance) 
                                  : ShooterConstants.getShotData(finalDistance);

        return new CalculatedShot(shot, predictedTarget);
    }

    public static CalculatedShot calculateNewtonShot(
            Pose2d robotPose,
            ChassisSpeeds fieldSpeeds,
            Translation3d target,
            int iterations,
            boolean isDumping) {

        double distance = getTurretTranslation(robotPose).getDistance(target.toTranslation2d());

        // initial estimate
        // ShotData shot = isDumping ? ShooterConstants.DUMP_MAP.get(distance) :

        // lets try dump data points
        // ShotData shot = isDumping ? ShooterConstants.DUMP_MAP.get(distance) : ShooterConstants.getShotData(distance);
        // double timeOfFlight = isDumping ? ShooterConstants.DUMP_TOF_MAP.get(distance)
        //         : ShooterConstants.getTOF(distance);

        // if dump data points are bad then just treat it as a hub
        ShotData shot = isDumping ? ShooterConstants.getShotData(distance) :
        ShooterConstants.getShotData(distance);
        double timeOfFlight = isDumping ? ShooterConstants.getTOF(distance) :
        ShooterConstants.getTOF(distance);

        Translation3d predictedTarget = target;

        double newDist = 0.0f;

        // iterative lookahead
        for (int i = 0; i < iterations; i++) {
            predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);

            newDist = getTurretTranslation(robotPose).getDistance(predictedTarget.toTranslation2d());
            
            // h = 1 because discrete iteration
            double deriv = newDist - distance;
            timeOfFlight = timeOfFlight - (distance / deriv);

            // // if dump data points are bad then just treat it as a hub
            // shot = isDumping ? ShooterConstants.getShotData(distance) :
            // ShooterConstants.getShotData(distance);

            // newTOF = isDumping ? ShooterConstants.getTOF(distance) : ShooterConstants.getTOF(distance); 

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
                TurretConstants.ROBOT_TO_TURRET_TRANSFORM.getTranslation().toTranslation2d()
                        .rotateBy(robotPose.getRotation()));
    }

    /**
     * Calculates the field-relative azimuth angle to a 3D target.
     */
    public static Rotation2d getTargetRotation(Pose2d robotPose, Translation3d target) {
        Translation2d direction = target.toTranslation2d().minus(getTurretTranslation(robotPose));
        return direction.getAngle();
    }
}
