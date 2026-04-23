package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ShotData;
import frc.robot.Constants.TurretConstants;

public class ShotCalculator {

    /**
     * calcs the exact field-relative position of the turret at time t while accounting for turret offset using a circular arc
     * 
     * calcs means calculates btw for anyone new to the chat
     *
     * @param robotPose   current robot pose
     * @param fieldSpeeds robot velocity relative to field
     * @param t           time in seconds into the future
     * @return future turret position
     */
    public static Translation2d futureTurretPosition(Pose2d robotPose, ChassisSpeeds fieldSpeeds, double t) {
        double omega = fieldSpeeds.omegaRadiansPerSecond;
        double vx = fieldSpeeds.vxMetersPerSecond;
        double vy = fieldSpeeds.vyMetersPerSecond;


        double futureRobotX = robotPose.getX() + vx * t;
        double futureRobotY = robotPose.getY() + vy * t;

        double futureAngle = robotPose.getRotation().getRadians() + omega * t;
        Translation2d futureTurretOffset = TurretConstants.ROBOT_TO_TURRET_TRANSFORM
                .getTranslation().toTranslation2d()
                .rotateBy(new Rotation2d(futureAngle));

        return new Translation2d(
                futureRobotX + futureTurretOffset.getX(),
                futureRobotY + futureTurretOffset.getY());
    }

    // -----------------------------------------------------------------------
    // old linear predict target position method
    // -----------------------------------------------------------------------
    // /**
    // * predicts the target position in the future based on robot velocity and time
    // * of flight
    // *
    // * @param target center of the hub
    // * @param fieldSpeeds robot speed relative to field
    // * @param timeOfFlight time of flight for ball to reach target
    // * @return predicted target position relative to the field
    // */
    // public static Translation3d predictTargetPos(Translation3d target,
    // ChassisSpeeds fieldSpeeds, double timeOfFlight, Pose2d robotPose) {
    // Translation2d turretFieldOffset =
    // getTurretTranslation(robotPose).minus(robotPose.getTranslation());
    //
    // double vxRot = -fieldSpeeds.omegaRadiansPerSecond * turretFieldOffset.getY();
    // double vyRot = fieldSpeeds.omegaRadiansPerSecond * turretFieldOffset.getX();
    //
    // double totalVx = fieldSpeeds.vxMetersPerSecond + vxRot;
    // double totalVy = fieldSpeeds.vyMetersPerSecond + vyRot;
    //
    // double predictedX = target.getX() - totalVx * timeOfFlight;
    // double predictedY = target.getY() - totalVy * timeOfFlight;
    // return new Translation3d(predictedX, predictedY, target.getZ());
    // }

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

        // -----------------------------------------------------------------------
        // old secant method
        // -----------------------------------------------------------------------
        // // secant method root finding inspired by red rock robotics
        // Translation2d turretPos = getTurretTranslation(robotPose);
        //
        // double t0 = 0.0;
        // double ft0 = isDumping ?
        // ShooterConstants.getTOF(turretPos.getDistance(target.toTranslation2d()))
        // : ShooterConstants.getTOF(turretPos.getDistance(target.toTranslation2d()));
        //
        // double t1 = ft0;
        // Translation3d target1 = predictTargetPos(target, fieldSpeeds, t1, robotPose);
        // double ft1 = isDumping ?
        // ShooterConstants.getTOF(turretPos.getDistance(target1.toTranslation2d()))
        // : ShooterConstants.getTOF(turretPos.getDistance(target1.toTranslation2d()));
        //
        // Translation3d predictedTarget = target1;
        //
        // for (int i = 0; i < iterations; i++) {
        // if (Math.abs(t1 - t0) < 1e-5) {
        // break;
        // }
        //
        // double f0 = ft0 - t0;
        // double f1 = ft1 - t1;
        //
        // if (Math.abs(f1 - f0) < 1e-5) {
        // break;
        // }
        //
        // double t2 = t1 - f1 * (t1 - t0) / (f1 - f0);
        //
        // t0 = t1;
        // ft0 = ft1;
        // t1 = t2;
        //
        // predictedTarget = predictTargetPos(target, fieldSpeeds, t1, robotPose);
        // ft1 = isDumping ?
        // ShooterConstants.getTOF(turretPos.getDistance(predictedTarget.toTranslation2d()))
        // :
        // ShooterConstants.getTOF(turretPos.getDistance(predictedTarget.toTranslation2d()));
        // }
        //
        // double finalDistance =
        // turretPos.getDistance(predictedTarget.toTranslation2d());
        // ShotData shot = isDumping ? ShooterConstants.getShotData(finalDistance)
        // : ShooterConstants.getShotData(finalDistance);
        //
        // return new CalculatedShot(shot, predictedTarget);



        // mr newt finds the derivative of f(t) = tof(dist(t)) - t

        // seed in the parameters for a static shot so we can guess tof
        Translation2d turretPos = getTurretTranslation(robotPose);
        double staticDist = turretPos.getDistance(target.toTranslation2d());
        double t = ShooterConstants.getTOF(staticDist);

        for (int i = 0; i < iterations; i++) {
            Translation2d futureTurret = futureTurretPosition(robotPose, fieldSpeeds, t);
            double dist = futureTurret.getDistance(target.toTranslation2d());

            double tofAtDist = ShooterConstants.getTOF(dist);
            double fVal = tofAtDist - t;


            double dTOF_dDist = 2.0 * ShooterConstants.TOF_A * dist
                    + ShooterConstants.TOF_B;

            double futureAngle = robotPose.getRotation().getRadians()
                    + fieldSpeeds.omegaRadiansPerSecond * t;
            double r = TurretConstants.ROBOT_TO_TURRET_TRANSFORM
                    .getTranslation().toTranslation2d().getNorm();
            double omega = fieldSpeeds.omegaRadiansPerSecond;
            double vx = fieldSpeeds.vxMetersPerSecond;
            double vy = fieldSpeeds.vyMetersPerSecond;

            double turretVx = vx - r * omega * Math.sin(futureAngle);
            double turretVy = vy + r * omega * Math.cos(futureAngle);

            double dx = futureTurret.getX() - target.getX();
            double dy = futureTurret.getY() - target.getY();

            double dDist_dt = (dx * turretVx + dy * turretVy) / dist;

            double fPrime = dTOF_dDist * dDist_dt - 1.0;

            // DONT DIVIDE BY ZERO!!!!
            if (Math.abs(fPrime) < 1e-6)
                break;

            double step = fVal / fPrime;
            t = t - step;
            
            // if its good enough then just stop bro
            if (Math.abs(step) < 1e-4)
                break;
        }

        // compute final shot data from the converged future turret position
        Translation2d futureTurret = futureTurretPosition(robotPose, fieldSpeeds, t);
        double finalDist = futureTurret.getDistance(target.toTranslation2d());
        ShotData shot = ShooterConstants.getShotData(finalDist);
        return new CalculatedShot(shot, target, futureTurret);
    }

    // sorry jacob i commented yo thing out

    // public static CalculatedShot calculateNewtonShot(
    // Pose2d robotPose,
    // ChassisSpeeds fieldSpeeds,
    // Translation3d target,
    // int iterations,
    // boolean isDumping) {
    //
    // double distance =
    // getTurretTranslation(robotPose).getDistance(target.toTranslation2d());
    //
    // // initial estimate
    // // ShotData shot = isDumping ? ShooterConstants.DUMP_MAP.get(distance) :
    //
    // // lets try dump data points
    // // ShotData shot = isDumping ? ShooterConstants.DUMP_MAP.get(distance) :
    // ShooterConstants.getShotData(distance);
    // // double timeOfFlight = isDumping ?
    // ShooterConstants.DUMP_TOF_MAP.get(distance)
    // // : ShooterConstants.getTOF(distance);
    //
    // // if dump data points are bad then just treat it as a hub
    // ShotData shot = isDumping ? ShooterConstants.getShotData(distance) :
    // ShooterConstants.getShotData(distance);
    // double timeOfFlight = isDumping ? ShooterConstants.getTOF(distance) :
    // ShooterConstants.getTOF(distance);
    //
    // Translation3d predictedTarget = target;
    //
    // double newDist = 0.0f;
    //
    // // iterative lookahead
    // for (int i = 0; i < iterations; i++) {
    // predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);
    //
    // newDist =
    // getTurretTranslation(robotPose).getDistance(predictedTarget.toTranslation2d());
    //
    // // h = 1 because discrete iteration
    // double deriv = newDist - distance;
    // timeOfFlight = timeOfFlight - (distance / deriv);
    //
    // // // if dump data points are bad then just treat it as a hub
    // // shot = isDumping ? ShooterConstants.getShotData(distance) :
    // // ShooterConstants.getShotData(distance);
    //
    // // newTOF = isDumping ? ShooterConstants.getTOF(distance) :
    // ShooterConstants.getTOF(distance);
    //
    // }
    //
    // return new CalculatedShot(shot, predictedTarget);
    // }


    public record CalculatedShot(ShotData shot, Translation3d target, Translation2d futureTurretPos) {
    }


    public static Translation2d getTurretTranslation(Pose2d robotPose) {
        return robotPose.getTranslation().plus(
                TurretConstants.ROBOT_TO_TURRET_TRANSFORM.getTranslation().toTranslation2d()
                        .rotateBy(robotPose.getRotation()));
    }


    public static Rotation2d getTargetRotation(Pose2d robotPose, Translation3d target) {
        Translation2d direction = target.toTranslation2d().minus(getTurretTranslation(robotPose));
        return direction.getAngle();
    }


    public static Rotation2d getTargetRotation(Translation2d turretOrigin, Translation3d target) {
        Translation2d direction = target.toTranslation2d().minus(turretOrigin);
        return direction.getAngle();
    }
}
