package frc.robot;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;
import frc.robot.generated.TunerConstants;

public class Utils {
    public static double clamp(double value, double min, double max) {
        if (value > max) {
            return max;
        }

        if (value < min) {
            return min;
        }

        return value;
    }

    public static double dist2d(double x1, double y1, double x2, double y2) {
        return Math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    }

    public static double wrapAngle(double angle) {
        angle = angle % 360.0;
        
        if (angle > 180.0) {
            angle -= 360.0;
        } else if (angle < -180.0) {
            angle += 360.0;
        }

        return angle;
    }

    public static class Lookup {
        private final ArrayList<double[]> hits = new ArrayList<>(); // [Landing Distance, Landing Direction]
        private final ArrayList<double[]> vals = new ArrayList<>(); // [Bot Speed, Bot Direction, Shooter RPM, Hood Angle]
        private final int size;

        public Lookup() { // Load lookup table into lists
            try (BufferedReader br = new BufferedReader(new FileReader(Constants.ShooterConstants.lookupTablePath))) {
                br.readLine(); // Skip header line

                // Read the CSV and save as lists
                String line;
                while ((line = br.readLine()) != null) {
                    String[] values = line.split(",");

                    double landingDistance = Double.parseDouble(values[0]);
                    double landingDirection = Double.parseDouble(values[1]);

                    double botSpeed = Double.parseDouble(values[2]);
                    double botDirection = Double.parseDouble(values[3]);
                    double shooterRpm = Double.parseDouble(values[4]);
                    double hoodAngle = Double.parseDouble(values[5]);
                    
                    hits.add(new double[]{landingDistance, landingDirection});
                    vals.add(new double[]{botSpeed, botDirection, shooterRpm, hoodAngle});
                }
            } catch (Exception e) {
                System.out.println("Error forming lookup table");
            }

            size = hits.size();
        }

        public int GetClosestDist(double distance) { // Runs a binary search to find the row with the closest distance to the specified
            // Initial bounds
            int leftBound = 0;
            int rightBound = size - 1;

            // Binary search
            int i = (leftBound + rightBound) / 2;
            while (leftBound <= rightBound) {
                double currentVal = hits.get(i)[0];

                if (currentVal < distance) {
                    leftBound = i + 1;
                } else if (currentVal > distance) {
                    rightBound = i - 1;
                } else {
                    return i;
                }

                i = (int)((leftBound + rightBound) / 2);
            }

            // Choose result
            if (rightBound < 0) {
                return 0;
            }
            if (leftBound >= size) {
                return size - 1;
            }

            if (Math.abs(hits.get(leftBound)[0] - distance) < Math.abs(hits.get(rightBound)[0] - distance)) {
                return leftBound;
            } else {
                return rightBound;
            }
        }

        public double[] FindOptimalVals(double distance, double velocityX, double velocityY, double shooterRpm, double hoodAngle) { // Finds the optimal shot for minimum hood movement and RPM change
            // Get range
            int startI = GetClosestDist(distance - Constants.ShooterConstants.autoshootDistanceRange);
            int endI = GetClosestDist(distance + Constants.ShooterConstants.autoshootDistanceRange);

            // Helper values
            double botSpeedRange = 2.0 * TunerConstants.kSpeedAt12Volts.magnitude();
            double shooterRpmRange = (double)(Constants.ShooterConstants.kMaxRPM - Constants.ShooterConstants.kMinRPM);
            double hoodAngleRange = Constants.HoodConstants.kMaximumAngle - Constants.HoodConstants.kMinimumAngle;

            double normalizedCurrentXVelocity= velocityX / botSpeedRange;
            double normalizedCurrentYVelocity = velocityY / botSpeedRange;
            double normalizedCurrentShooterRPM = (shooterRpm - Constants.ShooterConstants.kMinRPM) / shooterRpmRange;
            double normalizedCurrentHoodAngle = (hoodAngle - Constants.HoodConstants.kMinimumAngle) / hoodAngleRange;

            // Get least square distance between current RPM and Angle vs desired
            double minimumWeight = Double.POSITIVE_INFINITY;
            int minimumI = 0;
            for (int i = startI; i <= endI; i++) {
                double[] row = vals.get(i);

                double normalizedRowBotXVelocity = row[0] / botSpeedRange;
                double normalizedRowBotYVelocity = row[1] / botSpeedRange;
                double normalizedRowShooterRpm = (row[2] - Constants.ShooterConstants.kMinRPM) / shooterRpmRange;
                double normalizedRowAngle = (row[3] - Constants.HoodConstants.kMinimumAngle) / hoodAngleRange;

                double weight = Constants.ShooterConstants.botXVelocityWeight * (normalizedRowBotXVelocity - normalizedCurrentXVelocity)  * (normalizedRowBotXVelocity - normalizedCurrentXVelocity)
                              + Constants.ShooterConstants.botYVelocityWeight * (normalizedRowBotYVelocity - normalizedCurrentYVelocity) * (normalizedRowBotYVelocity - normalizedCurrentYVelocity)
                              + Constants.ShooterConstants.shooterRpmWeight * (normalizedRowShooterRpm - normalizedCurrentShooterRPM) * (normalizedRowShooterRpm - normalizedCurrentShooterRPM)
                              + Constants.ShooterConstants.hoodAngleWeight * (normalizedRowAngle - normalizedCurrentHoodAngle) * (normalizedRowAngle - normalizedCurrentHoodAngle);
                if (weight < minimumWeight) {
                    minimumWeight = weight;
                    minimumI = i;
                }
            }

            double optimalTurretOffset = hits.get(minimumI)[1];
            double optimalShooterRpm = vals.get(minimumI)[2];
            double optimalHoodAngle = vals.get(minimumI)[3];
            return new double[]{minimumWeight, optimalTurretOffset, optimalShooterRpm, optimalHoodAngle};
        }
    }

    public static Lookup createLookup() { // Create a Lookup object
        return new Lookup();
    }
}