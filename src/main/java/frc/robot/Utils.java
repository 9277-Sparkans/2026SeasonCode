package frc.robot;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;

import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

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

    public static class Lookup {
        private final ArrayList<double[]> hits = new ArrayList<>(); // [Landing Distance, Landing Direction]
        private final ArrayList<double[]> vals = new ArrayList<>(); // [Bot Speed, Bot Direction, Shooter RPM, Hood Angle]
        private final int size;

        private final Shooter shooter;
        private final Hood hood;

        public Lookup(Hood hood, Shooter shooter) { // Load lookup table into lists
            this.hood = hood;
            this.shooter = shooter;

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

        public double[] FindOptimalVals(double distance) { // Finds the optimal shot for minimum hood movement and RPM change
            // Get range
            int bestI = GetClosestDist(distance);
            int startI = bestI;
            int endI = bestI;

            while (distance - hits.get(startI)[0] < Constants.ShooterConstants.autoshootDistanceRange) {
                startI--;
            }
            while (hits.get(endI)[0] - distance < Constants.ShooterConstants.autoshootDistanceRange) {
                endI++;
            }

            // Helper values

            // |-------------------- PLACEHOLDER VALUES --------------------|
            double maxBotSpeed = 10.0; // The maximum (operational) speed the bot can travel m/sa
            double botXVelocity = 0.0; // The bots velocity to or from the target (positive means towards)
            double botYVelocity = 0.0; // The bots velocity to the left or right of the target (positive means left)
            // |-------------------- PLACEHOLDER VALUES --------------------|

            double botSpeedRange = 2.0 * maxBotSpeed;
            double shooterRpmRange = (double)(Constants.ShooterConstants.kMaxRPM - Constants.ShooterConstants.kMinRPM);
            double hoodAngleRange = Constants.HoodConstants.kMaximumAngle - Constants.HoodConstants.kMinimumAngle;

            double normalizedCurrentXVelocity= botXVelocity / botSpeedRange;
            double normalizedCurrentYVelocity = botYVelocity / botSpeedRange;
            double normalizedCurrentShooterRPM = (shooter.GetShooterRPM() - Constants.ShooterConstants.kMinRPM) / shooterRpmRange;
            double normalizedCurrentHoodAngle = (hood.GetHoodAngle() - Constants.HoodConstants.kMinimumAngle) / hoodAngleRange;

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

            double turretOffset = -hits.get(minimumI)[1];
            double shooterRpm = vals.get(minimumI)[2];
            double hoodAngle = vals.get(minimumI)[3];
            return new double[]{turretOffset, shooterRpm, hoodAngle};
        }
    }

    public static Lookup createLookup(Hood hood, Shooter shooter) { // Create a Lookup object
        return new Lookup(hood, shooter);
    }
}