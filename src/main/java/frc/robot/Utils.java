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
        private final ArrayList<Double> keys = new ArrayList<>(); // Distance
        private final ArrayList<double[]> vals = new ArrayList<>(); // [RPM, Angle]
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
                    double key = Double.parseDouble(values[0]);
                    double rpm = Double.parseDouble(values[1]);
                    double angle = Double.parseDouble(values[2]);
                    
                    keys.add(key);
                    vals.add(new double[]{rpm, angle});
                }
            } catch (Exception e) {
                System.out.println("Error forming lookup table");
            }

            size = keys.size();
        }

        public int GetBestRow(double distance) { // Runs a binary search to find the row with the closest distance to the specified
            // Initial bounds
            int leftBound = 0;
            int rightBound = size - 1;

            // Binary search
            int i = (leftBound + rightBound) / 2;
            while (leftBound <= rightBound) {
                double currentVal = keys.get(i);

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

            if (Math.abs(keys.get(leftBound) - distance) < Math.abs(keys.get(rightBound) - distance)) {
                return leftBound;
            } else {
                return rightBound;
            }
        }

        public double[] FindOptimalVals(double distance) { // Finds the optimal shot for minimum hood movement and RPM change
            // Get range
            int bestI = GetBestRow(distance);
            int startI = bestI;
            int endI = bestI;

            while (distance - keys.get(startI) < Constants.ShooterConstants.autoshootDistanceRange) {
                startI--;
            }
            while (keys.get(endI) - distance < Constants.ShooterConstants.autoshootDistanceRange) {
                endI++;
            }

            // Helper values
            double rpmRange = (double)(Constants.ShooterConstants.kMaxRPM - Constants.ShooterConstants.kMinRPM);
            double angleRange = Constants.HoodConstants.kMaximumAngle - Constants.HoodConstants.kMinimumAngle;

            double currentNormalizedRPM = (shooter.GetShooterRPM() - Constants.ShooterConstants.kMinRPM) / rpmRange;
            double currentNormalizedAngle = (hood.GetHoodAngle() - Constants.HoodConstants.kMinimumAngle) / angleRange;

            // Get least square distance between current RPM and Angle vs desired
            double minimumWeight = Constants.ShooterConstants.autoshootAngleWeight + 1;
            int minimumI = 0;
            for (int i = startI; i <= endI; i++) {
                double[] row = vals.get(i);

                double normalizedRpm = (row[0] - Constants.ShooterConstants.kMinRPM) / rpmRange;
                double normalizedAngle = (row[1] - Constants.HoodConstants.kMinimumAngle) / angleRange;

                double weight = (normalizedRpm - currentNormalizedRPM) * (normalizedRpm - currentNormalizedRPM) + Constants.ShooterConstants.autoshootAngleWeight * (normalizedAngle - currentNormalizedAngle) * (normalizedAngle - currentNormalizedAngle);
                if (weight < minimumWeight) {
                    minimumWeight = weight;
                    minimumI = i;
                }
            }

            return vals.get(minimumI);
        }
    }

    public static Lookup createLookup(Hood hood, Shooter shooter) { // Create a Lookup object
        return new Lookup(hood, shooter);
    }
}