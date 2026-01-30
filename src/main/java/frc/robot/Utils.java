package frc.robot;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;

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

    public class Lookup
    {
        ArrayList<Double> keys = new ArrayList<>(); // Distance
        ArrayList<Double[]> vals = new ArrayList<>(); // [RPM, Angle]
        int size = 0;

        Shooter shooter;
        Hood hood;

        public Lookup(String path, Hood hood, Shooter shooter) { // Load lookup table into variables
            this.hood = hood;
            this.shooter = shooter;

            try (BufferedReader br = new BufferedReader(new FileReader(path))) {
                br.readLine(); // Skip header line

                String line;
                while ((line = br.readLine()) != null) {
                    String[] values = line.split(",");
                    Double key = Double.parseDouble(values[0]);
                    Double rpm = Double.parseDouble(values[1]);
                    Double angle = Double.parseDouble(values[2]);
                    
                    keys.add(key);
                    vals.add(new Double[]{rpm, angle});
                }
            } catch (Exception e) {
                System.out.println("Error forming lookup table");
            }

            size = keys.size();
        }

        public int GetBestRow(Double distance) { // Runs a binary search to find the row with the closest distance to the specified
            int leftBound = 0;
            int rightBound = size - 1;
            int i = (rightBound - leftBound) / 2;

            int lastI = -1;
            while (lastI != i) {
                Double currentVal = keys.get(i);
                if (currentVal < distance) {
                    leftBound = i;
                } else if (currentVal > distance) {
                    rightBound = i;
                } else {
                    break;
                }

                lastI = i;
                i = (int)((rightBound - leftBound) / 2);
            }

            return i;
        }

        public Double[] FindOptimalVals(double distance) { // Finds the optimal shot for minimum hood movement and RPM change
            // TEMP
            int startI = 0;
            int endI = 0;

            // Helper values
            Double rpmRange = Constants.ShooterConstants.kMaxRPM - Constants.ShooterConstants.kMinRPM
            Double angleRange = Constants.HoodConstants.kMaximumAngle - Constants.HoodConstants.kMinimumAngle;

            Double currentNormalizedRPM = (shooter.GetHoodRPM() - Constants.ShooterConstants.kMinRPM) / rpmRange;
            Double currentNormalizedAngle = (hood.GetHoodAngle() - Constants.HoodConstants.kMinimumAngle) / angleRange;

            // Get least square distance between current (RPM, Angle) and desired
            Double[] weights = new Double[endI - startI + 1];
            for (int i = startI; i <= endI; i++) {
                Double[] row = vals.get(i);

                Double normalizedRpm = (row[0] - Constants.ShooterConstants.kMinRPM) / rpmRange;
                Double normalizedAngle = (row[1] - Constants.HoodConstants.kMinimumAngle) / angleRange;

                weights[i - startI] = (normalizedRpm - currentNormalizedRPM) * (normalizedRpm - currentNormalizedRPM) + Constants.Shooter.autoshootAngleWeight * (normalizedAngle - currentNormalizedAngle) * (normalizedAngle - currentNormalizedAngle);
            }

            // Chose the minimum weight
            Double minimumWeight = weights[0];
            Double minimumI = 0;
            for (int i = 1; i < weights.size(); i++) {
                Double weight = weights[i];
                if (weight < minimumWeight) {
                    minimumWeight = weight;
                    minimumI = i;
                }
            }

            return vals.get(startI + minimumI);
        }
    }
}