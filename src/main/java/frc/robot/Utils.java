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

    public static class Lookup
    {
        ArrayList<Double> keys = new ArrayList<>(); // Distance
        ArrayList<Double[]> vals = new ArrayList<>(); // [RPM, Angle]
        int size = 0;

        public Lookup(String path) { // Load lookup table into variables
            try (BufferedReader br = new BufferedReader(new FileReader(path + "/book.csv"))) {
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
            int left_bound = 0;
            int right_bound = size - 1;
            int i = (right_bound - left_bound) / 2;

            int last_i = -1;
            while (last_i != i) {
                Double currentVal = keys.get(i);
                if (currentVal < distance) {
                    left_bound = i;
                } else if (currentVal > distance) {
                    right_bound = i;
                } else {
                    break;
                }

                last_i = i;
                i = (int)((right_bound - left_bound) / 2);
            }

            return i;
        }

        public double RowWeight(int i) {
            Double[] row = vals.get(i);

            Double normalized_rpm = (row[0] - Constants.ShooterConstants.kMinRPM) / (Constants.ShooterConstants.kMaxRPM - Constants.ShooterConstants.kMinRPM);
            Double normalized_angle = (row[1] - Constants.HoodConstants.kMinimumAngle) / (Constants.HoodConstants.kMaximumAngle - Constants.HoodConstants.kMinimumAngle);

            return 0;
        }
    }
}