package frc.robot;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;

public class Lookup {
    ArrayList<Double> keys = new ArrayList<>(); // Distance
    ArrayList<Double[]> vals = new ArrayList<>(); // [RPM, Angle]
    int size = 0;

    public Lookup(String path) { // Load lookup table into variables
        try (BufferedReader br = new BufferedReader(new FileReader("book.csv"))) {
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

    public int GetBestRow(Double distance) {
        int left_bound = 0;
        int right_bound = size - 1;
        int i = (right_bound - left_bound) / 2;

        while (true) {
            Double currentVal = keys.get(i);
            if (currentVal < distance) {
                left_bound = i;
            } else if (currentVal > distance) {
                right_bound = i;
            } else {
                break;
            }

            i = (int)((right_bound - left_bound) / 2);
        }

        return i;
    }
}
