
// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.Vision;

import java.nio.file.Path;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout;
    private static boolean usedCustomField = false;
    private static String loadedLayoutDescription = "Unknown";
    static {
        try {
            aprilTagLayout = new AprilTagFieldLayout(Path
                    .of(Filesystem.getDeployDirectory().getAbsolutePath()
                            + "/vision/welded.json"));
            usedCustomField = true;
            loadedLayoutDescription = "Custom: vision/welded.json";
        } catch (Exception e) {
            aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
            loadedLayoutDescription = "Bundled: k2026RebuiltWelded";
        }
        Logger.recordOutput("Vision/LayoutLoaded", loadedLayoutDescription);
        Logger.recordOutput("Used Custom Field?", usedCustomField);
    }


    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "front_left";
    public static String camera1Name = "front_right";
    public static String camera2Name = "back_left";
    public static String camera3Name = "back_right";

    // Robot to camera transforms (Units: Meters and Radians)
    
    // front left camera: 317.54mm front, 324.226mm left, 178 mm up, and yaw 30 degrees in, pitch 20 degrees up.
    public static Transform3d robotToCamera0 = new Transform3d(
            0.31754, 0.324226, 0.178, 
            new Rotation3d(0.0, Units.degreesToRadians(-20), Units.degreesToRadians(-30)));

    // front right camera: 317.54mm front, 324.226mm right, 178 mm up, and yaw 30 degrees in, pitch 20 degrees up.
    public static Transform3d robotToCamera1 = new Transform3d(
            0.31754, -0.324226, 0.178, 
            new Rotation3d(0.0, Units.degreesToRadians(-20), Units.degreesToRadians(30)));

    // back left camera: 317.54mm back, 324.226mm left, 170mm up, and yaw 30 degrees out, pitch 20 degrees up.
    public static Transform3d robotToCamera2 = new Transform3d(
            -0.31754, 0.324226, 0.17, 
            new Rotation3d(0.0, Units.degreesToRadians(-20), Units.degreesToRadians(150)));
    
        // back right camera: 317.54mm back, 324.226mm right, 170mm up, and yaw 30 degrees out, pitch 20 degrees up.
    public static Transform3d robotToCamera3 = new Transform3d(
            -0.31754, -0.324226, 0.17, 
            new Rotation3d(0.0, Units.degreesToRadians(-20), Units.degreesToRadians(-150)));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {
            1.0, // Camera 0
            1.0, // Camera 1
            1.0, // Camera 2
            1.0 // Camera 3 
    };

    public static List<Integer> rejectedTags = Arrays.asList(2, 3, 4, 5, 14, 15, 16);
}
