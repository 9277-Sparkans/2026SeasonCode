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

import static frc.robot.Vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;

public class Vision extends SubsystemBase {
        private final VisionConsumer consumer;
        private final Supplier<Pose2d> poseSupplier;
        private final VisionIO[] io;
        private final frc.robot.generated.VisionIOInputsAutoLogged[] inputs;
        private final Alert[] disconnectedAlerts;
        public boolean visionHasTarget = false;
        private Pose3d latestVisionPose = new Pose3d();
        private Pose3d[] latestRobotPoses = new Pose3d[0];

        public boolean enableVision = true;

        // logs out poses from all cameras
        // incredibly expensive so only enable if needed for debugging
        private final boolean DEBUG_MODE = true;

        private Thread visionThread;

        /**
         * Get all robot poses from the latest cycle (same as Vision/Summary/RobotPoses
         * in NT).
         *
         * @return Array of all robot pose observations.
         */
        public Pose3d[] getRobotPoses() {
                return latestRobotPoses;
        }

        /**
         * Get the latest accepted vision pose with the lowest ambiguity.
         * Falls back to the last known good pose if no new observations are accepted.
         *
         * @return The best vision pose as a Pose3d.
         */
        public Pose3d getLatestVisionPose() {
                return latestVisionPose;
        }

        private boolean seesThisTarget = false;

        public Vision(VisionConsumer consumer, Supplier<Pose2d> poseSupplier, VisionIO... io) {
                this.consumer = consumer;
                this.poseSupplier = poseSupplier;
                this.io = io;

                // Initialize inputs
                this.inputs = new frc.robot.generated.VisionIOInputsAutoLogged[io.length];
                for (int i = 0; i < inputs.length; i++) {
                        inputs[i] = new frc.robot.generated.VisionIOInputsAutoLogged();
                }

                // Initialize disconnected alerts
                this.disconnectedAlerts = new Alert[io.length];
                for (int i = 0; i < inputs.length; i++) {
                        disconnectedAlerts[i] = new Alert(
                                        "Vision camera " + Integer.toString(i) + " is disconnected.",
                                        AlertType.kWarning);
                }

                visionThread = new Thread() {
                        public void run() {
                                while (true) {
                                        updateVision();
                                }
                        }
                };
                visionThread.start();
        }

        /**
         * Returns the X angle to the best target, which can be used for simple servoing
         * with vision.
         *
         * @param cameraIndex The index of the camera to use.
         */
        public Rotation2d getTargetX(int cameraIndex) {
                return inputs[cameraIndex].getLatestTargetObservation().tx();
        }

        public void updateVision() {
                if (!enableVision) return;

                for (int i = 0; i < io.length; i++) {
                        io[i].updateInputs(inputs[i], poseSupplier.get());
                        // Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
                }

                // Initialize logging values
                List<Pose3d> allTagPoses = new LinkedList<>();
                List<Pose3d> allRobotPoses = new LinkedList<>();
                List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
                List<Pose3d> allRobotPosesRejected = new LinkedList<>();
                double bestAmbiguity = Double.MAX_VALUE;
                Pose3d bestPose = null;
                int totalAccepted = 0;
                int totalRejected = 0;
                StringBuilder rejectionLog = new StringBuilder();

                // Loop over cameras
                for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
                        // Update disconnected alert
                        disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].isConnected());

                        // Initialize logging values
                        List<Pose3d> tagPoses = new LinkedList<>();
                        List<Pose3d> robotPoses = new LinkedList<>();
                        List<Pose3d> robotPosesAccepted = new LinkedList<>();
                        List<Pose3d> robotPosesRejected = new LinkedList<>();

                        // Add tag poses
                        for (int tagId : inputs[cameraIndex].getTagIds()) {
                                var tagPose = aprilTagLayout.getTagPose(tagId);
                                if (tagPose.isPresent() && !rejectedTags.contains(tagId)) {
                                        tagPoses.add(tagPose.get());
                                        seesThisTarget = true;
                                }
                        }

                        // Report to visionhas Target whether or not vision sees at least one tag
                        if (seesThisTarget) {
                                visionHasTarget = true;
                                // Now reset seesThisTarget for next periodic loop
                                seesThisTarget = false;
                        } else {
                                visionHasTarget = false;
                        }

                        // Loop over pose observations
                        for (VisionIO.PoseObservation observation : inputs[cameraIndex].getPoseObservations()) {
                                // Check whether to reject pose
                                boolean rejectPose = false;
                                String rejectionReason = "";
                                if (observation.tagCount() == 0) {
                                        rejectPose = true;
                                        rejectionReason = "TagCount=0";
                                } else if (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity) {
                                        rejectPose = true;
                                        rejectionReason = "Ambiguity=" + observation.ambiguity() + " > " + maxAmbiguity;
                                } else if (observation.tagCount() == 1 && observation.averageTagDistance() > 3.0) {
                                        rejectPose = true;
                                        rejectionReason = "Distance=" + String.format("%.2f", observation.averageTagDistance()) + " > 3.0 for 1 tag";
                                } else if (Math.abs(observation.pose().getZ()) > maxZError) {
                                        rejectPose = true;
                                        rejectionReason = "ZError=" + observation.pose().getZ() + " > " + maxZError;
                                } else if (Math.abs(observation.pose().getRotation().getX()) > Math.toRadians(5.0)
                                                || Math.abs(observation.pose().getRotation().getY()) > Math.toRadians(5.0)) {
                                        rejectPose = true;
                                        rejectionReason = "Bad Pitch/Roll";
                                } else if (observation.pose().getX() < 0.0
                                                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                                                || observation.pose().getY() < 0.0
                                                || observation.pose().getY() > aprilTagLayout.getFieldWidth()) {
                                        rejectPose = true;
                                        rejectionReason = "OutOfBounds";
                                // } else if (observation.pose().toPose2d().getTranslation().getDistance(poseSupplier.get().getTranslation()) < visionHysteresis) {
                                //         rejectPose = true;
                                //         rejectionReason = "Hysteresis";
                                }

                                // Add pose to log
                                robotPoses.add(observation.pose());
                                if (rejectPose) {
                                        robotPosesRejected.add(observation.pose());
                                } else {
                                        robotPosesAccepted.add(observation.pose());
                                        // Track lowest ambiguity pose across all cameras
                                        if (observation.ambiguity() < bestAmbiguity) {
                                                bestAmbiguity = observation.ambiguity();
                                                bestPose = observation.pose();
                                        }
                                }

                                // Skip if rejected
                                if (rejectPose) {
                                        totalRejected++;
                                        rejectionLog.append("Camera").append(cameraIndex)
                                                        .append(": ").append(rejectionReason)
                                                        .append(" pose=(")
                                                        .append(String.format("%.2f", observation.pose().getX()))
                                                        .append(",")
                                                        .append(String.format("%.2f", observation.pose().getY()))
                                                        .append(",")
                                                        .append(String.format("%.2f", observation.pose().getZ()))
                                                        .append(") | ");
                                        continue;
                                }
                                totalAccepted++;

                                // Calculate standard deviations
                                double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0)
                                                / observation.tagCount();
                                double linearStdDev = linearStdDevBaseline * stdDevFactor;
                                double angularStdDev = angularStdDevBaseline * stdDevFactor;
                                if (cameraIndex < cameraStdDevFactors.length) {
                                        linearStdDev *= cameraStdDevFactors[cameraIndex];
                                        angularStdDev *= cameraStdDevFactors[cameraIndex];
                                }

                                // Send vision observation
                                consumer.accept(
                                                observation.pose().toPose2d(),
                                                observation.timestamp(),
                                                VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
                        }

                        // Log camera metadata
                        Logger.recordOutput("Vision/Camera" + cameraIndex + "/TagPoses",
                                        tagPoses.toArray(new Pose3d[0]));
                        Logger.recordOutput("Vision/Camera" + cameraIndex + "/RobotPoses",
                                        robotPoses.toArray(new Pose3d[0]));
                        Logger.recordOutput("Vision/Camera" + cameraIndex + "/RobotPosesAccepted",
                                        robotPosesAccepted.toArray(new Pose3d[0]));
                        Logger.recordOutput("Vision/Camera" + cameraIndex + "/RobotPosesRejected",
                                        robotPosesRejected.toArray(new Pose3d[0]));

                        allTagPoses.addAll(tagPoses);
                        allRobotPoses.addAll(robotPoses);
                        allRobotPosesAccepted.addAll(robotPosesAccepted);
                        allRobotPosesRejected.addAll(robotPosesRejected);
                }

                // Store all robot poses for external access (same data as
                // Vision/Summary/RobotPoses)
                latestRobotPoses = allRobotPoses.toArray(new Pose3d[0]);

                // Log summary data
                Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
                Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
                Logger.recordOutput("Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
                Logger.recordOutput("Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));

                // Update latest vision pose if we found an accepted observation this cycle
                if (bestPose != null) {
                        latestVisionPose = bestPose;
                }
                Logger.recordOutput("Vision/BestVisionPose", latestVisionPose);
                Logger.recordOutput("Vision/Debug/AcceptedCount", totalAccepted);
                Logger.recordOutput("Vision/Debug/RejectedCount", totalRejected);
                Logger.recordOutput("Vision/Debug/RejectionReasons", rejectionLog.toString());

                try {
                        Thread.sleep(750);
                } catch (InterruptedException e) {
                        e.printStackTrace();
                }
        }

        @FunctionalInterface
        public static interface VisionConsumer {
                public void accept(
                                Pose2d visionRobotPoseMeters,
                                double timestampSeconds,
                                Matrix<N3, N1> visionMeasurementStdDevs);
        }
}
