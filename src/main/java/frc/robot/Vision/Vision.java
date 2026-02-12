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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Use fully-qualified name for generated inputs to avoid any package/import resolution issues
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class Vision extends SubsystemBase {
        private final VisionConsumer consumer;
        private final Supplier<Pose2d> poseSupplier;
        private final VisionIO[] io;
        private final frc.robot.generated.VisionIOInputsAutoLogged[] inputs;
        private final Alert[] disconnectedAlerts;
        public boolean visionHasTarget = false;
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

        @Override
        public void periodic() {
                for (int i = 0; i < io.length; i++) {
                        io[i].updateInputs(inputs[i], poseSupplier.get());
                        Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
                }

                // Loop over cameras
                for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
                        // Update disconnected alert
                        disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].isConnected());

                        // Initialize logging values

                        // Add tag poses
                        for (int tagId : inputs[cameraIndex].getTagIds()) {
                                var tagPose = aprilTagLayout.getTagPose(tagId);
                                if (tagPose.isPresent() && !rejectedTags.contains(tagId)) {
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
                        for (var observation : inputs[cameraIndex].getPoseObservations()) {
                                // Check whether to reject pose
                                boolean rejectPose = false;
                                if (observation.tagCount() == 0) {
                                        rejectPose = true;
                                } else if (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity) {
                                        rejectPose = true;
                                } else if (Math.abs(observation.pose().getZ()) > maxZError) {
                                        rejectPose = true;
                                } else if (observation.pose().getX() < 0.0
                                                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                                                || observation.pose().getY() < 0.0
                                                || observation.pose().getY() > aprilTagLayout.getFieldWidth()) {
                                        rejectPose = true;
                                }

                                // Skip if rejected
                                if (rejectPose) {
                                        continue;
                                }

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