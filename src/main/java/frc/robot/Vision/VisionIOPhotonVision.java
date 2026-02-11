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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Pose3d;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
	private final PhotonCamera camera;
	private final PhotonPoseEstimator estimator;

	/**
	 * Creates a new VisionIOPhotonVision.
	 *
	 * @param name         The configured name of the camera.
	 * @param roboToCamera The transform from the robot to the camera.
	 */

	public VisionIOPhotonVision(String name, Transform3d roboToCamera) {
		camera = new PhotonCamera(name);
		// In PhotonLib v2026, the strategy is defined by the method called (e.g.
		// estimateCoprocMultiTagPose)
		// rather than the constructor.
		estimator = new PhotonPoseEstimator(VisionConstants.aprilTagLayout,
				roboToCamera);
	}

	/**
	 * Compute the average distance from camera to all detected targets in meters.
	 *
	 * @param targets List of detected targets
	 * @return Average distance in meters, or 0.0 if no targets
	 */
	private double computeAverageTagDistance(java.util.List<org.photonvision.targeting.PhotonTrackedTarget> targets) {
		if (targets.isEmpty()) {
			return 0.0;
		}
		double totalDistance = 0.0;
		for (var target : targets) {
			totalDistance += target.getBestCameraToTarget().getTranslation().getNorm();
		}
		return totalDistance / targets.size();
	}

	@Override
	public void updateInputs(VisionIOInputs inputs, edu.wpi.first.math.geometry.Pose2d currentPose) {
		inputs.setConnected(camera.isConnected());

		// Seed the estimator with the current pose
		// estimator.setReferencePose(currentPose); // Deprecated

		// Read new camera observations
		Set<Short> tagIds = new HashSet<>();
		List<PoseObservation> poseObservations = new LinkedList<>();

		// Get all unread results from camera
		var allResults = camera.getAllUnreadResults();

		// DEBUG: Log number of results received
		org.littletonrobotics.junction.Logger.recordOutput(
				"Vision/Debug/" + camera.getName() + "/ResultsCount", allResults.size());

		for (var result : allResults) {
			// DEBUG: Log target detection
			org.littletonrobotics.junction.Logger.recordOutput(
					"Vision/Debug/" + camera.getName() + "/HasTargets", result.hasTargets());
			org.littletonrobotics.junction.Logger.recordOutput(
					"Vision/Debug/" + camera.getName() + "/TargetCount",
					result.hasTargets() ? result.getTargets().size() : 0);

			// Update latest target observation
			if (result.hasTargets()) {
				inputs.setLatestTargetObservation(
						new TargetObservation(
								Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
								Rotation2d.fromDegrees(result.getBestTarget().getPitch())));

				// DEBUG: Log detected tag IDs
				var detectedTagIds = new java.util.ArrayList<Integer>();
				for (var target : result.getTargets()) {
					detectedTagIds.add(target.getFiducialId());
				}
				org.littletonrobotics.junction.Logger.recordOutput(
						"Vision/Debug/" + camera.getName() + "/DetectedTagIDs",
						detectedTagIds.stream().mapToInt(Integer::intValue).toArray());
			} else {
				inputs.setLatestTargetObservation(
						new TargetObservation(new Rotation2d(), new Rotation2d()));
			}

			// Update pose estimator
			// 1. Try Coprocessor Multi-Tag (most accurate)
			Optional<EstimatedRobotPose> estimatedPose = estimator.estimateCoprocMultiTagPose(result);
			boolean usedMultiTag = estimatedPose.isPresent();

			// DEBUG: Log estimation method
			org.littletonrobotics.junction.Logger.recordOutput(
					"Vision/Debug/" + camera.getName() + "/UsedMultiTag", usedMultiTag);

			// 2. Fallback to local estimation (single tag or if coproc fails)
			if (estimatedPose.isEmpty() && result.hasTargets()) {
				estimatedPose = estimator.estimateClosestToReferencePose(result, new Pose3d(currentPose));
				org.littletonrobotics.junction.Logger.recordOutput(
						"Vision/Debug/" + camera.getName() + "/UsedLocalEstimation", true);
			} else {
				org.littletonrobotics.junction.Logger.recordOutput(
						"Vision/Debug/" + camera.getName() + "/UsedLocalEstimation", false);
			}

			// DEBUG: Log whether we got a pose estimate
			org.littletonrobotics.junction.Logger.recordOutput(
					"Vision/Debug/" + camera.getName() + "/HasPoseEstimate", estimatedPose.isPresent());

			if (estimatedPose.isPresent()) {
				var pose = estimatedPose.get();

				// DEBUG: Log raw pose from estimator (Pose3d)
				org.littletonrobotics.junction.Logger.recordOutput(
						"Vision/Debug/" + camera.getName() + "/RawPose3d", pose.estimatedPose);
				org.littletonrobotics.junction.Logger.recordOutput(
						"Vision/Debug/" + camera.getName() + "/RawPose3d_X", pose.estimatedPose.getX());
				org.littletonrobotics.junction.Logger.recordOutput(
						"Vision/Debug/" + camera.getName() + "/RawPose3d_Y", pose.estimatedPose.getY());
				org.littletonrobotics.junction.Logger.recordOutput(
						"Vision/Debug/" + camera.getName() + "/RawPose3d_Z", pose.estimatedPose.getZ());
				org.littletonrobotics.junction.Logger.recordOutput(
						"Vision/Debug/" + camera.getName() + "/RawPose3d_RotDeg",
						pose.estimatedPose.getRotation().toRotation2d().getDegrees());

				// Collect tag IDs from targets
				var usedTagIds = new java.util.ArrayList<Integer>();
				for (var target : pose.targetsUsed) {
					tagIds.add((short) target.getFiducialId());
					usedTagIds.add(target.getFiducialId());
				}

				// DEBUG: Log tags used for pose estimation
				org.littletonrobotics.junction.Logger.recordOutput(
						"Vision/Debug/" + camera.getName() + "/TagsUsedForPose",
						usedTagIds.stream().mapToInt(Integer::intValue).toArray());
				org.littletonrobotics.junction.Logger.recordOutput(
						"Vision/Debug/" + camera.getName() + "/TagsUsedCount", pose.targetsUsed.size());

				// Calculate ambiguity: for single-tag use target ambiguity,
				// for multi-tag use average target ambiguity
				double ambiguity = 0.0;
				if (pose.targetsUsed.size() == 1) {
					ambiguity = pose.targetsUsed.get(0).getPoseAmbiguity();
				} else if (pose.targetsUsed.size() > 1) {
					double totalAmbiguity = 0.0;
					for (var target : pose.targetsUsed) {
						totalAmbiguity += target.getPoseAmbiguity();
					}
					ambiguity = totalAmbiguity / pose.targetsUsed.size();
				}

				// DEBUG: Log ambiguity
				org.littletonrobotics.junction.Logger.recordOutput(
						"Vision/Debug/" + camera.getName() + "/Ambiguity", ambiguity);

				// Compute average distance to all targets for std dev scaling
				double avgDistance = computeAverageTagDistance(pose.targetsUsed);

				// DEBUG: Log average distance
				org.littletonrobotics.junction.Logger.recordOutput(
						"Vision/Debug/" + camera.getName() + "/AvgTagDistance", avgDistance);

				// DEBUG: Log converted Pose2d that will be sent to Vision subsystem
				var pose2d = pose.estimatedPose.toPose2d();
				org.littletonrobotics.junction.Logger.recordOutput(
						"Vision/Debug/" + camera.getName() + "/ConvertedPose2d", pose2d);
				org.littletonrobotics.junction.Logger.recordOutput(
						"Vision/Debug/" + camera.getName() + "/ConvertedPose2d_X", pose2d.getX());
				org.littletonrobotics.junction.Logger.recordOutput(
						"Vision/Debug/" + camera.getName() + "/ConvertedPose2d_Y", pose2d.getY());
				org.littletonrobotics.junction.Logger.recordOutput(
						"Vision/Debug/" + camera.getName() + "/ConvertedPose2d_RotDeg",
						pose2d.getRotation().getDegrees());

				// Add pose observation
				poseObservations.add(
						new PoseObservation(
								pose.timestampSeconds,
								pose.estimatedPose,
								ambiguity,
								pose.targetsUsed.size(),
								avgDistance,
								PoseObservationType.PHOTONVISION));

				// DEBUG: Log that we added a pose observation
				org.littletonrobotics.junction.Logger.recordOutput(
						"Vision/Debug/" + camera.getName() + "/AddedPoseObservation", true);
			} else {
				org.littletonrobotics.junction.Logger.recordOutput(
						"Vision/Debug/" + camera.getName() + "/AddedPoseObservation", false);
			}
		}

		// DEBUG: Log total pose observations created
		org.littletonrobotics.junction.Logger.recordOutput(
				"Vision/Debug/" + camera.getName() + "/PoseObservationsCount", poseObservations.size());

		// Save pose observations to inputs object
		inputs.setPoseObservations(new PoseObservation[poseObservations.size()]);
		for (int i = 0; i < poseObservations.size(); i++) {
			inputs.getPoseObservations()[i] = poseObservations.get(i);
		}

		// Save tag IDs to inputs objects
		inputs.setTagIds(new int[tagIds.size()]);
		int i = 0;
		for (int id : tagIds) {
			inputs.getTagIds()[i++] = id;
		}
	}

	/**
	 * Get the camera.
	 * 
	 * @return The camera.
	 */
	public PhotonCamera getCamera() {
		return camera;
	}

	/**
	 * Get the transform from the robot to the camera.
	 * 
	 * @return The transform from the robot to the camera.
	 */
	public Transform3d getRobotToCamera() {
		return estimator.getRobotToCameraTransform();
	}
}