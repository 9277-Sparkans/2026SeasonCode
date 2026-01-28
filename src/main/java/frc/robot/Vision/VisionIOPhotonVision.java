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

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

import static frc.robot.Constants.VisionConstants.TAG_LAYOUT;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import frc.robot.Vision.VisionConstants;

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
		estimator = new PhotonPoseEstimator(VisionConstants.aprilTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
				roboToCamera);
	}

	@Override
	public void updateInputs(VisionIOInputs inputs) {
		inputs.setConnected(camera.isConnected());

		// Read new camera observations
		Set<Short> tagIds = new HashSet<>();
		List<PoseObservation> poseObservations = new LinkedList<>();

		for (var result : camera.getAllUnreadResults()) {
			// Update latest target observation
			if (result.hasTargets()) {
				inputs.setLatestTargetObservation(
						new TargetObservation(
								Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
								Rotation2d.fromDegrees(result.getBestTarget().getPitch())));
			} else {
				inputs.setLatestTargetObservation(
						new TargetObservation(new Rotation2d(), new Rotation2d()));
			}

			// Update pose estimator
			var estimatedPose = estimator.estimateCoprocMultiTagPose(result);
			if (estimatedPose.isEmpty()) {
				estimatedPose = estimator.estimateLowestAmbiguityPose(result);
			}

			if (estimatedPose.isPresent()) {
				var pose = estimatedPose.get();

				// Collect tag IDs from targets
				for (var target : pose.targetsUsed) {
					tagIds.add((short) target.getFiducialId());
				}

				// Calculate ambiguity
				double ambiguity = 0.0;
				if (pose.targetsUsed.size() == 1) {
					ambiguity = pose.targetsUsed.get(0).getPoseAmbiguity();
				}

				// Add pose observation
				poseObservations.add(
						new PoseObservation(
								pose.timestampSeconds,
								pose.estimatedPose,
								ambiguity,
								pose.targetsUsed.size(),
								0.0, // Average tag distance not provided directly
								PoseObservationType.PHOTONVISION));
			}
		}

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