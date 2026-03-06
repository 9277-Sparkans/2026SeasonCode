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
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
	private final PhotonCamera camera;
	private Transform3d robotToCamera;

	public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
		this.camera = new PhotonCamera(name);
		this.robotToCamera = robotToCamera;
	}

	@Override
	public void updateInputs(VisionIOInputs inputs, edu.wpi.first.math.geometry.Pose2d currentPose) {
		inputs.setConnected(camera.isConnected());

		// Read new camera observations
		Set<Short> tagIds = new HashSet<>();
		List<PoseObservation> poseObservations = new LinkedList<>();

		// Get all unread results from camera
		var allResults = camera.getAllUnreadResults();

		for (var result : allResults) {
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

			// Add pose observation
			if (result.getMultiTagResult().isPresent()) { // Multitag result
				var multitagResult = result.getMultiTagResult().get();

				// Calculate robot pose
				// Multi-tag result estimated pose is the camera's pose in the field
				Transform3d fieldToCamera = multitagResult.estimatedPose.best;
				Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
				Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

				// Calculate average tag distance
				double totalTagDistance = 0.0;
				for (PhotonTrackedTarget target : result.getTargets()) {
					totalTagDistance += target.getBestCameraToTarget().getTranslation().getNorm();
				}

				// Add tag IDs
				for (int id : multitagResult.fiducialIDsUsed) {
					tagIds.add((short) id);
				}

				// Add observation
				poseObservations.add(new PoseObservation(
						result.getTimestampSeconds(),
						robotPose,
						multitagResult.estimatedPose.ambiguity,
						multitagResult.fiducialIDsUsed.size(),
						totalTagDistance / result.getTargets().size(),
						PoseObservationType.PHOTONVISION));

			} else if (result.hasTargets()) { // Single tag result
				PhotonTrackedTarget target = result.getBestTarget();

				// Calculate robot pose
				var tagPose = VisionConstants.aprilTagLayout.getTagPose(target.getFiducialId());
				if (tagPose.isPresent()) {
					Transform3d fieldToTarget = new Transform3d(
							tagPose.get().getTranslation(), tagPose.get().getRotation());
					Transform3d cameraToTarget = target.getBestCameraToTarget();
					Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
					Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
					Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

					// Add tag ID
					tagIds.add((short) target.getFiducialId());

					// Add observation
					poseObservations.add(new PoseObservation(
							result.getTimestampSeconds(),
							robotPose,
							target.getPoseAmbiguity(),
							1,
							cameraToTarget.getTranslation().getNorm(),
							PoseObservationType.PHOTONVISION));
				}
			}
		}

		// Save pose observations to inputs object
		inputs.setPoseObservations(poseObservations.toArray(new PoseObservation[0]));

		// Save tag IDs to inputs objects
		inputs.setTagIds(new int[tagIds.size()]);
		int i = 0;
		for (int id : tagIds) {
			inputs.getTagIds()[i++] = id;
		}
	}

	@Override
	public String getName() {
		return camera.getName();
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
		return robotToCamera;
	}
}