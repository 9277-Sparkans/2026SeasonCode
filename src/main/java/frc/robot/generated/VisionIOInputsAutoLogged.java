package frc.robot.generated;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.Vision.VisionIO;

public class VisionIOInputsAutoLogged
	extends VisionIO.VisionIOInputs
	implements LoggableInputs, Cloneable {

	@Override
	public void toLog(LogTable table) {
		table.put("Connected", isConnected());
		table.put("LatestTargetObservation", getLatestTargetObservation());
		table.put("PoseObservations", getPoseObservations());
		table.put("TagIds", getTagIds());
	}

	@Override
	public void fromLog(LogTable table) {
		setConnected(table.get("Connected", isConnected()));
		setLatestTargetObservation(
			table.get("LatestTargetObservation", getLatestTargetObservation())
		);
		setPoseObservations(
			table.get("PoseObservations", getPoseObservations())
		);
		setTagIds(
			table.get("TagIds", getTagIds())
		);
	}

	/**
	 * Creates a deep copy of this VisionIOInputsAutoLogged instance.
	 * @return A new instance with the same values.
	 */
	public VisionIOInputsAutoLogged clone() {
		VisionIOInputsAutoLogged copy = new VisionIOInputsAutoLogged();
		copy.setConnected(isConnected());
		copy.setLatestTargetObservation(getLatestTargetObservation());
		copy.setPoseObservations(getPoseObservations().clone());
		copy.setTagIds(getTagIds().clone());
		return copy;
	}
}