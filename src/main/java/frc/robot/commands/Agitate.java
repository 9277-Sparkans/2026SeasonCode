package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Hinge;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Hinge.HingeState;
import frc.robot.subsystems.Indexer.IndexerGoal;

public class Agitate extends Command {
    private Hinge hinge;

    public Agitate(Hinge hinge) {
        this.hinge = hinge;

        addRequirements(hinge);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    public static Command agitate(Hinge hinge, Indexer indexer) {
        return Commands.parallel(
            Commands.repeatingSequence(
                // Commands.waitSeconds(1),
                // Commands.runOnce(() -> hinge.states(HingeState.AGITATE)),
                // Commands.waitSeconds(1),
                // Commands.runOnce(() -> hinge.states(HingeState.DOWN))

                Commands.runOnce(() -> hinge.states(HingeState.AGITATE)),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> hinge.states(HingeState.DOWN)),
                Commands.waitSeconds(0.5)
            )
            // Commands.repeatingSequence(
            //     Commands.waitSeconds(1.5),
            //     Commands.runOnce(() -> indexer.agitate(true)),
            //     Commands.waitSeconds(0.5),
            //     Commands.runOnce(() -> indexer.agitate(false))
            // )
        ).finallyDo((interrupted) -> {
            if (interrupted) {
                hinge.states(HingeState.DOWN);
                // indexer.agitate(false);
            }
        });
    }
}
