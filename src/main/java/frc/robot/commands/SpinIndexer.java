package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Storage.Indexer;

public class SpinIndexer extends Command {
    private final Indexer indexer;

    public SpinIndexer(Indexer indexer) {
        this.indexer = indexer;

        addRequirements(indexer);
    }

    @Override
    public void execute() {
        indexer.start();
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stop();
    }
}
