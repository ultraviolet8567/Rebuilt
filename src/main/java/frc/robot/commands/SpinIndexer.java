package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.StorageConstants;
import frc.robot.subsystems.Storage.Indexer;

public class SpinIndexer extends Command {
    private final Indexer indexer;
    private final boolean reversed;

    public SpinIndexer(Indexer indexer, boolean reversed) {
        this.indexer = indexer;
        this.reversed = reversed;

        addRequirements(indexer);
    }

    @Override
    public void execute() {
        indexer.setIndexerVoltage(StorageConstants.kIndexerMaxVoltage * (reversed ? -1 : 1));
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stop();
    }
}
