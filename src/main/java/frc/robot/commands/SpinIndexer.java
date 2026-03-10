package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class SpinIndexer extends Command {
    private Indexer indexer;

    public SpinIndexer(Indexer indexer) {
        this.indexer = indexer;

        addRequirements(indexer);
    }

    @Override
    public void execute() {
        indexer.Start();
    }

    @Override
    public void end(boolean interrupted) {
        indexer.Stop();
    }
}
