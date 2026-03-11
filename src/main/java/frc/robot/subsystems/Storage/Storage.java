package frc.robot.subsystems.Storage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Storage extends SubsystemBase {
    private final Indexer indexer;

    public Storage() {
        indexer = new Indexer();
    }

    public Indexer getIndexer() {
        return indexer;
    }
}
