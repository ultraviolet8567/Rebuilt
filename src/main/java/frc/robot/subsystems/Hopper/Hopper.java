package frc.robot.subsystems.Hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
    private final Indexer indexer;

    public Hopper() {
        indexer = new Indexer();
    }

    public Indexer getIndexer() {
        return indexer;
    }
}
