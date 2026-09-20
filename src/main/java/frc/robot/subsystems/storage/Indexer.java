package frc.robot.subsystems.storage;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Alerts;
import org.littletonrobotics.junction.Logger;

/** The indexer belt. */
public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private final Alert disconnected =
            Alerts.create("Indexer motor disconnected", AlertType.kError);

    public Indexer(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Storage/Indexer", inputs);
        disconnected.set(!inputs.connected);
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public void stop() {
        io.stop();
    }

    /** Run the indexer for as long as the command is scheduled. */
    public Command run(boolean reversed) {
        return runEnd(
                        () -> setVoltage(StorageConstants.kIndexerVolts * (reversed ? -1 : 1)),
                        this::stop)
                .withName(reversed ? "Indexer reverse" : "Indexer forward");
    }
}
