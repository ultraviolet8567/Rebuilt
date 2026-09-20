package frc.robot.subsystems.storage;

import org.littletonrobotics.junction.AutoLog;

/** The indexer belt that moves a ball from storage into the kicker. */
public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public boolean connected = false;
        public double velocityRpm = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public default void updateInputs(IndexerIOInputs inputs) {}

    public default void setVoltage(double volts) {}

    public default void stop() {}
}
