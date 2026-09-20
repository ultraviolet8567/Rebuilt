package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** The funnel rollers that pull game pieces in. */
public interface FunnelIO {
    @AutoLog
    public static class FunnelIOInputs {
        public boolean connected = false;
        public double velocityRps = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public default void updateInputs(FunnelIOInputs inputs) {}

    public default void setVoltage(double volts) {}

    public default void stop() {}
}
