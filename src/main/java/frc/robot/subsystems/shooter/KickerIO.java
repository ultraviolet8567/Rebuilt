package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** The kicker wheel that feeds a ball into the flywheel. */
public interface KickerIO {
    @AutoLog
    public static class KickerIOInputs {
        public boolean connected = false;
        public double velocityRpm = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public default void updateInputs(KickerIOInputs inputs) {}

    public default void setVoltage(double volts) {}

    public default void stop() {}
}
