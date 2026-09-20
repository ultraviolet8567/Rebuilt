package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** The intake pivot arm. */
public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {
        public boolean motorConnected = false;
        public boolean absoluteEncoderConnected = false;
        public double relativeAngleRad = 0.0;
        public double absoluteAngleRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public default void updateInputs(PivotIOInputs inputs) {}

    public default void setVoltage(double volts) {}

    public default void seedRelativeEncoder(double angleRad) {}

    public default void stop() {}
}
