package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** The adjustable hood that sets the shot angle. */
public interface HoodIO {
    @AutoLog
    public static class HoodIOInputs {
        public boolean motorConnected = false;
        public boolean absoluteEncoderConnected = false;

        /** Hood angle from the motor's relative encoder, radians. */
        public double relativeAngleRad = 0.0;

        /** Hood angle from the duty-cycle absolute encoder, radians. Aliases outside its window. */
        public double absoluteAngleRad = 0.0;

        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public default void updateInputs(HoodIOInputs inputs) {}

    public default void setVoltage(double volts) {}

    /** Re-seed the relative encoder to the given hood angle. */
    public default void seedRelativeEncoder(double angleRad) {}

    public default void stop() {}
}
