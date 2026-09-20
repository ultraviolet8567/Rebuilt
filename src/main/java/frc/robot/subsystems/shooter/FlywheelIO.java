package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** The two-sided shooter flywheel. */
public interface FlywheelIO {
    @AutoLog
    public static class FlywheelIOInputs {
        public boolean leadConnected = false;
        public boolean followerConnected = false;

        /** Wheel velocity, RPM, positive when shooting. */
        public double leadVelocityRpm = 0.0;

        public double followerVelocityRpm = 0.0;
        public double leadAppliedVolts = 0.0;
        public double followerAppliedVolts = 0.0;
        public double leadCurrentAmps = 0.0;
        public double followerCurrentAmps = 0.0;
    }

    public default void updateInputs(FlywheelIOInputs inputs) {}

    /** Apply voltage to both sides. The implementation clamps and handles inversion. */
    public default void setVoltage(double leadVolts, double followerVolts) {}

    public default void stop() {}
}
