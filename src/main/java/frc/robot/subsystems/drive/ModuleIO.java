package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * Everything one swerve module can tell the robot, and everything the robot can ask of it.
 *
 * <p>The subsystem above this interface never touches a Spark, so it behaves identically on the
 * real robot, in desktop simulation, and when replaying a log -- only the implementation changes.
 * The {@code @AutoLog} annotation generates {@code ModuleIOInputsAutoLogged}, which serialises
 * every field into the log each loop and, in REPLAY mode, reads them back out. That is what makes a
 * match log re-runnable against modified control code.
 */
public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public boolean driveConnected = false;
        public double drivePositionMeters = 0.0;
        public double driveVelocityMetersPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public boolean turnConnected = false;
        public boolean absoluteEncoderConnected = false;

        /** Steering angle as measured by the analog absolute encoder, offset applied. */
        public Rotation2d turnPosition = Rotation2d.kZero;

        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;

        /** Raw analog voltage, logged so the encoder can be recalibrated from a match log. */
        public double absoluteEncoderVolts = 0.0;
    }

    public default void updateInputs(ModuleIOInputs inputs) {}

    /** Command a wheel speed in metres per second (closed loop, with feedforward). */
    public default void setDriveVelocity(double velocityMetersPerSec) {}

    /** Command a raw drive voltage. Used by characterisation routines. */
    public default void setDriveVoltage(double volts) {}

    /** Command a steering angle. Wrapping is handled by the implementation. */
    public default void setTurnPosition(Rotation2d rotation) {}

    /** Coast the drive motor and hold the current steering angle. */
    public default void stop() {}

    /** Switch the drive motor between brake and coast (coast while pushing the robot around). */
    public default void setBrakeMode(boolean brake) {}
}
