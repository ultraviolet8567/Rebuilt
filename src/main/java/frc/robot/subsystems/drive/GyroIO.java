package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** The yaw gyro, behind an interface so simulation and replay can stand in for the Pigeon. */
public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = Rotation2d.kZero;
        public double yawVelocityRadPerSec = 0.0;
    }

    public default void updateInputs(GyroIOInputs inputs) {}

    /** Declare the robot's current heading to be {@code yaw}. */
    public default void setYaw(Rotation2d yaw) {}
}
