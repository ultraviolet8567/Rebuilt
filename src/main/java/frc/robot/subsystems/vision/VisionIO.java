package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

/** One AprilTag camera. */
public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean connected = false;

        /** Whether the camera produced a usable pose this loop. */
        public boolean hasPose = false;

        public Pose2d estimatedPose = new Pose2d();
        public double timestampSeconds = 0.0;
        public int tagCount = 0;
        public double averageTagDistanceMeters = 0.0;

        /** Solver ambiguity of the best target, 0 = unambiguous. */
        public double ambiguity = 0.0;
    }

    public default void updateInputs(VisionIOInputs inputs) {}

    /**
     * Tell the camera which way the robot is facing. MegaTag2 needs this every loop: it uses the
     * gyro heading to disambiguate the tag solution, which is what makes it immune to the pose
     * flipping that plagues single-tag MegaTag1.
     */
    public default void setRobotOrientation(double yawDegrees) {}

    /** LEDs on (pit/disabled) or under pipeline control (match). */
    public default void setLedsForced(boolean forced) {}

    /** Throttle the camera's processing rate to save power while disabled. */
    public default void setThrottle(int throttle) {}
}
