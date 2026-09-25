package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.SynthesisDevices;
import java.util.Optional;

/**
 * A camera that sees exactly where Synthesis put the robot.
 *
 * <p>Each loop it reports the true field pose as a clean two-tag measurement, so it goes through
 * the same filtering and fusion as a Limelight pose on the real robot. It also reports when
 * Synthesis has just placed or teleported the robot, so the caller can reset odometry outright: the
 * pose estimator would otherwise take seconds to walk across the field, and heading comes from the
 * gyro, which starts at zero whichever way the robot was placed.
 */
public class VisionIOSynthesis implements VisionIO {
    private static final double kLatencySecs = 0.02;

    private final SynthesisDevices.FieldPose fieldPose = SynthesisDevices.fieldPose();
    private int seenPlacements = 0;

    private Pose2d pose() {
        return new Pose2d(fieldPose.x(), fieldPose.y(), new Rotation2d(fieldPose.headingRad()));
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        boolean valid = fieldPose.placementCount() > 0;
        inputs.connected = true;
        inputs.hasPose = valid;
        inputs.estimatedPose = valid ? pose() : new Pose2d();
        inputs.timestampSeconds = Timer.getFPGATimestamp() - kLatencySecs;
        inputs.tagCount = valid ? 2 : 0;
        inputs.averageTagDistanceMeters = 2.0;
        inputs.ambiguity = 0.0;
    }

    /** The true pose if Synthesis has placed the robot since the last call. */
    public Optional<Pose2d> takePlacement() {
        int count = fieldPose.placementCount();
        if (count > seenPlacements) {
            seenPlacements = count;
            return Optional.of(pose());
        }
        return Optional.empty();
    }
}
