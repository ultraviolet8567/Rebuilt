package frc.robot.subsystems.vision;

/**
 * Stand-in camera for desktop simulation.
 *
 * <p>Reports itself connected but never produces a pose, so the pose estimate in simulation comes
 * purely from wheel odometry. That is deliberate: it keeps the simulated robot's behaviour
 * reproducible and makes the {@code Drive/SimTruePose} vs {@code Drive/Pose} comparison a genuine
 * measure of odometry drift rather than of a synthetic camera's accuracy.
 */
public class VisionIOSim implements VisionIO {
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = true;
        inputs.hasPose = false;
        inputs.tagCount = 0;
    }
}
