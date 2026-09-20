package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;

/** A Limelight running MegaTag2. */
public class VisionIOLimelight implements VisionIO {
    private final String name;

    public VisionIOLimelight(String name) {
        this.name = name;
        LimelightHelpers.setCameraPose_RobotSpace(
                name,
                VisionConstants.kCameraOffsetMeters.getX(),
                VisionConstants.kCameraOffsetMeters.getY(),
                VisionConstants.kCameraOffsetMeters.getZ(),
                VisionConstants.kCameraRollDegrees,
                VisionConstants.kCameraPitchDegrees,
                VisionConstants.kCameraYawDegrees);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // A Limelight that has gone away stops bumping its heartbeat. Comparing the table's
        // existence is not enough -- NetworkTables keeps the last value forever.
        inputs.connected =
                NetworkTableInstance.getDefault().getTable(name).containsKey("tv")
                        && LimelightHelpers.getLimelightNTTableEntry(name, "hb").exists();

        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        if (estimate == null || estimate.tagCount == 0) {
            inputs.hasPose = false;
            inputs.tagCount = 0;
            return;
        }

        inputs.hasPose = true;
        inputs.estimatedPose = estimate.pose;
        inputs.timestampSeconds = estimate.timestampSeconds;
        inputs.tagCount = estimate.tagCount;
        inputs.averageTagDistanceMeters = estimate.avgTagDist;

        double worst = 0.0;
        if (estimate.rawFiducials != null) {
            for (RawFiducial fiducial : estimate.rawFiducials) {
                worst = Math.max(worst, fiducial.ambiguity);
            }
        }
        inputs.ambiguity = worst;
    }

    @Override
    public void setRobotOrientation(double yawDegrees) {
        LimelightHelpers.SetRobotOrientation(name, yawDegrees, 0, 0, 0, 0, 0);
    }

    @Override
    public void setLedsForced(boolean forced) {
        if (forced) {
            LimelightHelpers.setLEDMode_ForceOn(name);
        } else {
            LimelightHelpers.setLEDMode_PipelineControl(name);
        }
    }

    @Override
    public void setThrottle(int throttle) {
        NetworkTableInstance.getDefault()
                .getTable(name)
                .getEntry("throttle_set")
                .setNumber(throttle);
    }
}
