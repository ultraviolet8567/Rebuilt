package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Alerts;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.Logger;

/**
 * AprilTag pose estimation, and the decision about how much to believe it.
 *
 * <p>The previous code fed every MegaTag2 result straight into the pose estimator with a fixed
 * standard deviation, so a single tag seen edge-on at eight metres moved the robot's pose estimate
 * exactly as hard as two tags at two metres. That is the mechanism behind "the robot suddenly
 * teleports on the field display" and behind auto-aim snapping to the wrong heading.
 *
 * <p>Two layers now stand between the camera and the pose estimator:
 *
 * <ol>
 *   <li><b>Rejection.</b> Drop frames with no tags, an ambiguous single tag, a pose outside the
 *       field, tags beyond useful range, or any frame captured while the robot was spinning fast
 *       enough to smear them.
 *   <li><b>Weighting.</b> What survives is admitted with a standard deviation that grows with the
 *       square of the tag distance and shrinks with the square of the tag count, so distant and
 *       lonely tags nudge the estimate instead of yanking it.
 * </ol>
 */
public class Vision extends SubsystemBase {
    private final Drive drive;
    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private final Alert disconnected;

    public Vision(Drive drive, VisionIO io) {
        this.drive = drive;
        this.io = io;
        this.disconnected =
                Alerts.create(
                        "Camera disconnected: " + VisionConstants.kCameraName, AlertType.kWarning);
        drive.setDefaultVisionStdDevs();
    }

    @Override
    public void periodic() {
        // MegaTag2 needs the gyro heading BEFORE it solves this frame, so send it first.
        io.setRobotOrientation(drive.getGyroRotation().getDegrees());

        io.updateInputs(inputs);
        Logger.processInputs("Vision/" + VisionConstants.kCameraName, inputs);
        disconnected.set(!inputs.connected);

        if (!inputs.hasPose) {
            Logger.recordOutput("Vision/Accepted", false);
            Logger.recordOutput("Vision/RejectReason", "no pose");
            return;
        }

        String reject = rejectionReason(inputs.estimatedPose);
        Logger.recordOutput("Vision/RejectReason", reject == null ? "" : reject);
        Logger.recordOutput("Vision/Accepted", reject == null);
        Logger.recordOutput("Vision/RawPose", inputs.estimatedPose);

        if (reject != null) {
            return;
        }

        double linearStdDev =
                VisionConstants.kLinearStdDevBase
                        * Math.pow(inputs.averageTagDistanceMeters, 2.0)
                        / Math.pow(inputs.tagCount, 2.0);

        Logger.recordOutput("Vision/LinearStdDev", linearStdDev);

        drive.addVisionMeasurement(
                inputs.estimatedPose,
                inputs.timestampSeconds,
                VecBuilder.fill(linearStdDev, linearStdDev, VisionConstants.kAngularStdDev));
    }

    /** Null if the pose is usable, otherwise a short reason, logged for post-match review. */
    private String rejectionReason(Pose2d pose) {
        if (inputs.tagCount == 0) {
            return "no tags";
        }
        if (inputs.tagCount == 1 && inputs.ambiguity > VisionConstants.kMaxSingleTagAmbiguity) {
            return "ambiguous single tag";
        }
        if (inputs.averageTagDistanceMeters > VisionConstants.kMaxAverageTagDistanceMeters) {
            return "tags too far";
        }
        if (Math.abs(drive.getYawVelocityRadPerSec()) > VisionConstants.kMaxYawRateRadPerSec) {
            return "spinning too fast";
        }
        double m = VisionConstants.kFieldMarginMeters;
        if (pose.getX() < -m
                || pose.getX() > FieldConstants.kFieldLengthMeters + m
                || pose.getY() < -m
                || pose.getY() > FieldConstants.kFieldWidthMeters + m) {
            return "off field";
        }
        return null;
    }

    /** Full brightness while disabled so the pit crew can see the camera is alive. */
    public void setDisabledMode(boolean disabled) {
        io.setLedsForced(disabled);
        io.setThrottle(disabled ? 200 : 0);
    }
}
