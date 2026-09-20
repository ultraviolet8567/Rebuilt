package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Translation3d;

/** Camera mounting and pose-filtering parameters. */
public final class VisionConstants {
    private VisionConstants() {}

    public static final String kCameraName = "limelight-nip";

    /** Camera position in robot space, metres. */
    public static final Translation3d kCameraOffsetMeters = new Translation3d(-0.051, 0.27, 0.495);

    /**
     * Camera mounting angles in DEGREES -- {@code LimelightHelpers.setCameraPose_RobotSpace} takes
     * degrees. These were previously wrapped in a {@code Rotation3d}, whose constructor takes
     * RADIANS, so the -30 degree pitch was being sent to the Limelight as 1.4 degrees and every tag
     * range it computed was wrong.
     */
    public static final double kCameraRollDegrees = 0.0;

    public static final double kCameraPitchDegrees = -30.0;
    public static final double kCameraYawDegrees = 0.0;

    // -------------------------------------------------------------- filtering

    /** Reject a single-tag pose whose solver ambiguity is worse than this. */
    public static final double kMaxSingleTagAmbiguity = 0.3;

    /** Reject any pose further than this outside the field boundary, metres. */
    public static final double kFieldMarginMeters = 0.5;

    /**
     * Reject vision while spinning faster than this, rad/s. During a fast spin the rolling shutter
     * smears the tag and MegaTag2's answer is built on a gyro angle that has already moved.
     */
    public static final double kMaxYawRateRadPerSec = 2.0 * Math.PI;

    /**
     * Reject tags further away than this, metres -- the pose noise grows faster than the signal.
     */
    public static final double kMaxAverageTagDistanceMeters = 6.0;

    /**
     * Base translational standard deviation at one metre with one tag, metres.
     *
     * <p>The final value scales as {@code base * distance^2 / tagCount^2}: doubling the range
     * quadruples the uncertainty, and a second tag quarters it. The previous code used a fixed
     * (0.5, 0.5) regardless of range or tag count, so a lone tag glimpsed across the field was
     * trusted exactly as much as two tags two metres away. 34 of the 42 corpus robots scale this
     * way; the exponents here follow FRC 341's.
     */
    public static final double kLinearStdDevBase = 0.05;

    /**
     * Rotational standard deviation. With MegaTag2 the reported rotation is derived from the gyro
     * yaw we sent the camera, so fusing it back in just lets vision argue with the gyro. Infinite
     * means "ignore".
     */
    public static final double kAngularStdDev = Double.POSITIVE_INFINITY;
}
