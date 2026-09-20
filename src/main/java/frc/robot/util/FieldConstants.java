package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Fixed locations on the 2026 REBUILT field, in WPILib blue-origin coordinates. */
public final class FieldConstants {
    private FieldConstants() {}

    public static final double kFieldLengthMeters = 16.54;
    public static final double kFieldWidthMeters = 8.21;

    public static final Pose2d kRedHub = new Pose2d(11.91, 4.041, Rotation2d.kZero);
    public static final Pose2d kBlueHub = new Pose2d(4.623, 4.041, Rotation2d.kZero);

    /** Vertical distance from the shooter exit to the hub opening, metres. */
    public static final double kHubHeightDiff = 1.0885;

    public static final double kG = 9.8;

    /** The hub our alliance is scoring in. */
    public static Translation2d hubTranslation(boolean redAlliance) {
        return (redAlliance ? kRedHub : kBlueHub).getTranslation();
    }
}
