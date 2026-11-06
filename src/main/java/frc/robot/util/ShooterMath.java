package frc.robot.util;

/**
 * Pure shooter calculations, kept free of hardware so they can be unit tested.
 *
 * <p>{@link frc.robot.subsystems.Shooter.Flywheel} owns the motors and calls into this class. If
 * the distance-to-speed calibration changes, update it here and adjust {@code ShooterMathTest}.
 */
public final class ShooterMath {
    private ShooterMath() {}

    /**
     * Flywheel speed, in encoder velocity units, that lands a shot from {@code distanceMeters}
     * away.
     *
     * <p>This is an empirically fitted curve, not physics. It was measured on the real robot by
     * finding the speed that scored from several distances and fitting a quadratic.
     */
    public static double targetVelocityForDistance(double distanceMeters) {
        return 1310.13 + 1270.33 * distanceMeters - 128.09 * distanceMeters * distanceMeters;
    }

    /** Scales a velocity down for demo mode so the robot does not run at competition speed. */
    public static double scaleForDemo(double velocity, double scaleFactor) {
        return velocity * scaleFactor;
    }
}
