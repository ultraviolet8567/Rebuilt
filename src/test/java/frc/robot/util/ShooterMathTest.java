package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * Locks in the empirically fitted distance-to-velocity curve in {@link ShooterMath}.
 *
 * <p>If the calibration is re-fitted, update the expected values here. A failure means the shot
 * speed for a known distance changed, which is a robot-level behavior change, not a formatting nit.
 */
class ShooterMathTest {
    private static final double kTolerance = 0.01;

    @Test
    void velocityAtZeroDistanceMatchesCalibration() {
        assertEquals(1310.13, ShooterMath.targetVelocityForDistance(0.0), kTolerance);
    }

    @Test
    void velocityMatchesCalibrationPoints() {
        assertEquals(2452.37, ShooterMath.targetVelocityForDistance(1.0), kTolerance);
        assertEquals(3338.43, ShooterMath.targetVelocityForDistance(2.0), kTolerance);
        assertEquals(3968.31, ShooterMath.targetVelocityForDistance(3.0), kTolerance);
        assertEquals(4342.01, ShooterMath.targetVelocityForDistance(4.0), kTolerance);
    }

    @Test
    void velocityIncreasesWithDistanceAcrossCompetitionRange() {
        double previous = ShooterMath.targetVelocityForDistance(1.5);
        for (double distance = 1.55; distance <= 4.5; distance += 0.05) {
            double current = ShooterMath.targetVelocityForDistance(distance);
            assertTrue(
                    current > previous,
                    "Velocity must increase with distance: at "
                            + distance
                            + " m got "
                            + current
                            + " after "
                            + previous);
            previous = current;
        }
    }

    @Test
    void demoScaleReducesVelocity() {
        assertEquals(550.0, ShooterMath.scaleForDemo(2200.0, 0.25), kTolerance);
        assertTrue(ShooterMath.scaleForDemo(2200.0, 1.0) > ShooterMath.scaleForDemo(2200.0, 0.25));
    }
}
