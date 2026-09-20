package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants.ShooterConstants;
import frc.robot.TestRobot;
import frc.robot.subsystems.Shooter.Flywheel;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class ShooterMathTest {
    private static Flywheel flywheel;

    @BeforeAll
    static void boot() {
        TestRobot.init();
        Lights.getInstance();
        flywheel = new Flywheel();
    }

    @AfterAll
    static void done() {
        TestRobot.shutdown();
    }

    @Test
    void lookupTableMatchesTheFittedCurveInTheTunedRange() {
        assertEquals(3338, flywheel.calculateTargetVelocity(2.0), 5);
        assertEquals(4342, flywheel.calculateTargetVelocity(4.0), 5);
        assertTrue(
                flywheel.calculateTargetVelocity(3.0) > flywheel.calculateTargetVelocity(2.0),
                "speed increases with distance inside the tuned 2-5 m range");
    }

    @Test
    void lookupTableStillDecreasesPastFiveMetersUntilRefit() {
        // Documents the known limitation (TODO in Flywheel). If this starts failing, the
        // curve was refit and this test should be updated to the new behavior.
        assertTrue(flywheel.calculateTargetVelocity(7.0) < flywheel.calculateTargetVelocity(5.0));
    }

    @Test
    void demoModeScalesTheCommandedVelocityWithoutOverwritingIt() {
        Lights.getInstance().isDemo = false;
        flywheel.setTargetVelocity(4000);
        assertEquals(4000, flywheel.getTargetVelocity());
        Lights.getInstance().isDemo = true;
        assertEquals(
                4000 * ShooterConstants.shooterDemoScaleFactor, flywheel.getTargetVelocity(), 1e-9);
        // Reading in demo mode must not clobber the stored value (the old regression).
        Lights.getInstance().isDemo = false;
        assertEquals(4000, flywheel.getTargetVelocity());
    }
}
