package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.TestRobot;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class LoggedTunableNumberTest {
    @BeforeAll
    static void boot() {
        TestRobot.init();
    }

    @AfterAll
    static void done() {
        TestRobot.shutdown();
    }

    @Test
    void returnsDefaultUntilDashboardChangesIt() {
        LoggedTunableNumber n = new LoggedTunableNumber("TestNumberA", 4.5);
        assertEquals(4.5, n.get());
        assertEquals(4.5, n.getAsDouble());
    }

    @Test
    void noDefaultMeansZero() {
        assertEquals(0.0, new LoggedTunableNumber("TestNumberB").get());
    }

    @Test
    void hasChangedFiresOncePerCaller() {
        LoggedTunableNumber n = new LoggedTunableNumber("TestNumberC", 1.0);
        assertTrue(n.hasChanged(1), "first check for a caller reports a change");
        assertFalse(n.hasChanged(1), "unchanged value does not report again");
        assertTrue(n.hasChanged(2), "a different caller id gets its own first change");
    }

    @Test
    void ifChangedRunsActionOnlyOnFirstObservation() {
        LoggedTunableNumber a = new LoggedTunableNumber("TestNumberD", 2.0);
        AtomicInteger runs = new AtomicInteger();
        LoggedTunableNumber.ifChanged(99, runs::incrementAndGet, a);
        LoggedTunableNumber.ifChanged(99, runs::incrementAndGet, a);
        assertEquals(1, runs.get());
    }
}
