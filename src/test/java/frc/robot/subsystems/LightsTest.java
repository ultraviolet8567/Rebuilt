package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.TestRobot;
import frc.robot.subsystems.Lights.RobotState;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class LightsTest {
    private static Lights lights;
    private static AddressableLEDSim ledSim;

    @BeforeAll
    static void boot() {
        TestRobot.init();
        lights = Lights.getInstance();
        ledSim = new AddressableLEDSim();
    }

    @AfterAll
    static void done() {
        TestRobot.shutdown();
    }

    private static void run(int n) {
        for (int i = 0; i < n; i++) lights.run();
    }

    private static int litCount(int from, int to) {
        byte[] data = ledSim.getData();
        int lit = 0;
        for (int i = from; i < to; i++) {
            int b = data[i * 4] & 0xff, g = data[i * 4 + 1] & 0xff, r = data[i * 4 + 2] & 0xff;
            if (r + g + b > 0) lit++;
        }
        return lit;
    }

    @Test
    void everyStateRendersWithoutThrowingAndLightsTheStrip() {
        RoboRioSim.setVInVoltage(12.5);
        for (RobotState s : RobotState.values()) {
            lights.state = s;
            run(40);
            assertTrue(litCount(0, 42) > 0, "state " + s + " should light some LEDs");
        }
    }

    @Test
    void lowBatteryIsDetectedBelowTenVolts() {
        RoboRioSim.setVInVoltage(12.5);
        run(20);
        assertFalse(lights.lowBattery);
        RoboRioSim.setVInVoltage(9.0);
        run(20);
        assertTrue(lights.lowBattery);
        RoboRioSim.setVInVoltage(12.5);
    }
}
