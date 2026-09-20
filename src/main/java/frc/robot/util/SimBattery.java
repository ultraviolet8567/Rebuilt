package frc.robot.util;

import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import org.littletonrobotics.junction.Logger;

/**
 * Desktop-simulation battery model. Every simulated mechanism reports the current it is drawing
 * each loop with {@link #addCurrent(double)}; {@link #update()} (called from
 * Robot.simulationPeriodic) turns the total into a loaded battery voltage and hands it to the
 * simulated roboRIO, so RobotController.getBatteryVoltage() and the Lights low-battery warning
 * behave like they do on the real robot.
 *
 * <p>Does nothing useful on the real robot; the calls are cheap and are simply never made because
 * the subsystems only run their sim code when RobotBase.isSimulation() is true.
 */
public final class SimBattery {
    private static double totalCurrentAmps = 0.0;

    private SimBattery() {}

    /** Report the current (amps) drawn by one simulated motor this loop. */
    public static void addCurrent(double amps) {
        totalCurrentAmps += Math.abs(amps);
    }

    /** Compute the loaded battery voltage from all reported currents and reset for next loop. */
    public static void update() {
        double voltage = BatterySim.calculateDefaultBatteryLoadedVoltage(totalCurrentAmps);
        voltage = Math.max(0.0, voltage);
        RoboRioSim.setVInVoltage(voltage);

        Logger.recordOutput("Sim/BatteryVoltage", voltage);
        Logger.recordOutput("Sim/TotalCurrentAmps", totalCurrentAmps);

        totalCurrentAmps = 0.0;
    }
}
