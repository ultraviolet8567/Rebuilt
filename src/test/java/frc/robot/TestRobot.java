package frc.robot;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.SimBattery;

/** Shared helpers for tests that boot the robot in the HAL simulator. */
public final class TestRobot {
    private TestRobot() {}

    /** Initialize the HAL and pretend an enabled Driver Station is attached. */
    public static void init() {
        if (!HAL.initialize(500, 0)) throw new IllegalStateException("HAL failed to initialize");
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setTest(false);
        DriverStationSim.notifyNewData();
        DriverStation.refreshData();
    }

    /** Same as init() but with an alliance assigned before anything is constructed. */
    public static void init(DriverStation.Alliance alliance) {
        init();
        setAlliance(alliance);
    }

    /** Remove the alliance assignment (as before the DS connects). */
    public static void clearAlliance() {
        DriverStationSim.setAllianceStationId(edu.wpi.first.hal.AllianceStationID.Unknown);
        DriverStationSim.notifyNewData();
        DriverStation.refreshData();
    }

    public static void setAlliance(DriverStation.Alliance alliance) {
        DriverStationSim.setAllianceStationId(
                alliance == DriverStation.Alliance.Red
                        ? edu.wpi.first.hal.AllianceStationID.Red1
                        : edu.wpi.first.hal.AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();
        DriverStation.refreshData();
    }

    /** Run N robot loops: scheduler (periodic + simulationPeriodic) and the battery model. */
    public static void loop(int cycles) {
        for (int i = 0; i < cycles; i++) {
            DriverStationSim.notifyNewData();
            DriverStation.refreshData();
            CommandScheduler.getInstance().run();
            SimBattery.update();
            Timer.delay(0.002);
        }
    }

    public static void shutdown() {
        CommandScheduler.getInstance().cancelAll();
        HAL.shutdown();
    }
}
