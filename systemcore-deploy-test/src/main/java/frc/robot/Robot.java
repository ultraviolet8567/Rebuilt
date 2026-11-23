package frc.robot;

import java.io.InputStream;
import java.net.InetAddress;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Properties;
import org.wpilib.framework.TimedRobot;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;

/**
 * Deploy test. Publishes the build nonce and a counter to the NetworkTables table /DeployTest.
 *
 * <p>This program makes no motor, CAN or SmartIO objects. It is safe with any wiring on the
 * controller. The counter runs in robotPeriodic, so it runs without a Driver Station.
 */
public class Robot extends TimedRobot {
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("DeployTest");
    private final long startNanos = System.nanoTime();
    private final String nonce;
    private long counter = 0;

    public Robot() {
        Properties info = new Properties();
        try (InputStream in = Robot.class.getResourceAsStream("/build-info.properties")) {
            if (in != null) {
                info.load(in);
            }
        } catch (Exception e) {
            System.out.println("DeployTest: build-info.properties not read: " + e);
        }
        nonce = info.getProperty("nonce", "unknown");

        table.getEntry("nonce").setString(nonce);
        table.getEntry("buildTime").setString(info.getProperty("buildTime", "unknown"));
        table.getEntry("gradleRio").setString(info.getProperty("gradleRio", "unknown"));
        table.getEntry("javaVersion").setString(System.getProperty("java.version", "unknown"));
        table.getEntry("osArch").setString(System.getProperty("os.arch", "unknown"));
        table.getEntry("kernel").setString(System.getProperty("os.version", "unknown"));
        table.getEntry("hostname").setString(hostname());
        // The OS image version. WPILib 2027 alpha-8 needs Systemcore image 14 or later.
        table.getEntry("osRelease").setString(readFile("/etc/os-release"));

        System.out.println("DeployTest: started, nonce=" + nonce);
    }

    @Override
    public void robotPeriodic() {
        counter++;
        table.getEntry("counter").setInteger(counter);
        table.getEntry("uptimeSeconds").setDouble((System.nanoTime() - startNanos) / 1e9);
        if (counter % 250 == 0) {
            System.out.println("DeployTest: nonce=" + nonce + " counter=" + counter);
        }
    }

    private static String hostname() {
        try {
            return InetAddress.getLocalHost().getHostName();
        } catch (Exception e) {
            return "unknown";
        }
    }

    private static String readFile(String path) {
        try {
            return Files.readString(Path.of(path)).trim();
        } catch (Exception e) {
            return "not read: " + e.getClass().getSimpleName();
        }
    }
}
