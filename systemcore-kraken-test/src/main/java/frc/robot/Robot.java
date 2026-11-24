package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import java.io.InputStream;
import java.net.InetAddress;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Properties;
import org.wpilib.framework.TimedRobot;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;

/**
 * Kraken bench test. Turns one Kraken (TalonFX) at a low open-loop voltage while the Driver
 * Station is enabled in Teleoperated mode.
 *
 * <p>WARNING: this program turns a motor. Clamp the motor and keep the shaft clear before Enable.
 *
 * <p>Open-loop voltage control uses no position or velocity feedback, so no external encoder is
 * necessary. The speed in /KrakenTest comes from the sensor inside the motor.
 *
 * <p>The program also publishes the build nonce and a counter to /DeployTest, the same as the
 * deploy test, so ntProbe can confirm which build is on the controller.
 */
public class Robot extends TimedRobot {
    /** CAN ID of the Kraken. 0 is the factory default, seen on the bus on 2026-11-24. */
    private static final int KRAKEN_CAN_ID = 0;

    /** Bus 1 is can_s1. On the 2-channel HAT this is the connector with the label CAN0. */
    private static final int KRAKEN_CAN_BUS = 1;

    /** Limit for the commanded voltage. A free Kraken X60 turns at about 500 rpm per volt. */
    private static final double MAX_VOLTS = 2.0;

    private static final double DEFAULT_VOLTS = 1.0;
    private static final double STATOR_LIMIT_AMPS = 20.0;

    private final TalonFX kraken = new TalonFX(KRAKEN_CAN_ID, CANBus.systemcore(KRAKEN_CAN_BUS));
    private final VoltageOut voltageOut = new VoltageOut(0.0);
    private final NeutralOut neutralOut = new NeutralOut();
    private final StatusSignal<?> firmware = kraken.getVersion();
    private final StatusSignal<?> supplyVolts = kraken.getSupplyVoltage();
    private final StatusSignal<?> motorVolts = kraken.getMotorVoltage();
    private final StatusSignal<?> statorAmps = kraken.getStatorCurrent();
    private final StatusSignal<?> rotorRps = kraken.getVelocity();
    private final StatusSignal<?> tempC = kraken.getDeviceTemp();
    private final NetworkTable krakenTable =
            NetworkTableInstance.getDefault().getTable("KrakenTest");
    private double appliedCommandVolts = 0.0;

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

        // Only the current-limit group is written. Other settings on the motor stay as they are.
        StatusCode limitStatus = kraken.getConfigurator().apply(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(STATOR_LIMIT_AMPS)
                        .withStatorCurrentLimitEnable(true));
        krakenTable.getEntry("canId").setInteger(KRAKEN_CAN_ID);
        krakenTable.getEntry("canBus").setString("can_s" + KRAKEN_CAN_BUS);
        krakenTable.getEntry("currentLimitStatus").setString(limitStatus.toString());
        // Change this value from a NetworkTables client to set the voltage. Limit: MAX_VOLTS.
        krakenTable.getEntry("commandVolts").setDouble(DEFAULT_VOLTS);

        System.out.println("DeployTest: started, nonce=" + nonce);
        System.out.println("KrakenTest: TalonFX id " + KRAKEN_CAN_ID + " on can_s" + KRAKEN_CAN_BUS
                + ", current limit config: " + limitStatus);
    }

    @Override
    public void teleopPeriodic() {
        double volts = krakenTable.getEntry("commandVolts").getDouble(0.0);
        appliedCommandVolts = Math.max(-MAX_VOLTS, Math.min(MAX_VOLTS, volts));
        kraken.setControl(voltageOut.withOutput(appliedCommandVolts));
    }

    @Override
    public void teleopExit() {
        stopKraken();
    }

    @Override
    public void disabledInit() {
        stopKraken();
    }

    private void stopKraken() {
        appliedCommandVolts = 0.0;
        kraken.setControl(neutralOut);
    }

    private void publishKraken() {
        BaseStatusSignal.refreshAll(firmware, supplyVolts, motorVolts, statorAmps, rotorRps, tempC);
        int v = (int) firmware.getValueAsDouble();
        krakenTable.getEntry("connected").setBoolean(kraken.isConnected());
        krakenTable.getEntry("firmware").setString(
                ((v >> 24) & 0xFF) + "." + ((v >> 16) & 0xFF) + "." + ((v >> 8) & 0xFF) + "." + (v & 0xFF));
        krakenTable.getEntry("supplyVolts").setDouble(supplyVolts.getValueAsDouble());
        krakenTable.getEntry("motorVolts").setDouble(motorVolts.getValueAsDouble());
        krakenTable.getEntry("statorAmps").setDouble(statorAmps.getValueAsDouble());
        krakenTable.getEntry("rotorRpm").setDouble(rotorRps.getValueAsDouble() * 60.0);
        krakenTable.getEntry("tempC").setDouble(tempC.getValueAsDouble());
        krakenTable.getEntry("appliedCommandVolts").setDouble(appliedCommandVolts);
    }

    @Override
    public void robotPeriodic() {
        counter++;
        table.getEntry("counter").setInteger(counter);
        table.getEntry("uptimeSeconds").setDouble((System.nanoTime() - startNanos) / 1e9);
        publishKraken();
        if (counter % 250 == 0) {
            System.out.println("DeployTest: nonce=" + nonce + " counter=" + counter
                    + " kraken connected=" + kraken.isConnected()
                    + " rpm=" + Math.round(rotorRps.getValueAsDouble() * 60.0));
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
