package frc.robot;

import java.io.FileInputStream;
import java.util.Arrays;
import java.util.Properties;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.Topic;

/**
 * Laptop-side check. Connects to the controller as a NetworkTables client, lists the topics, and
 * compares /DeployTest/nonce with the nonce of the local build.
 *
 * <p>Arguments: address, path of build-info.properties, expect ("true" fails when /DeployTest is
 * absent or the nonce is different; "false" only lists what is on the controller).
 */
public final class NtProbe {
    private NtProbe() {}

    public static void main(String[] args) throws Exception {
        String address = args[0];
        boolean expect = Boolean.parseBoolean(args[2]);
        Properties local = new Properties();
        try (FileInputStream in = new FileInputStream(args[1])) {
            local.load(in);
        } catch (Exception e) {
            System.out.println("Local build-info.properties not read: " + e.getMessage());
        }
        String localNonce = local.getProperty("nonce", "none");

        NetworkTableInstance nt = NetworkTableInstance.create();
        nt.startClient("deploy-test-probe");
        nt.setServer(address, 5810);

        long deadline = System.currentTimeMillis() + 10_000;
        while (!nt.isConnected() && System.currentTimeMillis() < deadline) {
            Thread.sleep(100);
        }
        if (!nt.isConnected()) {
            System.out.println("RESULT: no NetworkTables server at " + address + ":5810");
            System.exit(expect ? 1 : 0);
        }
        System.out.println("Connected to " + address + ":5810");

        // Subscribe to all topics, then wait for the values.
        var all = new org.wpilib.networktables.MultiSubscriber(nt, new String[] {"/"});
        Thread.sleep(1500);

        Topic[] topics = nt.getTopics();
        String[] names = Arrays.stream(topics).map(Topic::getName).sorted().toArray(String[]::new);
        System.out.println("Topics on the controller: " + names.length);
        for (String name : names) {
            if (!name.startsWith("/DeployTest/")) {
                System.out.println("  " + name);
            }
        }

        NetworkTable table = nt.getTable("DeployTest");
        String remoteNonce = table.getEntry("nonce").getString("absent");
        long c1 = table.getEntry("counter").getInteger(-1);
        Thread.sleep(1000);
        long c2 = table.getEntry("counter").getInteger(-1);

        System.out.println("/DeployTest");
        for (String key : new String[] {"nonce", "buildTime", "gradleRio", "javaVersion",
            "osArch", "kernel", "hostname"}) {
            System.out.println("  " + key + " = " + table.getEntry(key).getString("absent"));
        }
        System.out.println("  uptimeSeconds = " + table.getEntry("uptimeSeconds").getDouble(-1));
        System.out.println("  counter = " + c1 + " then " + c2 + " after 1 s");
        System.out.println("  osRelease:");
        for (String line : table.getEntry("osRelease").getString("absent").split("\n")) {
            System.out.println("    " + line);
        }
        System.out.println("Local build nonce = " + localNonce);

        boolean nonceOk = remoteNonce.equals(localNonce);
        boolean counterOk = c1 >= 0 && c2 > c1;
        if (nonceOk && counterOk) {
            System.out.println("RESULT: PASS. The controller runs this build and the counter advances.");
        } else if ("absent".equals(remoteNonce)) {
            System.out.println("RESULT: /DeployTest is not on the controller.");
        } else {
            System.out.println("RESULT: FAIL. nonce match=" + nonceOk + ", counter advances=" + counterOk);
        }
        all.close();
        nt.close();
        System.exit(!expect || (nonceOk && counterOk) ? 0 : 1);
    }
}
