package frc.robot.testutil;

import edu.wpi.first.networktables.NetworkTableInstance;

/** Starts a local NT server for test runs when explicitly enabled. */
public final class TestNetworkTables {
    private static final String ENABLE_PROPERTY = "test.nt.enabled";

    private static boolean started = false;

    private TestNetworkTables() {}

    public static synchronized void startIfEnabled() {
        if (started || !isEnabled()) {
            return;
        }

        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        instance.stopClient();
        instance.stopServer();
        instance.startServer();
        started = true;
        System.out.println("[TestNetworkTables] Started local NT server (-D" + ENABLE_PROPERTY + "=true)");
    }

    private static boolean isEnabled() {
        return Boolean.getBoolean(ENABLE_PROPERTY);
    }
}
