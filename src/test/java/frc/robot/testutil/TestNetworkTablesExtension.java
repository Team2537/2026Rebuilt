package frc.robot.testutil;

import org.junit.jupiter.api.extension.BeforeAllCallback;
import org.junit.jupiter.api.extension.ExtensionContext;

/** Global JUnit extension that optionally exposes test SmartDashboard/NT traffic. */
public final class TestNetworkTablesExtension implements BeforeAllCallback {
    @Override
    public void beforeAll(ExtensionContext context) {
        TestNetworkTables.startIfEnabled();
    }
}
