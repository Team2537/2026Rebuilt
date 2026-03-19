package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/** Lightweight dashboard-backed tunable number with AdvantageKit logging. */
public final class LoggedTunableNumber {
    private final String key;
    private double defaultValue;
    private boolean initialized = false;
    private final Map<Integer, Double> lastValuesById = new HashMap<>();

    public LoggedTunableNumber(String key) {
        this.key = key;
    }

    public LoggedTunableNumber(String key, double defaultValue) {
        this.key = key;
        initDefault(defaultValue);
    }

    public void initDefault(double defaultValue) {
        this.defaultValue = defaultValue;
        if (!initialized) {
            SmartDashboard.putNumber(key, defaultValue);
            initialized = true;
        }
    }

    public double get() {
        double value = SmartDashboard.getNumber(key, defaultValue);
        Logger.recordOutput(key, value);
        return value;
    }

    public boolean hasChanged(int id) {
        double value = get();
        Double lastValue = lastValuesById.put(id, value);
        return lastValue == null || Double.compare(lastValue, value) != 0;
    }
}
