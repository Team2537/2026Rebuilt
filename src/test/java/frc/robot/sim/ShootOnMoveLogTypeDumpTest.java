package frc.robot.sim;

import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import org.junit.jupiter.api.Test;

class ShootOnMoveLogTypeDumpTest {
    @Test
    void dumpSelectedEntryTypes() throws Exception {
        Path wpilog = Path.of("logs/akit_26-03-18_22-40-46.wpilog").toAbsolutePath();
        DataLogReader reader = new DataLogReader(wpilog.toString());
        for (DataLogRecord record : recordsUntilFailure(reader)) {
            if (!record.isStart()) {
                continue;
            }
            var start = record.getStartData();
            String name = start.name;
            if (name.contains("/DriverStation/Joystick0/AxisValues")
                    || name.contains("/DriverStation/Joystick0/ButtonValues")
                    || name.contains("/DriverStation/Joystick0/POVs")
                    || name.contains("/RealOutputs/Shooting/")
                    || name.contains("/RealOutputs/ShotYaw/")
                    || name.contains("/RealOutputs/Shooter/CompensatedHub")
                    || name.contains("/RealOutputs/SwerveChassisSpeeds/Measured")
                    || name.contains("/RealOutputs/SwerveChassisSpeeds/Requested")
                    || name.contains("/RealOutputs/Odometry/Robot")
                    || name.contains("/RealOutputs/Commands/lastEvent")) {
                System.out.println(start.entry + " | " + start.type + " | " + name);
            }
        }
    }

    private static List<DataLogRecord> recordsUntilFailure(DataLogReader reader) {
        List<DataLogRecord> records = new ArrayList<>();
        Iterator<DataLogRecord> iterator = reader.iterator();
        while (true) {
            try {
                if (!iterator.hasNext()) {
                    break;
                }
                records.add(iterator.next());
            } catch (IllegalArgumentException exception) {
                break;
            }
        }
        return records;
    }
}
