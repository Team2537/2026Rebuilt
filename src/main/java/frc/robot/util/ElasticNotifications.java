package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import java.util.Locale;

/** Sends toast notifications to the Elastic dashboard via NetworkTables. */
public final class ElasticNotifications {
    private static final StringPublisher publisher;

    static {
        StringTopic topic = NetworkTableInstance.getDefault()
                .getStringTopic("/Elastic/RobotNotifications");
        publisher = topic.publish();
    }

    private ElasticNotifications() {}

    public enum Level {
        INFO,
        WARNING,
        ERROR
    }

    public static void send(Level level, String title, String description) {
        String json = String.format(
                Locale.US,
                "{\"level\":\"%s\",\"title\":\"%s\",\"description\":\"%s\",\"displayTime\":%d,\"width\":350,\"height\":-1}",
                level.name().toLowerCase(Locale.US),
                escapeJson(title),
                escapeJson(description),
                level == Level.ERROR ? 5000 : 3000);
        publisher.set(json);
    }

    public static void sendInfo(String title, String description) {
        send(Level.INFO, title, description);
    }

    public static void sendWarning(String title, String description) {
        send(Level.WARNING, title, description);
    }

    public static void sendError(String title, String description) {
        send(Level.ERROR, title, description);
    }

    private static String escapeJson(String text) {
        if (text == null) {
            return "";
        }
        return text.replace("\\", "\\\\").replace("\"", "\\\"").replace("\n", "\\n");
    }
}
