package frc.robot.util;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import java.util.Locale;

/** Sends toast notifications to the Elastic dashboard via NetworkTables. */
public final class ElasticNotifications {
    private static final ObjectMapper mapper = new ObjectMapper();
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
        NotificationPayload payload = new NotificationPayload(
                level.name().toLowerCase(Locale.US),
                title == null ? "" : title,
                description == null ? "" : description,
                level == Level.ERROR ? 5000 : 3000,
                350,
                -1);
        try {
            publisher.set(mapper.writeValueAsString(payload));
        } catch (JsonProcessingException exception) {
            publisher.set("{\"level\":\"error\",\"title\":\"Notification Serialization\",\"description\":\"Failed to serialize Elastic notification payload\",\"displayTime\":5000,\"width\":350,\"height\":-1}");
        }
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

    private record NotificationPayload(
            String level,
            String title,
            String description,
            int displayTime,
            int width,
            int height) {}
}
