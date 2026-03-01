package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.WeakHashMap;
import org.littletonrobotics.junction.Logger;

/** Tracks command lifecycle events and publishes telemetry to AdvantageKit logs. */
public final class CommandTelemetry {
    private static final int COMMAND_EVENT_HISTORY_LIMIT = 40;
    // Must stay a power of two because periodic uses a bitmask check.
    private static final int COMMAND_LOG_EVERY_CYCLES = Constants.ENABLE_PERF_LOG_DECIMATION
            ? Constants.PERF_LOG_DECIMATION_CYCLES
            : 1;
    private static final String UNKNOWN_COMMAND_SOURCE = "unknown";

    private final Map<Command, String> commandSources = new WeakHashMap<>();
    private final Map<Command, Double> commandStartTimesSec = new IdentityHashMap<>();
    private final Map<Command, Integer> commandRunIds = new IdentityHashMap<>();
    private final ArrayDeque<String> recentCommandEvents = new ArrayDeque<>();
    private int cycle = 0;
    private int nextCommandRunId = 1;

    /** Registers command lifecycle callbacks with the {@link CommandScheduler}. */
    public void configure() {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.onCommandInitialize(this::handleCommandInitialize);
        scheduler.onCommandFinish(this::handleCommandFinish);
        scheduler.onCommandInterrupt(this::handleCommandInterrupt);
        updateCommandLoggingOutputs(Timer.getFPGATimestamp());
    }

    /** Increments the cycle counter and periodically flushes command logging outputs. */
    public void periodic() {
        cycle++;
        if ((cycle & (COMMAND_LOG_EVERY_CYCLES - 1)) == 0) {
            updateCommandLoggingOutputs(Timer.getFPGATimestamp());
        }
    }

    /** Returns the current periodic cycle count (useful as a per-loop cache key). */
    public int getCycle() {
        return cycle;
    }

    /** Associates a source label with a command for telemetry tracking. */
    public Command withCommandSource(String source, Command command) {
        commandSources.put(command, normalizeCommandSource(source));
        return command;
    }

    /** Schedules commands while recording the source in the command event log. */
    public void schedule(String source, Command... commands) {
        String normalizedSource = normalizeCommandSource(source);
        List<Command> commandsToSchedule = new ArrayList<>();
        List<String> commandNames = new ArrayList<>();

        for (Command command : commands) {
            if (command == null) {
                continue;
            }
            Command trackedCommand = withCommandSource(normalizedSource, command);
            commandsToSchedule.add(trackedCommand);
            commandNames.add(normalizeCommandName(trackedCommand));
        }

        if (commandsToSchedule.isEmpty()) {
            return;
        }

        recordCommandEvent("REQUEST source=" + normalizedSource + " commands=" + String.join(", ", commandNames));
        Logger.recordOutput("commands/lastSchedule/source", normalizedSource);
        Logger.recordOutput("commands/lastSchedule/commands", commandNames.toArray(String[]::new));
        CommandScheduler.getInstance().schedule(commandsToSchedule.toArray(Command[]::new));
    }

    /** Cancels all scheduled commands and records the cancel source. */
    public void cancelAllCommands(String source) {
        String normalizedSource = normalizeCommandSource(source);
        recordCommandEvent("CANCEL_ALL source=" + normalizedSource);
        Logger.recordOutput("commands/lastCancelAllSource", normalizedSource);
        CommandScheduler.getInstance().cancelAll();
    }

    private void handleCommandInitialize(Command command) {
        double nowSec = Timer.getFPGATimestamp();
        int runId = nextCommandRunId++;
        commandStartTimesSec.put(command, nowSec);
        commandRunIds.put(command, runId);

        String commandName = normalizeCommandName(command);
        String source = commandSources.getOrDefault(command, UNKNOWN_COMMAND_SOURCE);
        String requirements = getRequirementsSummary(command);
        String commandKey = sanitizeCommandKey(commandName);

        Logger.recordOutput("commands/" + commandName, true);
        Logger.recordOutput("commands/byName/" + commandKey + "/running", true);
        Logger.recordOutput("commands/byName/" + commandKey + "/activeInstances", countRunningInstances(commandName));
        Logger.recordOutput("commands/lastStarted/name", commandName);
        Logger.recordOutput("commands/lastStarted/source", source);
        Logger.recordOutput("commands/lastStarted/runId", runId);
        Logger.recordOutput("commands/lastStarted/requirements", requirements);

        recordCommandEvent(String.format(
                Locale.US,
                "START run=%d name=%s source=%s requirements=%s",
                runId,
                commandName,
                source,
                requirements));
        updateCommandLoggingOutputs(nowSec);
    }

    private void handleCommandFinish(Command command) {
        handleCommandEnd(command, false);
    }

    private void handleCommandInterrupt(Command command) {
        handleCommandEnd(command, true);
    }

    private void handleCommandEnd(Command command, boolean interrupted) {
        double nowSec = Timer.getFPGATimestamp();
        Double startTimeSec = commandStartTimesSec.remove(command);
        Integer runId = commandRunIds.remove(command);

        String commandName = normalizeCommandName(command);
        String source = commandSources.getOrDefault(command, UNKNOWN_COMMAND_SOURCE);
        String commandKey = sanitizeCommandKey(commandName);
        double durationSec = startTimeSec == null ? 0.0 : nowSec - startTimeSec;
        boolean stillRunning = isCommandNameRunning(commandName);

        Logger.recordOutput("commands/" + commandName, stillRunning);
        Logger.recordOutput("commands/byName/" + commandKey + "/running", stillRunning);
        Logger.recordOutput("commands/byName/" + commandKey + "/activeInstances", countRunningInstances(commandName));
        Logger.recordOutput("commands/lastEnded/name", commandName);
        Logger.recordOutput("commands/lastEnded/source", source);
        Logger.recordOutput("commands/lastEnded/runId", runId == null ? -1 : runId);
        Logger.recordOutput("commands/lastEnded/durationSec", durationSec);
        Logger.recordOutput("commands/lastEnded/interrupted", interrupted);

        recordCommandEvent(String.format(
                Locale.US,
                "%s run=%d name=%s source=%s duration=%.3fs",
                interrupted ? "INTERRUPT" : "FINISH",
                runId == null ? -1 : runId,
                commandName,
                source,
                durationSec));
        updateCommandLoggingOutputs(nowSec);
    }

    private void updateCommandLoggingOutputs(double nowSec) {
        List<Map.Entry<Command, Double>> runningEntries = new ArrayList<>(commandStartTimesSec.entrySet());
        runningEntries.sort(Map.Entry.comparingByValue(Comparator.naturalOrder()));

        List<String> runningNames = new ArrayList<>();
        List<String> runningInstances = new ArrayList<>();

        for (Map.Entry<Command, Double> entry : runningEntries) {
            Command command = entry.getKey();
            String name = normalizeCommandName(command);
            if (!runningNames.contains(name)) {
                runningNames.add(name);
            }
            runningInstances.add(String.format(
                    Locale.US,
                    "run=%d name=%s source=%s elapsed=%.3fs requirements=%s",
                    commandRunIds.getOrDefault(command, -1),
                    name,
                    commandSources.getOrDefault(command, UNKNOWN_COMMAND_SOURCE),
                    nowSec - entry.getValue(),
                    getRequirementsSummary(command)));
        }

        Logger.recordOutput("commands/running/count", runningEntries.size());
        Logger.recordOutput("commands/running/names", runningNames.toArray(String[]::new));
        Logger.recordOutput("commands/running/instances", runningInstances.toArray(String[]::new));
        Logger.recordOutput("commands/recentEvents", recentCommandEvents.toArray(String[]::new));
    }

    private int countRunningInstances(String commandName) {
        int count = 0;
        for (Command runningCommand : commandStartTimesSec.keySet()) {
            if (commandName.equals(normalizeCommandName(runningCommand))) {
                count++;
            }
        }
        return count;
    }

    private boolean isCommandNameRunning(String commandName) {
        return countRunningInstances(commandName) > 0;
    }

    private static String getRequirementsSummary(Command command) {
        List<String> requirementNames = new ArrayList<>();
        for (var requirement : command.getRequirements()) {
            String requirementName = requirement.getName();
            if (requirementName == null || requirementName.isBlank()) {
                requirementName = requirement.getClass().getSimpleName();
            }
            requirementNames.add(requirementName);
        }
        requirementNames.sort(String::compareTo);
        return requirementNames.isEmpty() ? "[]" : "[" + String.join(", ", requirementNames) + "]";
    }

    private void recordCommandEvent(String event) {
        String eventWithTimestamp = String.format(Locale.US, "%.3fs %s", Timer.getFPGATimestamp(), event);
        recentCommandEvents.addLast(eventWithTimestamp);
        while (recentCommandEvents.size() > COMMAND_EVENT_HISTORY_LIMIT) {
            recentCommandEvents.removeFirst();
        }
        Logger.recordOutput("commands/lastEvent", eventWithTimestamp);
    }

    private static String normalizeCommandName(Command command) {
        if (command == null) {
            return "UnnamedCommand";
        }
        String name = command.getName();
        if (name == null || name.isBlank()) {
            String fallbackName = command.getClass().getSimpleName();
            if (fallbackName == null || fallbackName.isBlank()) {
                return "UnnamedCommand";
            }
            return fallbackName;
        }
        return name;
    }

    private static String normalizeCommandSource(String source) {
        return source == null || source.isBlank() ? UNKNOWN_COMMAND_SOURCE : source;
    }

    private static String sanitizeCommandKey(String key) {
        StringBuilder sanitized = new StringBuilder(key.length());
        for (int i = 0; i < key.length(); i++) {
            char character = key.charAt(i);
            if (Character.isLetterOrDigit(character) || character == '_' || character == '-') {
                sanitized.append(character);
            } else {
                sanitized.append('_');
            }
        }
        return sanitized.isEmpty() ? "unnamed" : sanitized.toString();
    }
}
