package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private enum GoalState {
        RETRACTED,
        EXTENDED
    }

    private enum MotionState {
        UNKNOWN,
        RETRACTED,
        EXTENDED,
        MOVING_TO_RETRACTED,
        MOVING_TO_EXTENDED,
        HOMING
    }

    private enum SmartRetractMode {
        DISABLED,
        NIBBLE,
        CURRENT_HOLD
    }

    private static final class MotionProfile {
        private final double velocityRotPerSec;
        private final double accelerationRotPerSecSq;
        private final double maxVolts;

        private MotionProfile(double velocityRotPerSec, double accelerationRotPerSecSq, double maxVolts) {
            this.velocityRotPerSec = velocityRotPerSec;
            this.accelerationRotPerSecSq = accelerationRotPerSecSq;
            this.maxVolts = maxVolts;
        }
    }

    private static final class SmartRetractSession {
        private SmartRetractMode mode = SmartRetractMode.DISABLED;
        private boolean startedExtended = false;
        private boolean active = false;
        private boolean nibbleBackoffActive = false;
        private int nibbleSpikeCycles = 0;
        private double nibbleBackoffUntilSec = Double.NaN;
        private double commandedLeftTargetRot = Double.NaN;
        private double filteredSignalCurrentAmps = 0.0;
        private double baselineSignalCurrentAmps = 0.0;
        private boolean feedLatched = false;
        private int feedTrueCycles = 0;
        private int feedFalseCycles = 0;
    }

    private static final String DASHBOARD_SMART_RETRACT_ENABLE_NIBBLE_KEY = "Intake/SmartRetract/EnableNibble";
    private static final String DASHBOARD_SMART_RETRACT_ENABLE_CURRENT_HOLD_KEY = "Intake/SmartRetract/EnableCurrentHold";
    private static final String DASHBOARD_SMART_RETRACT_STATUS_MODE_KEY = "Intake/SmartRetract/StatusMode";

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private GoalState goalState = GoalState.RETRACTED;
    private MotionState motionState = MotionState.RETRACTED;
    private boolean leftHomeSucceeded = false;
    private boolean rightHomeSucceeded = false;
    private MotionState preHomeMotionState = MotionState.RETRACTED;
    private GoalState preHomeGoalState = GoalState.RETRACTED;
    private boolean cachedSmartRetractNibbleEnabled = false;
    private boolean cachedSmartRetractCurrentHoldEnabled = false;
    private SmartRetractMode cachedSmartRetractMode = SmartRetractMode.DISABLED;

    public Intake(IntakeIO io) {
        super("intake");
        this.io = io;
        initDashboardSmartRetractEntries();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        refreshDashboardSmartRetractMode();

        if (DriverStation.isDisabled()) {
            io.stop();
        }

        updateMotionStateFromSensors();
        Logger.recordOutput("Intake/Extended", isExtended());
        Logger.recordOutput("Intake/RequestedExtended", isGoalExtended());
        Logger.recordOutput("Intake/GoalState", goalState.name());
        Logger.recordOutput("Intake/MotionState", motionState.name());
        Logger.recordOutput("Intake/SmartRetract/EnableNibble", cachedSmartRetractNibbleEnabled);
        Logger.recordOutput("Intake/SmartRetract/EnableCurrentHold", cachedSmartRetractCurrentHoldEnabled);
        Logger.recordOutput("Intake/SmartRetract/BothModesEnabled",
                cachedSmartRetractNibbleEnabled && cachedSmartRetractCurrentHoldEnabled);
        Logger.recordOutput("Intake/SmartRetract/SelectedMode", cachedSmartRetractMode.name());
    }

    public void setExtended(boolean isExtended) {
        goalState = isExtended ? GoalState.EXTENDED : GoalState.RETRACTED;
        motionState = atGoalState(goalState);
        MotionProfile profile = standardMotionProfile();
        requestIntakePosition(
                targetRotations(goalState),
                profile.velocityRotPerSec,
                profile.accelerationRotPerSecSq,
                profile.maxVolts);
    }

    public boolean isExtended() {
        return motionState == MotionState.EXTENDED;
    }

    public Command retractCommand() {
        return moveToGoalCommand(
                GoalState.RETRACTED,
                standardMotionProfile(),
                "IntakeWaitForPosition").withName("IntakeRetract");
    }

    public Command slowRetractCommand() {
        return moveToGoalCommand(
                GoalState.RETRACTED,
                slowRetractMotionProfile(),
                "IntakeWaitForSlowRetract").withName("IntakeSlowRetract");
    }

    public Command smartRetractDuringShootCommand(BooleanSupplier activelyFeedingSupplier) {
        BooleanSupplier feedSupplier = Objects.requireNonNull(activelyFeedingSupplier, "activelyFeedingSupplier");
        SmartRetractSession session = new SmartRetractSession();
        return Commands.run(
                        () -> executeSmartRetractSession(session, feedSupplier),
                        this)
                .beforeStarting(() -> initializeSmartRetractSession(session))
                .finallyDo(interrupted -> endSmartRetractSession(session))
                .withName("IntakeSmartRetractDuringShoot");
    }

    public Command smartRetractDuringShootCommand() {
        return smartRetractDuringShootCommand(() -> true);
    }

    public Command extendCommand() {
        return moveToGoalCommand(
                GoalState.EXTENDED,
                standardMotionProfile(),
                "IntakeWaitForPosition").withName("IntakeExtend");
    }

    public Command toggleExtendedCommand() {
        return Commands.runOnce(
                () -> requestGoal(toggleGoal(goalState), standardMotionProfile()),
                this)
                .withName("IntakeToggleExtended");
    }

    public Command homeCommand() {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    preHomeMotionState = motionState;
                    preHomeGoalState = goalState;
                    motionState = MotionState.HOMING;
                    leftHomeSucceeded = false;
                    rightHomeSucceeded = false;
                }, this),
                Commands.parallel(homeLeftCommand(), homeRightCommand()).withName("IntakeHomeBoth"),
                Commands.either(
                        Commands.sequence(
                                Commands.waitSeconds(0.1),
                                Commands.runOnce(io::resetEncoders, this),
                                Commands.runOnce(
                                        () -> requestGoal(GoalState.RETRACTED, standardMotionProfile()),
                                        this)),
                        Commands.runOnce(
                                () -> {
                                    motionState = preHomeMotionState;
                                    goalState = preHomeGoalState;
                                    DriverStation.reportWarning(
                                            "Intake homing did not hit both current thresholds; skipping encoder reset/retract.",
                                            false);
                                },
                                this),
                        this::didHomeSucceed))
                .finallyDo(interrupted -> {
                    if (interrupted) {
                        io.stop();
                        motionState = MotionState.UNKNOWN;
                    }
                })
                .withName("IntakeHome");
    }

    private Command homeLeftCommand() {
        return homeSideCommand(
                () -> io.setLeftIntakeVoltage(homingVoltage()),
                io::stopLeftIntake,
                () -> Math.abs(inputs.leftStatorCurrentAmps) > IntakeConstants.HOMING_CURRENT_THRESHOLD_AMPS,
                (success) -> leftHomeSucceeded = success,
                () -> leftHomeSucceeded,
                "IntakeHomeLeftWaitUntil",
                "Left intake homing timed out before current threshold.",
                "IntakeHomeLeft");
    }

    private Command homeRightCommand() {
        return homeSideCommand(
                () -> io.setRightIntakeVoltage(homingVoltage()),
                io::stopRightIntake,
                () -> Math.abs(inputs.rightStatorCurrentAmps) > IntakeConstants.HOMING_CURRENT_THRESHOLD_AMPS,
                (success) -> rightHomeSucceeded = success,
                () -> rightHomeSucceeded,
                "IntakeHomeRightWaitUntil",
                "Right intake homing timed out before current threshold.",
                "IntakeHomeRight");
    }

    private Command homeSideCommand(
            Runnable homeAction,
            Runnable stopAction,
            BooleanSupplier atHomingStop,
            java.util.function.Consumer<Boolean> setSuccess,
            BooleanSupplier didSucceed,
            String waitCommandName,
            String timeoutWarning,
            String commandName) {
        return Commands.sequence(
                        Commands.runOnce(() -> setSuccess.accept(false)),
                        Commands.runOnce(homeAction),
                        Commands.sequence(
                                        Commands.waitUntil(atHomingStop).withName(waitCommandName),
                                        Commands.runOnce(() -> setSuccess.accept(true)))
                                .withTimeout(homingWaitTimeoutSec()),
                        Commands.runOnce(stopAction),
                        Commands.runOnce(
                                () -> {
                                    if (!didSucceed.getAsBoolean()) {
                                        DriverStation.reportWarning(timeoutWarning, false);
                                    }
                                }))
                .withName(commandName);
    }

    private static double homingWaitTimeoutSec() {
        return IntakeConstants.HOMING_WAIT_TIMEOUT_SEC;
    }

    private static double homingVoltage() {
        return RobotBase.isSimulation() ? -12.0 : IntakeConstants.HOMING_VOLTAGE;
    }

    private void initializeSmartRetractSession(SmartRetractSession session) {
        session.mode = cachedSmartRetractMode;
        session.startedExtended = isExtended() || isGoalExtended();
        session.active = session.startedExtended && session.mode != SmartRetractMode.DISABLED;
        session.commandedLeftTargetRot = clampSmartRetractTargetRot(getLeftPositionRotations());
        session.filteredSignalCurrentAmps = getSmartRetractSignalCurrentAmps();
        session.baselineSignalCurrentAmps = session.filteredSignalCurrentAmps;
        session.nibbleBackoffActive = false;
        session.nibbleSpikeCycles = 0;
        session.nibbleBackoffUntilSec = Double.NaN;
        session.feedLatched = false;
        session.feedTrueCycles = 0;
        session.feedFalseCycles = 0;

        if (session.active) {
            commandSmartRetractTarget(session.commandedLeftTargetRot);
        }

        Logger.recordOutput("Intake/SmartRetract/SessionActive", session.active);
        Logger.recordOutput("Intake/SmartRetract/SessionMode", session.mode.name());
    }

    private void executeSmartRetractSession(SmartRetractSession session, BooleanSupplier activelyFeedingSupplier) {
        boolean shouldSpinRoller = session.startedExtended && (isGoalExtended() || !isAtRetractedTarget());
        if (shouldSpinRoller) {
            io.setRollerRpm(IntakeConstants.SLOW_ROLLER_RPM);
        } else {
            io.stopRoller();
        }

        if (!session.active) {
            return;
        }

        double rawSignalCurrentAmps = getSmartRetractSignalCurrentAmps();
        session.filteredSignalCurrentAmps = filteredCurrent(
                session.filteredSignalCurrentAmps,
                rawSignalCurrentAmps,
                IntakeConstants.SMART_RETRACT_CURRENT_FILTER_ALPHA);
        boolean feedLatched = updateFeedLatch(session, activelyFeedingSupplier.getAsBoolean());

        if (!feedLatched) {
            logSmartRetractSessionOutputs(session, rawSignalCurrentAmps);
            return;
        }

        if (isAtRetractedTarget()) {
            commandSmartRetractTarget(targetRotations(GoalState.RETRACTED));
            logSmartRetractSessionOutputs(session, rawSignalCurrentAmps);
            return;
        }

        switch (session.mode) {
            case NIBBLE -> runNibbleSmartRetract(session);
            case CURRENT_HOLD -> runCurrentHoldSmartRetract(session);
            case DISABLED -> {
                // Session is marked active only for non-disabled modes.
            }
        }

        commandSmartRetractTarget(session.commandedLeftTargetRot);
        logSmartRetractSessionOutputs(session, rawSignalCurrentAmps);
    }

    private void runNibbleSmartRetract(SmartRetractSession session) {
        double nowSec = Timer.getFPGATimestamp();
        if (session.nibbleBackoffActive) {
            if (nowSec < session.nibbleBackoffUntilSec) {
                return;
            }
            session.nibbleBackoffActive = false;
            session.nibbleSpikeCycles = 0;
            session.baselineSignalCurrentAmps = session.filteredSignalCurrentAmps;
        }

        double thresholdAmps =
                session.baselineSignalCurrentAmps + IntakeConstants.SMART_RETRACT_NIBBLE_CURRENT_DELTA_AMPS;
        if (session.filteredSignalCurrentAmps >= thresholdAmps) {
            session.nibbleSpikeCycles++;
        } else {
            session.nibbleSpikeCycles = 0;
            session.baselineSignalCurrentAmps = filteredCurrent(
                    session.baselineSignalCurrentAmps,
                    session.filteredSignalCurrentAmps,
                    0.05);
        }

        if (session.nibbleSpikeCycles >= IntakeConstants.SMART_RETRACT_NIBBLE_DETECT_CYCLES) {
            session.commandedLeftTargetRot = clampSmartRetractTargetRot(
                    getLeftPositionRotations() + IntakeConstants.SMART_RETRACT_NIBBLE_BACKOFF_ROT);
            session.nibbleBackoffActive = true;
            session.nibbleBackoffUntilSec =
                    nowSec + IntakeConstants.SMART_RETRACT_NIBBLE_BACKOFF_DWELL_SEC;
            session.nibbleSpikeCycles = 0;
            return;
        }

        session.commandedLeftTargetRot = clampSmartRetractTargetRot(
                session.commandedLeftTargetRot - IntakeConstants.SMART_RETRACT_NIBBLE_STEP_ROT);
    }

    private void runCurrentHoldSmartRetract(SmartRetractSession session) {
        double targetCurrent = IntakeConstants.SMART_RETRACT_HOLD_TARGET_CURRENT_AMPS;
        double deadband = IntakeConstants.SMART_RETRACT_HOLD_DEADBAND_AMPS;

        if (session.filteredSignalCurrentAmps > targetCurrent + deadband) {
            session.commandedLeftTargetRot = clampSmartRetractTargetRot(
                    session.commandedLeftTargetRot + IntakeConstants.SMART_RETRACT_HOLD_BACKOFF_STEP_ROT);
            return;
        }

        if (session.filteredSignalCurrentAmps < targetCurrent - deadband) {
            session.commandedLeftTargetRot = clampSmartRetractTargetRot(
                    session.commandedLeftTargetRot - IntakeConstants.SMART_RETRACT_HOLD_FAST_STEP_ROT);
            return;
        }

        session.commandedLeftTargetRot = clampSmartRetractTargetRot(
                session.commandedLeftTargetRot - IntakeConstants.SMART_RETRACT_HOLD_SLOW_STEP_ROT);
    }

    private void endSmartRetractSession(SmartRetractSession session) {
        io.stopRoller();

        if (!session.active || DriverStation.isDisabled()) {
            Logger.recordOutput("Intake/SmartRetract/RestoreExtendedOnExit", false);
            return;
        }

        boolean restoreExtended = session.startedExtended && !isAtRetractedTarget();
        Logger.recordOutput("Intake/SmartRetract/RestoreExtendedOnExit", restoreExtended);
        if (restoreExtended) {
            requestGoal(GoalState.EXTENDED, standardMotionProfile());
        }
    }

    private void logSmartRetractSessionOutputs(SmartRetractSession session, double rawSignalCurrentAmps) {
        Logger.recordOutput("Intake/SmartRetract/SignalCurrentRawAmps", rawSignalCurrentAmps);
        Logger.recordOutput("Intake/SmartRetract/SignalCurrentFilteredAmps", session.filteredSignalCurrentAmps);
        Logger.recordOutput("Intake/SmartRetract/SignalBaselineAmps", session.baselineSignalCurrentAmps);
        Logger.recordOutput("Intake/SmartRetract/FeedLatched", session.feedLatched);
        Logger.recordOutput("Intake/SmartRetract/FeedTrueCycles", session.feedTrueCycles);
        Logger.recordOutput("Intake/SmartRetract/FeedFalseCycles", session.feedFalseCycles);
        Logger.recordOutput("Intake/SmartRetract/NibbleSpikeCycles", session.nibbleSpikeCycles);
        Logger.recordOutput("Intake/SmartRetract/NibbleBackoffActive", session.nibbleBackoffActive);
        Logger.recordOutput("Intake/SmartRetract/CommandedTargetRot", session.commandedLeftTargetRot);
    }

    private boolean updateFeedLatch(SmartRetractSession session, boolean activelyFeeding) {
        if (activelyFeeding) {
            session.feedTrueCycles++;
            session.feedFalseCycles = 0;
        } else {
            session.feedTrueCycles = 0;
            session.feedFalseCycles++;
        }

        if (!session.feedLatched
                && session.feedTrueCycles >= IntakeConstants.SMART_RETRACT_FEED_ENGAGE_CYCLES) {
            session.feedLatched = true;
        } else if (session.feedLatched
                && session.feedFalseCycles >= IntakeConstants.SMART_RETRACT_FEED_RELEASE_CYCLES) {
            session.feedLatched = false;
        }

        return session.feedLatched;
    }

    private void commandSmartRetractTarget(double leftTargetRot) {
        goalState = GoalState.RETRACTED;
        motionState = isAtRetractedTarget()
                ? MotionState.RETRACTED
                : MotionState.MOVING_TO_RETRACTED;
        MotionProfile profile = slowRetractMotionProfile();
        requestIntakePosition(
                clampSmartRetractTargetRot(leftTargetRot),
                profile.velocityRotPerSec,
                profile.accelerationRotPerSecSq,
                profile.maxVolts);
    }

    private void refreshDashboardSmartRetractMode() {
        cachedSmartRetractNibbleEnabled = SmartDashboard.getBoolean(DASHBOARD_SMART_RETRACT_ENABLE_NIBBLE_KEY, false);
        cachedSmartRetractCurrentHoldEnabled =
                SmartDashboard.getBoolean(DASHBOARD_SMART_RETRACT_ENABLE_CURRENT_HOLD_KEY, false);
        cachedSmartRetractMode = selectSmartRetractMode(
                cachedSmartRetractNibbleEnabled,
                cachedSmartRetractCurrentHoldEnabled);
        SmartDashboard.putString(DASHBOARD_SMART_RETRACT_STATUS_MODE_KEY, cachedSmartRetractMode.name());
    }

    private static SmartRetractMode selectSmartRetractMode(
            boolean nibbleEnabled,
            boolean currentHoldEnabled) {
        if (nibbleEnabled) {
            return SmartRetractMode.NIBBLE;
        }
        if (currentHoldEnabled) {
            return SmartRetractMode.CURRENT_HOLD;
        }
        return SmartRetractMode.DISABLED;
    }

    private static void initDashboardSmartRetractEntries() {
        SmartDashboard.setDefaultBoolean(DASHBOARD_SMART_RETRACT_ENABLE_NIBBLE_KEY, false);
        SmartDashboard.setDefaultBoolean(DASHBOARD_SMART_RETRACT_ENABLE_CURRENT_HOLD_KEY, false);
        SmartDashboard.putString(DASHBOARD_SMART_RETRACT_STATUS_MODE_KEY, SmartRetractMode.DISABLED.name());
    }

    public Command spinRoller() {
        return this.runEnd(
                () -> io.setRollerRpm(IntakeConstants.ROLLER_RPM),
                io::stopRoller)
                .withName("IntakeSpinRoller");
    }

    public Command backgroundCommand() {
        return Commands.runEnd(
                () -> {
                    if (isExtended()) {
                        io.setRollerRpm(IntakeConstants.SLOW_ROLLER_RPM);
                    } else {
                        io.stopRoller();
                    }
                },
                io::stopRoller,
                this).withName("IntakeBackground");
    }

    public Command stopCommand() {
        return Commands.runOnce(() -> {
            io.stop();
            motionState = MotionState.UNKNOWN;
        }, this).withName("IntakeStop");
    }

    public Command stopAndRetractCommand() {
        return Commands.sequence(
                retractCommand(),
                stopCommand())
                .withName("IntakeStopAndRetract");
    }

    public void stopAll() {
        io.stop();
        motionState = MotionState.UNKNOWN;
    }

    private Command moveToGoalCommand(
            GoalState goal,
            MotionProfile profile,
            String waitCommandName) {
        double targetRot = targetRotations(goal);
        return Commands.sequence(
                Commands.runOnce(
                        () -> requestGoal(goal, profile),
                        this),
                Commands.waitUntil(() -> isAtTargetPosition(targetRot))
                        .withTimeout(IntakeConstants.MOVE_TIMEOUT_SEC)
                        .withName(waitCommandName),
                Commands.runOnce(this::updateMotionStateFromSensors, this));
    }

    private void requestIntakePosition(
            double leftTargetRot,
            double velocityRotPerSec,
            double accelerationRotPerSecSq,
            double maxVolts) {
        io.setIntakePosition(leftTargetRot, velocityRotPerSec, accelerationRotPerSecSq, maxVolts);
    }

    private boolean isAtTargetPosition(double leftTargetRot) {
        double leftPositionRot = Units.radiansToRotations(inputs.leftPositionRad);
        double rightPositionRot = Units.radiansToRotations(inputs.rightPositionRad);
        double rightTargetRot = IntakeConstants.RIGHT_OPPOSES_LEFT ? -leftTargetRot : leftTargetRot;
        return Math.abs(leftPositionRot - leftTargetRot) <= IntakeConstants.POSITION_TOLERANCE_ROT
                && Math.abs(rightPositionRot - rightTargetRot) <= IntakeConstants.POSITION_TOLERANCE_ROT;
    }

    private boolean isAtRetractedTarget() {
        return isAtTargetPosition(targetRotations(GoalState.RETRACTED));
    }

    private double getLeftPositionRotations() {
        return Units.radiansToRotations(inputs.leftPositionRad);
    }

    private static double filteredCurrent(double previous, double current, double alpha) {
        double clampedAlpha = MathUtil.clamp(alpha, 0.0, 1.0);
        return previous + clampedAlpha * (current - previous);
    }

    private static double clampSmartRetractTargetRot(double targetRot) {
        return MathUtil.clamp(
                targetRot,
                IntakeConstants.RETRACTED_POSITION_ROT,
                IntakeConstants.EXTENDED_POSITION_ROT);
    }

    private double getSmartRetractSignalCurrentAmps() {
        return Math.abs(inputs.rollerStatorCurrentAmps);
    }

    private boolean didHomeSucceed() {
        return leftHomeSucceeded && rightHomeSucceeded;
    }

    private void requestGoal(GoalState goal, MotionProfile profile) {
        goalState = goal;
        double targetRot = targetRotations(goal);
        motionState = isAtTargetPosition(targetRot)
                ? atGoalState(goal)
                : movingToGoalState(goal);
        requestIntakePosition(
                targetRot,
                profile.velocityRotPerSec,
                profile.accelerationRotPerSecSq,
                profile.maxVolts);
    }

    private boolean isGoalExtended() {
        return goalState == GoalState.EXTENDED;
    }

    private void updateMotionStateFromSensors() {
        if (motionState == MotionState.HOMING || motionState == MotionState.UNKNOWN) {
            return;
        }
        if (goalState == GoalState.EXTENDED && motionState == MotionState.EXTENDED) {
            return;
        }
        if (goalState == GoalState.RETRACTED && motionState == MotionState.RETRACTED) {
            return;
        }
        if (isAtTargetPosition(targetRotations(goalState))) {
            motionState = atGoalState(goalState);
            return;
        }
        motionState = movingToGoalState(goalState);
    }

    private static GoalState toggleGoal(GoalState goal) {
        return goal == GoalState.EXTENDED ? GoalState.RETRACTED : GoalState.EXTENDED;
    }

    private static MotionState movingToGoalState(GoalState goal) {
        return goal == GoalState.EXTENDED ? MotionState.MOVING_TO_EXTENDED : MotionState.MOVING_TO_RETRACTED;
    }

    private static MotionState atGoalState(GoalState goal) {
        return goal == GoalState.EXTENDED ? MotionState.EXTENDED : MotionState.RETRACTED;
    }

    private static double targetRotations(GoalState goal) {
        return goal == GoalState.EXTENDED
                ? IntakeConstants.EXTENDED_POSITION_ROT
                : IntakeConstants.RETRACTED_POSITION_ROT;
    }

    private static MotionProfile standardMotionProfile() {
        return new MotionProfile(
                IntakeConstants.INTAKE_VELOCITY,
                IntakeConstants.INTAKE_ACCELERATION,
                IntakeConstants.INTAKE_MAX_VOLTS);
    }

    private static MotionProfile slowRetractMotionProfile() {
        return new MotionProfile(
                IntakeConstants.SLOW_INTAKE_VELOCITY,
                IntakeConstants.SLOW_INTAKE_ACCELERATION,
                IntakeConstants.SLOW_RETRACT_MAX_VOLTS);
    }
}
