package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.coordination.intake.SmartRetractController;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;
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

    private record PositionDirective(double targetRot, GoalState goalState, MotionProfile profile) {}

    private record OscillationProfile(String logStem, double baselineRot, double peakRot, double switchIntervalSec) {}

    private static final class PositionOverlayState {
        private OscillationProfile profile;
        private GoalState goalState = GoalState.RETRACTED;
        private MotionProfile motionProfile;
        private double startSec = Double.NaN;

        private boolean active() {
            return profile != null;
        }

        private void activate(OscillationProfile nextProfile, GoalState nextGoalState, MotionProfile nextMotionProfile) {
            boolean shouldResetPhase = profile == null || !profile.logStem().equals(nextProfile.logStem());
            profile = nextProfile;
            goalState = nextGoalState;
            motionProfile = nextMotionProfile;
            if (shouldResetPhase || Double.isNaN(startSec)) {
                startSec = Timer.getFPGATimestamp();
            }
        }

        private void clear() {
            profile = null;
            motionProfile = null;
            startSec = Double.NaN;
        }

        private PositionDirective resolve() {
            double elapsedSec = Math.max(0.0, Timer.getFPGATimestamp() - startSec);
            boolean highPhase = ((int) Math.floor(elapsedSec / profile.switchIntervalSec())) % 2 == 1;
            double targetRot = highPhase ? profile.peakRot() : profile.baselineRot();
            return new PositionDirective(targetRot, goalState, motionProfile);
        }
    }

    private static final String DASHBOARD_SMART_RETRACT_ENABLE_NIBBLE_KEY = "Intake/SmartRetract/EnableNibble";
    private static final String DASHBOARD_SMART_RETRACT_ENABLE_HALF_RETRACT_RETURN_KEY =
            "Intake/SmartRetract/EnableHalfRetractReturn";
    private static final String DASHBOARD_SMART_RETRACT_STATUS_MODE_KEY = "Intake/SmartRetract/StatusMode";

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final SmartRetractController smartRetractController = new SmartRetractController();
    private final PositionOverlayState positionOverlay = new PositionOverlayState();
    private GoalState goalState = GoalState.RETRACTED;
    private MotionState motionState = MotionState.RETRACTED;
    private boolean leftHomeSucceeded = false;
    private boolean rightHomeSucceeded = false;
    private MotionState preHomeMotionState = MotionState.RETRACTED;
    private GoalState preHomeGoalState = GoalState.RETRACTED;
    private boolean preHomePositionControlEnabled = true;
    private boolean cachedSmartRetractNibbleEnabled = false;
    private boolean cachedSmartRetractHalfRetractReturnEnabled = false;
    private SmartRetractController.Mode cachedSmartRetractMode = SmartRetractController.Mode.DISABLED;
    private boolean positionControlEnabled = true;
    private PositionDirective basePositionDirective = goalDirective(GoalState.RETRACTED, standardMotionProfile());
    private double commandedLeftTargetRot = IntakeConstants.RETRACTED_POSITION_ROT;

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
            updateMotionStateFromSensors();
        } else if (motionState != MotionState.HOMING && positionControlEnabled) {
            applyResolvedPositionDirective();
        } else {
            updateMotionStateFromSensors();
        }

        Logger.recordOutput("Intake/Extended", isExtended());
        Logger.recordOutput("Intake/RequestedExtended", isGoalExtended());
        Logger.recordOutput("Intake/GoalState", goalState.name());
        Logger.recordOutput("Intake/MotionState", motionState.name());
        Logger.recordOutput("Intake/PositionControlEnabled", positionControlEnabled);
        Logger.recordOutput("Intake/PositionOverlayActive", positionOverlay.active());
        Logger.recordOutput("Intake/SmartRetract/EnableNibble", cachedSmartRetractNibbleEnabled);
        Logger.recordOutput(
                "Intake/SmartRetract/EnableHalfRetractReturn",
                cachedSmartRetractHalfRetractReturnEnabled);
        Logger.recordOutput(
                "Intake/SmartRetract/BothModesEnabled",
                cachedSmartRetractNibbleEnabled && cachedSmartRetractHalfRetractReturnEnabled);
        Logger.recordOutput("Intake/SmartRetract/SelectedMode", cachedSmartRetractMode.name());
    }

    public void setExtended(boolean isExtended) {
        requestGoal(isExtended ? GoalState.EXTENDED : GoalState.RETRACTED, standardMotionProfile());
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

    public Command smartRetractDuringShootCommand(
            BooleanSupplier activelyFeedingSupplier,
            BooleanSupplier shotPulseDetectedSupplier) {
        BooleanSupplier feedSupplier = Objects.requireNonNull(activelyFeedingSupplier, "activelyFeedingSupplier");
        BooleanSupplier shotSupplier = Objects.requireNonNull(shotPulseDetectedSupplier, "shotPulseDetectedSupplier");
        SmartRetractController.Session session = new SmartRetractController.Session();
        return Commands.run(
                        () -> executeSmartRetractSession(session, feedSupplier, shotSupplier),
                        this)
                .beforeStarting(() -> initializeSmartRetractSession(session))
                .finallyDo(interrupted -> endSmartRetractSession(session))
                .withName("IntakeSmartRetractDuringShoot");
    }

    public Command smartRetractDuringShootCommand(
            BooleanSupplier activelyFeedingSupplier,
            IntSupplier shotCountSupplier) {
        BooleanSupplier feedSupplier = Objects.requireNonNull(activelyFeedingSupplier, "activelyFeedingSupplier");
        IntSupplier countSupplier = Objects.requireNonNull(shotCountSupplier, "shotCountSupplier");
        SmartRetractController.Session session = new SmartRetractController.Session();
        int[] lastObservedShotCount = new int[] {0};
        return Commands.run(
                        () -> executeSmartRetractSession(session, feedSupplier, countSupplier, lastObservedShotCount),
                        this)
                .beforeStarting(() -> {
                    lastObservedShotCount[0] = countSupplier.getAsInt();
                    initializeSmartRetractSession(session);
                })
                .finallyDo(interrupted -> endSmartRetractSession(session))
                .withName("IntakeSmartRetractDuringShoot");
    }

    public Command smartRetractDuringShootCommand(BooleanSupplier activelyFeedingSupplier) {
        return smartRetractDuringShootCommand(activelyFeedingSupplier, () -> true);
    }

    public Command smartRetractDuringShootCommand() {
        return smartRetractDuringShootCommand(() -> true, () -> true);
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
                            preHomePositionControlEnabled = positionControlEnabled;
                            motionState = MotionState.HOMING;
                            positionControlEnabled = false;
                            clearPositionOverlay();
                            leftHomeSucceeded = false;
                            rightHomeSucceeded = false;
                        }, this),
                        Commands.parallel(homeLeftCommand(), homeRightCommand()).withName("IntakeHomeBoth"),
                        Commands.either(
                                Commands.sequence(
                                Commands.waitSeconds(0.1),
                                Commands.runOnce(io::resetEncoders, this),
                                Commands.runOnce(
                                                () -> {
                                                    motionState = MotionState.UNKNOWN;
                                                    requestGoal(GoalState.RETRACTED, standardMotionProfile());
                                                },
                                                this)),
                                Commands.runOnce(
                                        () -> {
                                            motionState = preHomeMotionState;
                                            goalState = preHomeGoalState;
                                            positionControlEnabled = preHomePositionControlEnabled;
                                            updateMotionStateFromSensors();
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

    private void initializeSmartRetractSession(SmartRetractController.Session session) {
        clearPositionOverlay();
        smartRetractController.initialize(
                session,
                cachedSmartRetractMode,
                isExtended() || isGoalExtended(),
                getLeftPositionRotations(),
                getSmartRetractSignalCurrentAmps());

        if (session.active()) {
            commandSmartRetractTarget(session.commandedLeftTargetRot());
        }

        Logger.recordOutput("Intake/SmartRetract/SessionActive", session.active());
        Logger.recordOutput("Intake/SmartRetract/SessionMode", session.mode().name());
    }

    private void executeSmartRetractSession(
            SmartRetractController.Session session,
            BooleanSupplier activelyFeedingSupplier,
            BooleanSupplier shotPulseDetectedSupplier) {
        boolean activelyFeeding = activelyFeedingSupplier.getAsBoolean();
        SmartRetractController.Update update = smartRetractController.update(
                session,
                activelyFeeding,
                shotPulseDetectedSupplier.getAsBoolean(),
                getLeftPositionRotations(),
                getSmartRetractSignalCurrentAmps(),
                isGoalExtended(),
                isAtRetractedTarget());

        if (update.spinRoller()) {
            io.setRollerRpm(IntakeConstants.smartRetractRollerRpm());
        } else {
            io.stopRoller();
        }

        if (update.commandRetractTarget()) {
            commandSmartRetractTarget(update.commandedLeftTargetRot());
        }

        if (shouldRunSmartRetractWiggle(session, activelyFeeding)) {
            activatePositionOverlay(smartRetractWiggleProfile(), GoalState.RETRACTED, standardMotionProfile());
        } else {
            clearPositionOverlay();
        }

        logSmartRetractSessionOutputs(session, update.rawSignalCurrentAmps());
    }

    private void executeSmartRetractSession(
            SmartRetractController.Session session,
            BooleanSupplier activelyFeedingSupplier,
            IntSupplier shotCountSupplier,
            int[] lastObservedShotCount) {
        int currentShotCount = shotCountSupplier.getAsInt();
        boolean shotPulseDetectedThisCycle = currentShotCount > lastObservedShotCount[0];
        lastObservedShotCount[0] = currentShotCount;
        Logger.recordOutput("Intake/SmartRetract/ObservedShotCount", currentShotCount);
        executeSmartRetractSession(session, activelyFeedingSupplier, () -> shotPulseDetectedThisCycle);
    }

    private void endSmartRetractSession(SmartRetractController.Session session) {
        io.stopRoller();
        clearPositionOverlay();

        boolean atSmartRetractTarget = getLeftPositionRotations()
                <= IntakeConstants.smartRetractRetractedPositionRot() + IntakeConstants.POSITION_TOLERANCE_ROT;
        boolean restoreExtended = smartRetractController.shouldRestoreExtendedOnExit(
                session,
                DriverStation.isDisabled(),
                isAtRetractedTarget() || atSmartRetractTarget);
        Logger.recordOutput("Intake/SmartRetract/RestoreExtendedOnExit", restoreExtended);
        if (restoreExtended) {
            requestGoal(GoalState.EXTENDED, standardMotionProfile());
        }
    }

    private void logSmartRetractSessionOutputs(
            SmartRetractController.Session session,
            double rawSignalCurrentAmps) {
        Logger.recordOutput("Intake/SmartRetract/SignalCurrentRawAmps", rawSignalCurrentAmps);
        Logger.recordOutput("Intake/SmartRetract/SignalCurrentFilteredAmps", session.filteredSignalCurrentAmps());
        Logger.recordOutput("Intake/SmartRetract/SignalThresholdAmps", session.nibbleCurrentThresholdAmps());
        Logger.recordOutput("Intake/SmartRetract/SignalBaselineAmps", session.nibbleCurrentThresholdAmps());
        Logger.recordOutput("Intake/SmartRetract/FeedLatched", session.feedLatched());
        Logger.recordOutput("Intake/SmartRetract/FeedTrueCycles", session.feedTrueCycles());
        Logger.recordOutput("Intake/SmartRetract/FeedFalseCycles", session.feedFalseCycles());
        Logger.recordOutput("Intake/SmartRetract/NibbleSpikeCycles", session.nibbleSpikeCycles());
        Logger.recordOutput("Intake/SmartRetract/NibbleBackoffActive", session.nibbleBackoffActive());
        Logger.recordOutput("Intake/SmartRetract/FullRetractReached", session.fullRetractReached());
        Logger.recordOutput("Intake/SmartRetract/RollerRpm", IntakeConstants.smartRetractRollerRpm());
        Logger.recordOutput("Intake/SmartRetract/SawShotPulse", session.sawShotPulse());
        Logger.recordOutput("Intake/SmartRetract/LastShotOrFeedTimestampSec", session.lastShotOrFeedTimestampSec());
        Logger.recordOutput("Intake/SmartRetract/JamRecoveryActive", session.jamRecoveryActive());
        Logger.recordOutput("Intake/SmartRetract/JamRecoveryCount", session.jamRecoveryCount());
        Logger.recordOutput("Intake/SmartRetract/CommandedTargetRot", session.commandedLeftTargetRot());
    }

    private void commandSmartRetractTarget(double leftTargetRot) {
        double clampedTargetRot = clampSmartRetractTargetRot(leftTargetRot);
        GoalState semanticGoal = clampedTargetRot >= IntakeConstants.EXTENDED_POSITION_ROT - IntakeConstants.POSITION_TOLERANCE_ROT
                ? GoalState.EXTENDED
                : GoalState.RETRACTED;
        setBasePositionDirective(new PositionDirective(clampedTargetRot, semanticGoal, standardMotionProfile()));
    }

    private void refreshDashboardSmartRetractMode() {
        cachedSmartRetractNibbleEnabled = SmartDashboard.getBoolean(DASHBOARD_SMART_RETRACT_ENABLE_NIBBLE_KEY, true);
        cachedSmartRetractHalfRetractReturnEnabled =
                SmartDashboard.getBoolean(DASHBOARD_SMART_RETRACT_ENABLE_HALF_RETRACT_RETURN_KEY, false);
        cachedSmartRetractMode = selectSmartRetractMode(
                cachedSmartRetractNibbleEnabled,
                cachedSmartRetractHalfRetractReturnEnabled);
        SmartDashboard.putString(DASHBOARD_SMART_RETRACT_STATUS_MODE_KEY, cachedSmartRetractMode.name());
    }

    private static SmartRetractController.Mode selectSmartRetractMode(
            boolean nibbleEnabled,
            boolean halfRetractReturnEnabled) {
        if (nibbleEnabled) {
            return SmartRetractController.Mode.NIBBLE;
        }
        if (halfRetractReturnEnabled) {
            return SmartRetractController.Mode.HALF_RETRACT_RETURN;
        }
        return SmartRetractController.Mode.DISABLED;
    }

    private static void initDashboardSmartRetractEntries() {
        SmartDashboard.setDefaultBoolean(DASHBOARD_SMART_RETRACT_ENABLE_NIBBLE_KEY, true);
        SmartDashboard.setDefaultBoolean(DASHBOARD_SMART_RETRACT_ENABLE_HALF_RETRACT_RETURN_KEY, false);
        SmartDashboard.putString(
                DASHBOARD_SMART_RETRACT_STATUS_MODE_KEY,
                SmartRetractController.Mode.DISABLED.name());
    }

    public Command spinRoller() {
        return this.runEnd(
                        () -> io.setRollerRpm(IntakeConstants.ROLLER_RPM),
                        io::stopRoller)
                .withName("IntakeSpinRoller");
    }

    public Command wiggleSpinRollerCommand() {
        return createRollerAgitationCommand(driverTriggerAgitationProfile(), "IntakeSpinRoller");
    }

    public Command driverTriggerSpinRollerCommand() {
        return wiggleSpinRollerCommand();
    }

    public Command driverAgitationCommand() {
        return createRollerAgitationCommand(driverAgitationProfile(), "IntakeAgitate");
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
            clearPositionOverlay();
            positionControlEnabled = false;
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
        clearPositionOverlay();
        positionControlEnabled = false;
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
        commandedLeftTargetRot = leftTargetRot;
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

    private static double clampSmartRetractTargetRot(double targetRot) {
        return MathUtil.clamp(
                targetRot,
                IntakeConstants.smartRetractRetractedPositionRot(),
                IntakeConstants.EXTENDED_POSITION_ROT);
    }

    private double getSmartRetractSignalCurrentAmps() {
        return Math.abs(inputs.rollerStatorCurrentAmps);
    }

    private boolean didHomeSucceed() {
        return leftHomeSucceeded && rightHomeSucceeded;
    }

    private void requestGoal(GoalState goal, MotionProfile profile) {
        clearPositionOverlay();
        setBasePositionDirective(goalDirective(goal, profile));
    }

    private boolean isGoalExtended() {
        return goalState == GoalState.EXTENDED;
    }

    private void updateMotionStateFromSensors() {
        if (motionState == MotionState.HOMING) {
            return;
        }
        if (isAtTargetPosition(commandedLeftTargetRot)) {
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

    private Command createRollerAgitationCommand(OscillationProfile profile, String commandName) {
        return Commands.run(
                        () -> executeRollerAgitation(profile),
                        this)
                .finallyDo(interrupted -> endRollerAgitation(profile))
                .withName(commandName);
    }

    private static OscillationProfile driverTriggerAgitationProfile() {
        return new OscillationProfile(
                "Intake/DriverTriggerWiggle",
                IntakeConstants.DRIVER_TRIGGER_WIGGLE_BASELINE_ROT,
                IntakeConstants.DRIVER_TRIGGER_WIGGLE_PEAK_ROT,
                IntakeConstants.DRIVER_TRIGGER_WIGGLE_SWITCH_INTERVAL_SEC);
    }

    private static OscillationProfile driverAgitationProfile() {
        return new OscillationProfile(
                "Intake/DriverAgitation",
                IntakeConstants.DRIVER_AGITATION_BASELINE_ROT,
                IntakeConstants.DRIVER_AGITATION_PEAK_ROT,
                IntakeConstants.DRIVER_AGITATION_SWITCH_INTERVAL_SEC);
    }

    private static OscillationProfile smartRetractWiggleProfile() {
        double baselineRot = IntakeConstants.smartRetractRetractedPositionRot();
        double peakRot = MathUtil.clamp(
                baselineRot + IntakeConstants.smartRetractWiggleOutRot(),
                baselineRot,
                IntakeConstants.EXTENDED_POSITION_ROT);
        return new OscillationProfile(
                "Intake/SmartRetract/Wiggle",
                baselineRot,
                peakRot,
                IntakeConstants.smartRetractWiggleSwitchIntervalSec());
    }

    private void executeRollerAgitation(OscillationProfile profile) {
        io.setRollerRpm(IntakeConstants.ROLLER_RPM);
        if (!isGoalExtended() && !isExtended()) {
            clearPositionOverlay();
            recordOscillation(profile, false, commandedLeftTargetRot);
            return;
        }

        activatePositionOverlay(profile, GoalState.EXTENDED, standardMotionProfile());
        PositionDirective directive = positionOverlay.resolve();
        recordOscillation(profile, true, directive.targetRot());
    }

    private void endRollerAgitation(OscillationProfile profile) {
        io.stopRoller();
        clearPositionOverlay();
        recordOscillation(profile, false, profile.baselineRot());
    }

    private void activatePositionOverlay(
            OscillationProfile profile,
            GoalState semanticGoal,
            MotionProfile motionProfile) {
        positionOverlay.activate(profile, semanticGoal, motionProfile);
        applyResolvedPositionDirective();
    }

    private void clearPositionOverlay() {
        if (positionOverlay.active()) {
            recordOscillation(positionOverlay.profile, false, positionOverlay.profile.baselineRot());
            positionOverlay.clear();
            if (positionControlEnabled && motionState != MotionState.HOMING && !DriverStation.isDisabled()) {
                applyResolvedPositionDirective();
            }
        }
    }

    private void setBasePositionDirective(PositionDirective directive) {
        basePositionDirective = directive;
        positionControlEnabled = true;
        applyResolvedPositionDirective();
    }

    private PositionDirective resolvePositionDirective() {
        if (positionOverlay.active()) {
            PositionDirective overlayDirective = positionOverlay.resolve();
            recordOscillation(positionOverlay.profile, true, overlayDirective.targetRot());
            return overlayDirective;
        }
        return basePositionDirective;
    }

    private void applyResolvedPositionDirective() {
        if (!positionControlEnabled || motionState == MotionState.HOMING || DriverStation.isDisabled()) {
            updateMotionStateFromSensors();
            return;
        }

        PositionDirective resolved = resolvePositionDirective();
        goalState = resolved.goalState();
        motionState = isAtTargetPosition(resolved.targetRot())
                ? atGoalState(goalState)
                : movingToGoalState(goalState);
        requestIntakePosition(
                resolved.targetRot(),
                resolved.profile().velocityRotPerSec,
                resolved.profile().accelerationRotPerSecSq,
                resolved.profile().maxVolts);
    }

    private static PositionDirective goalDirective(GoalState goal, MotionProfile profile) {
        return new PositionDirective(targetRotations(goal), goal, profile);
    }

    private static void recordOscillation(OscillationProfile profile, boolean active, double targetRot) {
        Logger.recordOutput(profile.logStem() + "Active", active);
        Logger.recordOutput(profile.logStem() + "TargetRot", targetRot);
    }

    private static boolean shouldRunSmartRetractWiggle(
            SmartRetractController.Session session,
            boolean activelyFeeding) {
        return session.active()
                && session.mode() == SmartRetractController.Mode.NIBBLE
                && session.fullRetractReached()
                && session.feedLatched()
                && activelyFeeding;
    }
}
