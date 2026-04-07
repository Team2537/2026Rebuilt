package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import frc.robot.util.LoggedTunableNumber;

public final class IntakeConstants {
    private IntakeConstants() {}

    public static final int ROLLER_MOTOR_ID = 25;
    public static final int LEFT_MOTOR_ID = 23;
    public static final int RIGHT_MOTOR_ID = 24;

    public static final double ROLLER_RPM = 3500.0;
    public static final double SLOW_ROLLER_RPM = 1000.0;
    public static final double ROLLER_MAX_RPM = 6200.0;
    public static final double STATUS_UPDATE_HZ = 50.0;

    public static final double ROLLER_SENSOR_TO_MECHANISM_RATIO = 1.92; // 23/12
    public static final double ROLLER_KP = 10.0;
    public static final double ROLLER_KI = 0.0;
    public static final double ROLLER_KD = 0.0;
    public static final double ROLLER_KS = 0.0;
    public static final double ROLLER_KV = 0.12;

    public static final double INTAKE_KP = 4.0;
    public static final double INTAKE_KI = 0.5;
    public static final double INTAKE_KD = 0.0;
    public static final double INTAKE_KS = 0.0;
    public static final double INTAKE_KV = 0.0;

    public static final boolean LEFT_INTAKE_INVERTED = true;
    public static final boolean RIGHT_OPPOSES_LEFT = true;
    public static final boolean ROLLER_INVERTED = false;

    public static final double ROLLER_STATOR_CURRENT_LIMIT_AMPS = 120.0;
    public static final double ROLLER_SUPPLY_CURRENT_LIMIT_AMPS = 120.0;
    public static final double INTAKE_STATOR_CURRENT_LIMIT_AMPS = 70.0;
    public static final double INTAKE_SUPPLY_CURRENT_LIMIT_AMPS = 70.0;

    public static final double INTAKE_STATOR_CURRENT_LIMIT_AMPS_LOW = 15.0;
    public static final double INTAKE_SUPPLY_CURRENT_LIMIT_AMPS_LOW = 15.0;

    public static final double INTAKE_VELOCITY = 100.0;
    public static final double INTAKE_ACCELERATION = 400.0;

    public static final double SLOW_INTAKE_VELOCITY = 20.0;
    public static final double SLOW_INTAKE_ACCELERATION = 50.0;

    public static final double TRAVEL_IN_PER_MOTOR_REV = 0.5233;
    public static final double MOTOR_REV_PER_IN = 1.0 / TRAVEL_IN_PER_MOTOR_REV;
    public static final double FULL_TRAVEL_IN = 10.7;
    public static final double RETRACTED_POSITION_ROT = 6.75;
    public static final double EXTENDED_POSITION_ROT = FULL_TRAVEL_IN * MOTOR_REV_PER_IN;
    public static final double DRIVER_TRIGGER_WIGGLE_BASELINE_IN = FULL_TRAVEL_IN;
    public static final double DRIVER_TRIGGER_WIGGLE_PEAK_IN = 11.8;
    public static final double DRIVER_TRIGGER_WIGGLE_BASELINE_ROT =
            DRIVER_TRIGGER_WIGGLE_BASELINE_IN * MOTOR_REV_PER_IN;
    public static final double DRIVER_TRIGGER_WIGGLE_PEAK_ROT =
            DRIVER_TRIGGER_WIGGLE_PEAK_IN * MOTOR_REV_PER_IN;
    public static final double DRIVER_TRIGGER_WIGGLE_SWITCH_INTERVAL_SEC = 0.2;

    public static final double DRIVER_AGITATION_BASELINE_IN = FULL_TRAVEL_IN;
    public static final double DRIVER_AGITATION_PEAK_IN = 11.8;
    public static final double DRIVER_AGITATION_BASELINE_ROT =
            DRIVER_AGITATION_BASELINE_IN * MOTOR_REV_PER_IN;
    public static final double DRIVER_AGITATION_PEAK_ROT =
            DRIVER_AGITATION_PEAK_IN * MOTOR_REV_PER_IN;
    public static final double DRIVER_AGITATION_SWITCH_INTERVAL_SEC = 0.15;
    public static final double POSITION_TOLERANCE_ROT = 0.25;
    public static final double MOVE_TIMEOUT_SEC = 1.5;
    public static final double INTAKE_MAX_VOLTS = 12.0;
    public static final double SLOW_RETRACT_MAX_VOLTS = 4.0;

    public static final double HOMING_CURRENT_THRESHOLD_AMPS = 20.0;
    public static final double HOMING_VOLTAGE = -2.0;
    public static final double HOMING_WAIT_TIMEOUT_SEC = 4.0;

    // Smart-retract will only drive intake as far as this value (unless regular retract is used).
    public static final double SMART_RETRACT_RETRACTED_POSITION_ROT = 2.0;
    public static final double SMART_RETRACT_CURRENT_FILTER_ALPHA = 0.20;
    public static final int SMART_RETRACT_FEED_ENGAGE_CYCLES = 2;
    public static final double SMART_RETRACT_FEED_START_DELAY_SEC = 0.2;

    public static final double SMART_RETRACT_STEP_ROT = 0.6;
    public static final double SMART_RETRACT_ROLLER_RPM = 0.0;
    public static final double SMART_RETRACT_JAM_BACKOFF_CURRENT_THRESHOLD_AMPS = 3.0;
    public static final int SMART_RETRACT_JAM_BACKOFF_DETECT_CYCLES = 10;
    public static final double SMART_RETRACT_JAM_FIRST_SHOT_TIMEOUT_SEC = 0.7;
    public static final double SMART_RETRACT_JAM_INTER_SHOT_TIMEOUT_SEC = 0.4;
    public static final double SMART_RETRACT_JAM_RECOVERY_EXTEND_POSITION_ROT = EXTENDED_POSITION_ROT;
    public static final double SMART_RETRACT_INNER_STALL_RECOVERY_EXTEND_POSITION_ROT = 15.0;
    public static final double SMART_RETRACT_FULL_RETRACT_GRACE_SEC = 0.50;
    public static final double SMART_RETRACT_TAIL_DRAIN_GRACE_SEC = 2.0;
    public static final double SMART_RETRACT_WIGGLE_OUT_ROT = 8.0;
    public static final double SMART_RETRACT_WIGGLE_SWITCH_INTERVAL_SEC = DRIVER_TRIGGER_WIGGLE_SWITCH_INTERVAL_SEC;

    private static final LoggedTunableNumber smartRetractRetractedPositionRot =
            new LoggedTunableNumber(
                    "Intake/SmartRetract/RetractedPositionRot",
                    SMART_RETRACT_RETRACTED_POSITION_ROT);
    private static final LoggedTunableNumber smartRetractCurrentFilterAlpha =
            new LoggedTunableNumber(
                    "Intake/SmartRetract/CurrentFilterAlpha",
                    SMART_RETRACT_CURRENT_FILTER_ALPHA);
    private static final LoggedTunableNumber smartRetractFeedEngageCycles =
            new LoggedTunableNumber(
                    "Intake/SmartRetract/FeedEngageCycles",
                    SMART_RETRACT_FEED_ENGAGE_CYCLES);
    private static final LoggedTunableNumber smartRetractFeedStartDelaySec =
            new LoggedTunableNumber(
                    "Intake/SmartRetract/FeedStartDelaySec",
                    SMART_RETRACT_FEED_START_DELAY_SEC);

    private static final LoggedTunableNumber smartRetractStepRot =
            new LoggedTunableNumber(
                    "Intake/SmartRetract/StepRot",
                    SMART_RETRACT_STEP_ROT);
    private static final LoggedTunableNumber smartRetractRollerRpm =
            new LoggedTunableNumber(
                    "Intake/SmartRetract/RollerRpm",
                    SMART_RETRACT_ROLLER_RPM);
    private static final LoggedTunableNumber smartRetractJamBackoffCurrentThresholdAmps =
            new LoggedTunableNumber(
                    "Intake/SmartRetract/JamBackoffCurrentThresholdAmps",
                    SMART_RETRACT_JAM_BACKOFF_CURRENT_THRESHOLD_AMPS);
    private static final LoggedTunableNumber smartRetractJamBackoffDetectCycles =
            new LoggedTunableNumber(
                    "Intake/SmartRetract/JamBackoffDetectCycles",
                    SMART_RETRACT_JAM_BACKOFF_DETECT_CYCLES);
    private static final LoggedTunableNumber smartRetractJamFirstShotTimeoutSec =
            new LoggedTunableNumber(
                    "Intake/SmartRetract/JamFirstShotTimeoutSec",
                    SMART_RETRACT_JAM_FIRST_SHOT_TIMEOUT_SEC);
    private static final LoggedTunableNumber smartRetractJamInterShotTimeoutSec =
            new LoggedTunableNumber(
                    "Intake/SmartRetract/JamInterShotTimeoutSec",
                    SMART_RETRACT_JAM_INTER_SHOT_TIMEOUT_SEC);
    private static final LoggedTunableNumber smartRetractJamRecoveryExtendPositionRot =
            new LoggedTunableNumber(
                    "Intake/SmartRetract/JamRecoveryExtendPositionRot",
                    SMART_RETRACT_JAM_RECOVERY_EXTEND_POSITION_ROT);
    private static final LoggedTunableNumber smartRetractInnerStallRecoveryExtendPositionRot =
            new LoggedTunableNumber(
                    "Intake/SmartRetract/InnerStallRecoveryExtendPositionRot",
                    SMART_RETRACT_INNER_STALL_RECOVERY_EXTEND_POSITION_ROT);
    private static final LoggedTunableNumber smartRetractFullRetractGraceSec =
            new LoggedTunableNumber(
                    "Intake/SmartRetract/FullRetractGraceSec",
                    SMART_RETRACT_FULL_RETRACT_GRACE_SEC);
    private static final LoggedTunableNumber smartRetractTailDrainGraceSec =
            new LoggedTunableNumber(
                    "Intake/SmartRetract/TailDrainGraceSec",
                    SMART_RETRACT_TAIL_DRAIN_GRACE_SEC);
    private static final LoggedTunableNumber smartRetractWiggleOutRot =
            new LoggedTunableNumber(
                    "Intake/SmartRetract/WiggleOutRot",
                    SMART_RETRACT_WIGGLE_OUT_ROT);
    private static final LoggedTunableNumber smartRetractWiggleSwitchIntervalSec =
            new LoggedTunableNumber(
                    "Intake/SmartRetract/WiggleSwitchIntervalSec",
                    SMART_RETRACT_WIGGLE_SWITCH_INTERVAL_SEC);

    public static double smartRetractRetractedPositionRot() {
        return MathUtil.clamp(smartRetractRetractedPositionRot.get(), 0.0, EXTENDED_POSITION_ROT);
    }

    public static double smartRetractCurrentFilterAlpha() {
        return MathUtil.clamp(smartRetractCurrentFilterAlpha.get(), 0.0, 1.0);
    }

    public static int smartRetractFeedEngageCycles() {
        return Math.max(1, (int) Math.round(smartRetractFeedEngageCycles.get()));
    }

    public static double smartRetractFeedStartDelaySec() {
        return Math.max(0.0, smartRetractFeedStartDelaySec.get());
    }

    public static double smartRetractStepRot() {
        return Math.max(0.0, smartRetractStepRot.get());
    }

    public static double smartRetractRollerRpm() {
        return MathUtil.clamp(smartRetractRollerRpm.get(), -ROLLER_MAX_RPM, ROLLER_MAX_RPM);
    }

    public static double smartRetractJamBackoffCurrentThresholdAmps() {
        return Math.max(0.0, smartRetractJamBackoffCurrentThresholdAmps.get());
    }

    public static int smartRetractJamBackoffDetectCycles() {
        return Math.max(1, (int) Math.round(smartRetractJamBackoffDetectCycles.get()));
    }

    public static double smartRetractJamFirstShotTimeoutSec() {
        return Math.max(0.0, smartRetractJamFirstShotTimeoutSec.get());
    }

    public static double smartRetractJamInterShotTimeoutSec() {
        return Math.max(0.0, smartRetractJamInterShotTimeoutSec.get());
    }

    public static double smartRetractJamRecoveryExtendPositionRot() {
        return MathUtil.clamp(
                smartRetractJamRecoveryExtendPositionRot.get(),
                smartRetractRetractedPositionRot(),
                EXTENDED_POSITION_ROT);
    }

    public static double smartRetractInnerStallRecoveryExtendPositionRot() {
        return MathUtil.clamp(
                smartRetractInnerStallRecoveryExtendPositionRot.get(),
                smartRetractRetractedPositionRot(),
                EXTENDED_POSITION_ROT);
    }

    public static double smartRetractFullRetractGraceSec() {
        return Math.max(0.0, smartRetractFullRetractGraceSec.get());
    }

    public static double smartRetractTailDrainGraceSec() {
        return Math.max(0.0, smartRetractTailDrainGraceSec.get());
    }

    public static double smartRetractWiggleOutRot() {
        return Math.max(0.0, smartRetractWiggleOutRot.get());
    }

    public static double smartRetractWiggleSwitchIntervalSec() {
        return Math.max(1.0 / STATUS_UPDATE_HZ, smartRetractWiggleSwitchIntervalSec.get());
    }
}
