package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import frc.robot.util.LoggedTunableNumber;

public final class IntakeConstants {
    private IntakeConstants() {}

    public static final int ROLLER_MOTOR_ID = 25;
    public static final int LEFT_MOTOR_ID = 23;
    public static final int RIGHT_MOTOR_ID = 24;

    public static final double ROLLER_RPM = 4800.0;
    public static final double SLOW_ROLLER_RPM = 0.0;
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
    public static final double DRIVER_TRIGGER_WIGGLE_PEAK_IN = 12.1;
    public static final double DRIVER_TRIGGER_WIGGLE_BASELINE_ROT =
            DRIVER_TRIGGER_WIGGLE_BASELINE_IN * MOTOR_REV_PER_IN;
    public static final double DRIVER_TRIGGER_WIGGLE_PEAK_ROT =
            DRIVER_TRIGGER_WIGGLE_PEAK_IN * MOTOR_REV_PER_IN;
    public static final double DRIVER_TRIGGER_WIGGLE_SWITCH_INTERVAL_SEC = 0.15;

    public static final double DRIVER_AGITATION_BASELINE_IN = FULL_TRAVEL_IN;
    public static final double DRIVER_AGITATION_PEAK_IN = 12.1;
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
    public static final double SMART_RETRACT_HALF_RETRACT_POSITION_ROT = 10.0;
    public static final double SMART_RETRACT_CURRENT_FILTER_ALPHA = 0.20;
    public static final int SMART_RETRACT_FEED_ENGAGE_CYCLES = 2;
    public static final double SMART_RETRACT_FEED_START_DELAY_SEC = 0.4;

    public static final double SMART_RETRACT_NIBBLE_CURRENT_THRESHOLD_AMPS = 4.0;
    public static final int SMART_RETRACT_NIBBLE_DETECT_CYCLES = 2;
    public static final double SMART_RETRACT_NIBBLE_STEP_ROT = 0.50;
    public static final double SMART_RETRACT_NIBBLE_BACKOFF_ROT = 7.0;
    public static final double SMART_RETRACT_NIBBLE_BACKOFF_DWELL_SEC = 0.3;

    private static final LoggedTunableNumber smartRetractRetractedPositionRot =
            new LoggedTunableNumber(
                    "Intake/SmartRetract/RetractedPositionRot",
                    SMART_RETRACT_RETRACTED_POSITION_ROT);
    private static final LoggedTunableNumber smartRetractHalfRetractPositionRot =
            new LoggedTunableNumber(
                    "Intake/SmartRetract/HalfRetractPositionRot",
                    SMART_RETRACT_HALF_RETRACT_POSITION_ROT);
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

    private static final LoggedTunableNumber smartRetractNibbleCurrentThresholdAmps =
            new LoggedTunableNumber(
                    "Intake/SmartRetract/NibbleCurrentThresholdAmps",
                    SMART_RETRACT_NIBBLE_CURRENT_THRESHOLD_AMPS);
    private static final LoggedTunableNumber smartRetractNibbleDetectCycles =
            new LoggedTunableNumber(
                    "Intake/SmartRetract/NibbleDetectCycles",
                    SMART_RETRACT_NIBBLE_DETECT_CYCLES);
    private static final LoggedTunableNumber smartRetractNibbleStepRot =
            new LoggedTunableNumber(
                    "Intake/SmartRetract/NibbleStepRot",
                    SMART_RETRACT_NIBBLE_STEP_ROT);
    private static final LoggedTunableNumber smartRetractNibbleBackoffRot =
            new LoggedTunableNumber(
                    "Intake/SmartRetract/NibbleBackoffRot",
                    SMART_RETRACT_NIBBLE_BACKOFF_ROT);
    private static final LoggedTunableNumber smartRetractNibbleBackoffDwellSec =
            new LoggedTunableNumber(
                    "Intake/SmartRetract/NibbleBackoffDwellSec",
                    SMART_RETRACT_NIBBLE_BACKOFF_DWELL_SEC);

    public static double smartRetractRetractedPositionRot() {
        return MathUtil.clamp(smartRetractRetractedPositionRot.get(), 0.0, EXTENDED_POSITION_ROT);
    }

    public static double smartRetractHalfRetractPositionRot() {
        return MathUtil.clamp(
                smartRetractHalfRetractPositionRot.get(),
                smartRetractRetractedPositionRot(),
                EXTENDED_POSITION_ROT);
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

    public static double smartRetractNibbleCurrentThresholdAmps() {
        return Math.max(0.0, smartRetractNibbleCurrentThresholdAmps.get());
    }

    public static int smartRetractNibbleDetectCycles() {
        return Math.max(1, (int) Math.round(smartRetractNibbleDetectCycles.get()));
    }

    public static double smartRetractNibbleStepRot() {
        return Math.max(0.0, smartRetractNibbleStepRot.get());
    }

    public static double smartRetractNibbleBackoffRot() {
        return Math.max(0.0, smartRetractNibbleBackoffRot.get());
    }

    public static double smartRetractNibbleBackoffDwellSec() {
        return Math.max(0.0, smartRetractNibbleBackoffDwellSec.get());
    }
}
