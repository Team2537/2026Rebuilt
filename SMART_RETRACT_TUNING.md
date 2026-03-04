# Smart Retract Tuning (Real Robot)

This is a quick field guide for tuning the new intake smart retract behavior on real hardware.

## What Exists

- Smart retract only moves while **shoot feed is active** (feed gate open).
- If shoot is released early and intake is not fully retracted, it commands back to extended.
- Two modes are available from dashboard:
  - `Intake/SmartRetract/EnableNibble`
  - `Intake/SmartRetract/EnableHalfRetractReturn`
- If both are enabled, `NIBBLE` wins (priority).
- Status key:
  - `Intake/SmartRetract/StatusMode`

## Where To Tune Constants

- File: `src/main/java/frc/robot/subsystems/intake/IntakeConstants.java`

## Preflight (Do This First)

1. Keep smart retract modes disabled.
2. Verify intake homing and manual extend/retract work cleanly.
3. Verify slow retract (`X`) feels materially gentler than normal retract.
4. Verify shoot command works normally with intake extended.

## Signals To Watch in AdvantageScope

- `Intake/SmartRetract/SelectedMode`
- `Intake/SmartRetract/FeedLatched`
- `Intake/SmartRetract/FeedTrueCycles`
- `Intake/SmartRetract/FeedFalseCycles`
- `Intake/SmartRetract/SignalCurrentRawAmps`
- `Intake/SmartRetract/SignalCurrentFilteredAmps`
- `Intake/SmartRetract/SignalBaselineAmps`
- `Intake/SmartRetract/CommandedTargetRot`
- `Intake/SmartRetract/NibbleSpikeCycles`
- `Intake/SmartRetract/NibbleBackoffActive`
- `Intake/SmartRetract/RestoreExtendedOnExit`
- Intake position/current traces (`Intake/*`)

## Recommended Tuning Order

1. Tune feed start delay first.
2. Tune `NIBBLE`.
3. Tune `HALF_RETRACT_RETURN`.
4. Pick one mode for matches.

## 1) Feed Start Delay

Start with defaults:
- `SMART_RETRACT_FEED_START_DELAY_SEC = 1.0`
- `SMART_RETRACT_FEED_ENGAGE_CYCLES = 2` (minimum floor; delay dominates at 50 Hz)

If retract starts too early:
- Increase `SMART_RETRACT_FEED_START_DELAY_SEC`.
If retract starts too late:
- Decrease `SMART_RETRACT_FEED_START_DELAY_SEC`.

## 2) NIBBLE Mode Tuning

Enable:
- `EnableNibble = true`
- `EnableHalfRetractReturn = false`

Start from defaults and test repeated shoot/release cycles.

Main knobs:
- `SMART_RETRACT_NIBBLE_CURRENT_DELTA_AMPS`
- `SMART_RETRACT_NIBBLE_DETECT_CYCLES`
- `SMART_RETRACT_NIBBLE_STEP_ROT`
- `SMART_RETRACT_NIBBLE_BACKOFF_ROT`
- `SMART_RETRACT_NIBBLE_BACKOFF_DWELL_SEC`

Adjustments:
- Retracts too aggressively into compression:
  - Increase `CURRENT_DELTA_AMPS`, reduce `STEP_ROT`, or increase `BACKOFF_ROT`.
- Barely retracts:
  - Lower `CURRENT_DELTA_AMPS` or increase `STEP_ROT`.
- Jitters near compression:
  - Increase `DETECT_CYCLES` and/or `BACKOFF_DWELL_SEC`.

## 3) HALF_RETRACT_RETURN Mode Tuning

Enable:
- `EnableNibble = false`
- `EnableHalfRetractReturn = true`

Main knobs:
- `SMART_RETRACT_HALF_RETRACT_POSITION_ROT`
- `SLOW_INTAKE_VELOCITY`
- `SLOW_INTAKE_ACCELERATION`
- `SLOW_RETRACT_MAX_VOLTS`

Adjustments:
- Doesn’t retract enough before returning:
  - Lower `SMART_RETRACT_HALF_RETRACT_POSITION_ROT` (deeper retract target).
- Retract/return is too abrupt:
  - Reduce `SLOW_INTAKE_VELOCITY` and/or `SLOW_RETRACT_MAX_VOLTS`.
- Retract/return is too slow:
  - Increase `SLOW_INTAKE_VELOCITY` carefully.

## Pass/Fail Checklist

- No visible retract flicker while holding shoot at marginal readiness.
- Smart retract pauses cleanly when feed is not active.
- Early shoot release returns intake to extended (unless already fully retracted).
- Full retract before release stays retracted.
- Driver reports predictable behavior for at least 10 repeated cycles.

## Practical Tips

- Change one constant at a time.
- Keep a tuning log with battery voltage and note load.
- Retest at both full and partially depleted battery.
- Stop and revert if behavior becomes non-deterministic under repeated cycles.
