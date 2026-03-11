package frc.robot.coordination.shooting;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter.ReadinessDiagnostics;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.transfer.TransferConstants;
import frc.robot.util.AutoAimHeadingConfig;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ShootCoordinator {
    private final Shooter shooter;
    private final Transfer transfer;
    private final ShootCoordinatorConstants.FeedGateMode feedGateMode;
    private int readyStableCycles = 0;
    private boolean activelyFeeding = false;

    public ShootCoordinator(Shooter shooter, Transfer transfer) {
        this(shooter, transfer, ShootCoordinatorConstants.DEFAULT_FEED_GATE_MODE);
    }

    public ShootCoordinator(
            Shooter shooter,
            Transfer transfer,
            ShootCoordinatorConstants.FeedGateMode feedGateMode) {
        this.shooter = Objects.requireNonNull(shooter, "shooter");
        this.transfer = Objects.requireNonNull(transfer, "transfer");
        this.feedGateMode = Objects.requireNonNull(feedGateMode, "feedGateMode");
    }

    public Command aimForDistance(DoubleSupplier distanceMetersSupplier) {
        return shooter.aimForDistance(distanceMetersSupplier).withName("ShootCoordinatorAim");
    }

    public Command shootForDistance(DoubleSupplier distanceMetersSupplier) {
        return shootForDistance(distanceMetersSupplier, () -> true).withName("ShootCoordinatorShootForDistance");
    }

    public Command shootForDistance(DoubleSupplier distanceMetersSupplier, BooleanSupplier aimReadySupplier) {
        return shootForDistance(distanceMetersSupplier, aimReadySupplier, () -> false, () -> true)
                .withName("ShootCoordinatorShootForDistance");
    }

    public Command shootForDistance(
            DoubleSupplier distanceMetersSupplier,
            BooleanSupplier aimReadySupplier,
            BooleanSupplier manualFeedOverrideSupplier) {
        return shootForDistance(distanceMetersSupplier, aimReadySupplier, manualFeedOverrideSupplier, () -> true)
                .withName("ShootCoordinatorShootForDistance");
    }

    public Command shootForDistance(
            DoubleSupplier distanceMetersSupplier,
            BooleanSupplier aimReadySupplier,
            BooleanSupplier manualFeedOverrideSupplier,
            BooleanSupplier automaticFeedEnabledSupplier) {
        return Commands.runEnd(
                        () -> executeShoot(
                                distanceMetersSupplier,
                                aimReadySupplier,
                                manualFeedOverrideSupplier,
                                automaticFeedEnabledSupplier),
                        this::stopAllOutputs,
                        shooter,
                        transfer)
                .withName("ShootCoordinatorShootForDistance");
    }

    public Command manualFeedCommand() {
        return Commands.runEnd(
                        this::applyManualFeedOutputs,
                        this::stopFeedOutputs,
                        shooter,
                        transfer)
                .withName("ShootCoordinatorManualFeed");
    }

    private void executeShoot(
            DoubleSupplier distanceMetersSupplier,
            BooleanSupplier aimReadySupplier,
            BooleanSupplier manualFeedOverrideSupplier,
            BooleanSupplier automaticFeedEnabledSupplier) {
        double distanceMeters = distanceMetersSupplier.getAsDouble();
        boolean distanceValid = Double.isFinite(distanceMeters);
        if (distanceValid) {
            shooter.setTargetsForDistance(distanceMeters);
        }

        // Use same-cycle readiness after targets are updated to avoid one-loop stale gate
        // decisions at mode transitions / target changes.
        ReadinessDiagnostics readiness = shooter.getReadinessDiagnosticsNow();
        boolean aimReady = aimReadySupplier.getAsBoolean();
        boolean manualFeedOverride = manualFeedOverrideSupplier.getAsBoolean();
        boolean automaticFeedEnabled = automaticFeedEnabledSupplier.getAsBoolean();
        GateEvaluation gateEvaluation =
                evaluateFeedGate(
                        distanceValid,
                        readiness.atSetpoint(),
                        aimReady,
                        manualFeedOverride,
                        automaticFeedEnabled,
                        readyStableCycles);
        readyStableCycles = gateEvaluation.nextReadyStableCycles();

        applyFeedOutputs(gateEvaluation.gateDecision().gateOpen(), manualFeedOverride);

        logShootingState(
                distanceMeters,
                distanceValid,
                readiness.atSetpoint(),
                aimReady,
                manualFeedOverride,
                automaticFeedEnabled,
                gateEvaluation.gateDecision());
    }

    private void applyManualFeedOutputs() {
        applyFeedOutputs(false, true);
    }

    private void applyFeedOutputs(boolean gateOpen, boolean manualFeedOverride) {
        activelyFeeding = gateOpen || manualFeedOverride;
        if (!activelyFeeding) {
            stopFeedOutputs();
            return;
        }
        shooter.setKickerTorqueAmps(ShooterConstants.DEFAULT_KICKER_TORQUE_AMPS);
        transfer.setPercent(TransferConstants.RUN_TRANSFER_PERCENT);
    }

    private void stopFeedOutputs() {
        shooter.stopKicker();
        transfer.stopAll();
    }

    private void logShootingState(
            double distanceMeters,
            boolean distanceValid,
            boolean shooterAtSetpoint,
            boolean aimReady,
            boolean manualFeedOverride,
            boolean automaticFeedEnabled,
            GateDecision gateDecision) {
        Logger.recordOutput("Shooting/DistanceMeters", distanceMeters);
        Logger.recordOutput("Shooting/DistanceValid", distanceValid);
        Logger.recordOutput("Shooting/FeedGateMode", feedGateMode.name());
        Logger.recordOutput("Shooting/FeedGatePolicy", "InlineModeSwitch");
        Logger.recordOutput("Shooting/AimToleranceRad", AutoAimHeadingConfig.AIM_TOLERANCE_RAD);
        Logger.recordOutput("Shooting/ShooterAtSetpoint", shooterAtSetpoint);
        Logger.recordOutput("Shooting/AimReady", aimReady);
        Logger.recordOutput("Shooting/ManualFeedOverride", manualFeedOverride);
        Logger.recordOutput("Shooting/AutomaticFeedEnabled", automaticFeedEnabled);
        Logger.recordOutput("Shooting/GateOpen", gateDecision.gateOpen());
        Logger.recordOutput("Shooting/BlockReason", gateDecision.blockReason());
        Logger.recordOutput("Shooting/ReadyStableCycles", readyStableCycles);

        String state;
        if (manualFeedOverride) {
            state = "MANUAL_FEED_OVERRIDE";
        } else if (!automaticFeedEnabled) {
            state = "AIM_ONLY_OVERRIDE";
        } else if (!distanceValid) {
            state = "NO_TARGET";
        } else if (gateDecision.gateOpen()) {
            state = "SHOOTING";
        } else if (shooterAtSetpoint && aimReady) {
            state = "READY";
        } else if (shooterAtSetpoint) {
            state = "AIMING";
        } else {
            state = "SPINNING_UP";
        }
        Logger.recordOutput("Shooting/State", state);
    }

    private void stopAllOutputs() {
        readyStableCycles = 0;
        activelyFeeding = false;
        shooter.stopAll();
        transfer.stopAll();
    }

    public boolean isActivelyFeeding() {
        return activelyFeeding;
    }

    private GateEvaluation evaluateFeedGate(
            boolean distanceValid,
            boolean shooterAtSetpoint,
            boolean aimReady,
            boolean manualFeedOverride,
            boolean automaticFeedEnabled,
            int previousReadyStableCycles) {
        if (manualFeedOverride) {
            return new GateEvaluation(new GateDecision(true, "ManualFeedOverride"), 0);
        }
        if (!automaticFeedEnabled) {
            return new GateEvaluation(new GateDecision(false, "FeedingDisabledOverride"), 0);
        }
        if (!distanceValid) {
            return new GateEvaluation(new GateDecision(false, "DistanceInvalid"), 0);
        }
        return switch (feedGateMode) {
            case IMMEDIATE -> {
                yield new GateEvaluation(new GateDecision(true, ""), 0);
            }
            case SHOOTER_AT_SETPOINT -> {
                if (!shooterAtSetpoint) {
                    yield new GateEvaluation(new GateDecision(false, "ShooterNotAtSetpoint"), 0);
                }
                int nextReadyStableCycles = previousReadyStableCycles + 1;
                boolean gateOpen = nextReadyStableCycles >= ShootCoordinatorConstants.GATE_READY_DEBOUNCE_CYCLES;
                yield new GateEvaluation(
                        new GateDecision(gateOpen, gateOpen ? "" : "ReadinessDebounce"),
                        nextReadyStableCycles);
            }
            case SHOOTER_AND_AIM -> {
                if (shooterAtSetpoint && aimReady) {
                    int nextReadyStableCycles = previousReadyStableCycles + 1;
                    boolean gateOpen = nextReadyStableCycles >= ShootCoordinatorConstants.GATE_READY_DEBOUNCE_CYCLES;
                    yield new GateEvaluation(
                            new GateDecision(gateOpen, gateOpen ? "" : "ReadinessDebounce"),
                            nextReadyStableCycles);
                }
                if (!shooterAtSetpoint && !aimReady) {
                    yield new GateEvaluation(new GateDecision(false, "ShooterNotAtSetpoint+AimNotReady"), 0);
                }
                if (!shooterAtSetpoint) {
                    yield new GateEvaluation(new GateDecision(false, "ShooterNotAtSetpoint"), 0);
                }
                yield new GateEvaluation(new GateDecision(false, "AimNotReady"), 0);
            }
        };
    }

    private record GateDecision(boolean gateOpen, String blockReason) {}
    private record GateEvaluation(GateDecision gateDecision, int nextReadyStableCycles) {}
}
