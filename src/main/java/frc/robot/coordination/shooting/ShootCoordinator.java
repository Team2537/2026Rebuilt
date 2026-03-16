package frc.robot.coordination.shooting;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ReadinessMode;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter.ReadinessDiagnostics;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.transfer.TransferConstants;
import frc.robot.util.AutoAimHeadingConfig;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShootCoordinator {
    private final Shooter shooter;
    private final Transfer transfer;
    private final ShootCoordinatorConstants.FeedGateMode feedGateMode;
    private int readyStableCycles = 0;
    private int readyDropStableCycles = 0;
    private boolean feedGateOpen = false;
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
        return shootForDistance(distanceMetersSupplier, aimReadySupplier, () -> false, () -> false, () -> true)
                .withName("ShootCoordinatorShootForDistance");
    }

    public Command shootForDistance(
            DoubleSupplier distanceMetersSupplier,
            BooleanSupplier aimReadySupplier,
            BooleanSupplier manualFeedOverrideSupplier) {
        return shootForDistance(
                        distanceMetersSupplier,
                        aimReadySupplier,
                        () -> false,
                        manualFeedOverrideSupplier,
                        () -> true)
                .withName("ShootCoordinatorShootForDistance");
    }

    public Command shootForDistance(
            DoubleSupplier distanceMetersSupplier,
            BooleanSupplier aimReadySupplier,
            BooleanSupplier shotOnMoveSupplier,
            BooleanSupplier manualFeedOverrideSupplier,
            BooleanSupplier automaticFeedEnabledSupplier) {
        return shootForDistance(
                        distanceMetersSupplier,
                        aimReadySupplier,
                        () -> shotOnMoveSupplier.getAsBoolean()
                                ? ReadinessMode.SHOT_ON_MOVE
                                : ReadinessMode.STATIONARY,
                        manualFeedOverrideSupplier,
                        automaticFeedEnabledSupplier)
                .withName("ShootCoordinatorShootForDistance");
    }

    public Command shootForDistance(
            DoubleSupplier distanceMetersSupplier,
            BooleanSupplier aimReadySupplier,
            Supplier<ReadinessMode> readinessModeSupplier,
            BooleanSupplier manualFeedOverrideSupplier,
            BooleanSupplier automaticFeedEnabledSupplier) {
        return Commands.runEnd(
                        () -> executeShoot(
                                distanceMetersSupplier,
                                aimReadySupplier,
                                readinessModeSupplier,
                                manualFeedOverrideSupplier,
                                automaticFeedEnabledSupplier),
                        this::stopAllOutputs,
                        shooter,
                        transfer)
                .withName("ShootCoordinatorShootForDistance");
    }

    public Command shootForDistance(
            DoubleSupplier distanceMetersSupplier,
            BooleanSupplier aimReadySupplier,
            BooleanSupplier manualFeedOverrideSupplier,
            BooleanSupplier automaticFeedEnabledSupplier) {
        return shootForDistance(
                        distanceMetersSupplier,
                        aimReadySupplier,
                        () -> false,
                        manualFeedOverrideSupplier,
                        automaticFeedEnabledSupplier)
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
            Supplier<ReadinessMode> readinessModeSupplier,
            BooleanSupplier manualFeedOverrideSupplier,
            BooleanSupplier automaticFeedEnabledSupplier) {
        double distanceMeters = distanceMetersSupplier.getAsDouble();
        boolean distanceValid = Double.isFinite(distanceMeters);
        if (distanceValid) {
            shooter.setTargetsForDistance(distanceMeters);
        }

        // Use same-cycle readiness after targets are updated to avoid one-loop stale gate
        // decisions at mode transitions / target changes.
        ReadinessMode readinessMode = Objects.requireNonNullElse(readinessModeSupplier.get(), ReadinessMode.STATIONARY);
        boolean shotOnMove = readinessMode != ReadinessMode.STATIONARY;
        ReadinessDiagnostics readiness = shooter.getReadinessDiagnosticsNow(readinessMode);
        boolean aimReady = aimReadySupplier.getAsBoolean();
        boolean manualFeedOverride = manualFeedOverrideSupplier.getAsBoolean();
        boolean automaticFeedEnabled = automaticFeedEnabledSupplier.getAsBoolean();
        GateEvaluation gateEvaluation =
                evaluateFeedGate(
                        distanceValid,
                        readiness.atSetpoint(),
                        aimReady,
                        shotOnMove,
                        manualFeedOverride,
                        automaticFeedEnabled,
                        feedGateOpen,
                        readyStableCycles,
                        readyDropStableCycles);
        readyStableCycles = gateEvaluation.nextReadyStableCycles();
        readyDropStableCycles = gateEvaluation.nextReadyDropStableCycles();
        feedGateOpen = gateEvaluation.gateDecision().gateOpen();

        applyFeedOutputs(gateEvaluation.gateDecision().gateOpen());

        logShootingState(
                distanceMeters,
                distanceValid,
                readiness.atSetpoint(),
                aimReady,
                readinessMode,
                shotOnMove,
                manualFeedOverride,
                automaticFeedEnabled,
                gateEvaluation.gateDecision());
    }

    private void applyManualFeedOutputs() {
        applyFeedOutputs(true);
    }

    private void applyFeedOutputs(boolean gateOpen) {
        activelyFeeding = gateOpen;
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
            ReadinessMode readinessMode,
            boolean shotOnMove,
            boolean manualFeedOverride,
            boolean automaticFeedEnabled,
            GateDecision gateDecision) {
        Logger.recordOutput("Shooting/DistanceMeters", distanceMeters);
        Logger.recordOutput("Shooting/DistanceValid", distanceValid);
        Logger.recordOutput("Shooting/FeedGateMode", feedGateMode.name());
        Logger.recordOutput("Shooting/FeedGatePolicy", "InlineModeSwitch");
        Logger.recordOutput(
                "Shooting/AimToleranceRad",
                switch (readinessMode) {
                    case STATIONARY -> AutoAimHeadingConfig.AIM_TOLERANCE_RAD;
                    case SHOT_ON_MOVE -> AutoAimHeadingConfig.SHOT_ON_MOVE_AIM_TOLERANCE_RAD;
                    case PASSING -> AutoAimHeadingConfig.PASS_AIM_TOLERANCE_RAD;
                });
        Logger.recordOutput("Shooting/ShooterAtSetpoint", shooterAtSetpoint);
        Logger.recordOutput("Shooting/AimReady", aimReady);
        Logger.recordOutput("Shooting/ReadinessMode", readinessMode.name());
        Logger.recordOutput("Shooting/ShotOnMove", shotOnMove);
        Logger.recordOutput("Shooting/ManualFeedOverride", manualFeedOverride);
        Logger.recordOutput("Shooting/AutomaticFeedEnabled", automaticFeedEnabled);
        Logger.recordOutput("Shooting/GateOpen", gateDecision.gateOpen());
        Logger.recordOutput("Shooting/BlockReason", gateDecision.blockReason());
        Logger.recordOutput("Shooting/ReadyStableCycles", readyStableCycles);
        Logger.recordOutput("Shooting/ReadyDropStableCycles", readyDropStableCycles);

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
        readyDropStableCycles = 0;
        feedGateOpen = false;
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
            boolean shotOnMove,
            boolean manualFeedOverride,
            boolean automaticFeedEnabled,
            boolean previousGateOpen,
            int previousReadyStableCycles,
            int previousReadyDropStableCycles) {
        if (!distanceValid) {
            return new GateEvaluation(new GateDecision(false, "DistanceInvalid"), 0, 0);
        }
        if (manualFeedOverride) {
            return new GateEvaluation(new GateDecision(true, "ManualFeedOverride"), 0, 0);
        }
        if (!automaticFeedEnabled) {
            return new GateEvaluation(new GateDecision(false, "FeedingDisabledOverride"), 0, 0);
        }
        return switch (feedGateMode) {
            case IMMEDIATE -> {
                yield new GateEvaluation(new GateDecision(true, ""), 0, 0);
            }
            case SHOOTER_AT_SETPOINT -> {
                if (shooterAtSetpoint) {
                    int nextReadyStableCycles = previousReadyStableCycles + 1;
                    boolean gateOpen = nextReadyStableCycles >= ShootCoordinatorConstants.GATE_READY_DEBOUNCE_CYCLES;
                    yield new GateEvaluation(
                            new GateDecision(gateOpen, gateOpen ? "" : "ReadinessDebounce"),
                            nextReadyStableCycles,
                            0);
                }
                yield holdOrCloseForReadinessDrop(
                        shotOnMove,
                        previousGateOpen,
                        previousReadyDropStableCycles,
                        "ShooterNotAtSetpoint");
            }
            case SHOOTER_AND_AIM -> {
                if (shooterAtSetpoint && aimReady) {
                    int nextReadyStableCycles = previousReadyStableCycles + 1;
                    boolean gateOpen = nextReadyStableCycles >= ShootCoordinatorConstants.GATE_READY_DEBOUNCE_CYCLES;
                    yield new GateEvaluation(
                            new GateDecision(gateOpen, gateOpen ? "" : "ReadinessDebounce"),
                            nextReadyStableCycles,
                            0);
                }
                if (!shooterAtSetpoint && !aimReady) {
                    yield holdOrCloseForReadinessDrop(
                            shotOnMove,
                            previousGateOpen,
                            previousReadyDropStableCycles,
                            "ShooterNotAtSetpoint+AimNotReady");
                }
                if (!shooterAtSetpoint) {
                    yield holdOrCloseForReadinessDrop(
                            shotOnMove,
                            previousGateOpen,
                            previousReadyDropStableCycles,
                            "ShooterNotAtSetpoint");
                }
                yield holdOrCloseForReadinessDrop(
                        shotOnMove,
                        previousGateOpen,
                        previousReadyDropStableCycles,
                        "AimNotReady");
            }
        };
    }

    private GateEvaluation holdOrCloseForReadinessDrop(
            boolean shotOnMove,
            boolean previousGateOpen,
            int previousReadyDropStableCycles,
            String closedBlockReason) {
        if (shotOnMove && previousGateOpen) {
            int nextReadyDropStableCycles = previousReadyDropStableCycles + 1;
            if (nextReadyDropStableCycles < ShootCoordinatorConstants.SHOT_ON_MOVE_GATE_DROP_DEBOUNCE_CYCLES) {
                return new GateEvaluation(
                        new GateDecision(true, "ShotOnMoveReadinessDropDebounce"),
                        0,
                        nextReadyDropStableCycles);
            }
        }
        return new GateEvaluation(new GateDecision(false, closedBlockReason), 0, 0);
    }

    private record GateDecision(boolean gateOpen, String blockReason) {}
    private record GateEvaluation(
            GateDecision gateDecision,
            int nextReadyStableCycles,
            int nextReadyDropStableCycles) {}
}
