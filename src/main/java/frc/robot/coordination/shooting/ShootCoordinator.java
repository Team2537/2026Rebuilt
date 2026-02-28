package frc.robot.coordination.shooting;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter.ReadinessDiagnostics;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.transfer.TransferConstants;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ShootCoordinator {
    private final Shooter shooter;
    private final Transfer transfer;
    private final Constants.FeedGateMode feedGateMode;
    private int readyStableCycles = 0;

    public ShootCoordinator(Shooter shooter, Transfer transfer) {
        this(shooter, transfer, Constants.SHOOTING_FEED_GATE_MODE);
    }

    public ShootCoordinator(Shooter shooter, Transfer transfer, Constants.FeedGateMode feedGateMode) {
        this.shooter = Objects.requireNonNull(shooter, "shooter");
        this.transfer = Objects.requireNonNull(transfer, "transfer");
        this.feedGateMode = Objects.requireNonNull(feedGateMode, "feedGateMode");
    }

    public Command aimForDistance(DoubleSupplier distanceMetersSupplier) {
        return Commands.runEnd(
                        () -> shooter.setTargetsForDistance(distanceMetersSupplier.getAsDouble()),
                        shooter::stopAll,
                        shooter)
                .withName("ShootCoordinatorAim");
    }

    public Command shootForDistance(DoubleSupplier distanceMetersSupplier) {
        return shootForDistance(distanceMetersSupplier, () -> true).withName("ShootCoordinatorShootForDistance");
    }

    public Command shootForDistance(DoubleSupplier distanceMetersSupplier, BooleanSupplier aimReadySupplier) {
        return Commands.runEnd(
                        () -> executeShoot(distanceMetersSupplier, aimReadySupplier),
                        this::stopAllOutputs,
                        shooter,
                        transfer)
                .withName("ShootCoordinatorShootForDistance");
    }

    private void executeShoot(DoubleSupplier distanceMetersSupplier, BooleanSupplier aimReadySupplier) {
        double distanceMeters = distanceMetersSupplier.getAsDouble();
        boolean distanceValid = Double.isFinite(distanceMeters);
        if (distanceValid) {
            shooter.setTargetsForDistance(distanceMeters);
        }

        // Use same-cycle readiness after targets are updated to avoid one-loop stale gate
        // decisions at mode transitions / target changes.
        ReadinessDiagnostics readiness = shooter.getReadinessDiagnosticsNow();
        boolean aimReady = aimReadySupplier.getAsBoolean();
        GateDecision gateDecision = evaluateFeedGate(distanceValid, readiness.atSetpoint(), aimReady);

        applyFeedOutputs(gateDecision.gateOpen());

        logShootingState(distanceMeters, distanceValid, readiness.atSetpoint(), aimReady, gateDecision);
    }

    private void applyFeedOutputs(boolean gateOpen) {
        if (!gateOpen) {
            shooter.stopKicker();
            transfer.stopAll();
            return;
        }
        shooter.setKickerTorqueAmps(ShooterConstants.DEFAULT_KICKER_TORQUE_AMPS);
        transfer.setPercent(TransferConstants.RUN_TRANSFER_PERCENT);
    }

    private void logShootingState(
            double distanceMeters,
            boolean distanceValid,
            boolean shooterAtSetpoint,
            boolean aimReady,
            GateDecision gateDecision) {
        Logger.recordOutput("Shooting/DistanceMeters", distanceMeters);
        Logger.recordOutput("Shooting/DistanceValid", distanceValid);
        Logger.recordOutput("Shooting/FeedGateMode", feedGateMode.name());
        Logger.recordOutput("Shooting/FeedGatePolicy", "InlineModeSwitch");
        Logger.recordOutput("Shooting/AimToleranceRad", Constants.SHOOTING_AIM_TOLERANCE_RAD);
        Logger.recordOutput("Shooting/ShooterAtSetpoint", shooterAtSetpoint);
        Logger.recordOutput("Shooting/AimReady", aimReady);
        Logger.recordOutput("Shooting/GateOpen", gateDecision.gateOpen());
        Logger.recordOutput("Shooting/BlockReason", gateDecision.blockReason());

        String state;
        if (!distanceValid) {
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
        shooter.stopAll();
        transfer.stopAll();
    }

    private GateDecision evaluateFeedGate(boolean distanceValid, boolean shooterAtSetpoint, boolean aimReady) {
        if (!distanceValid) {
            readyStableCycles = 0;
            return new GateDecision(false, "DistanceInvalid");
        }
        return switch (feedGateMode) {
            case IMMEDIATE -> {
                readyStableCycles = 0;
                yield new GateDecision(true, "");
            }
            case SHOOTER_AT_SETPOINT -> {
                if (!shooterAtSetpoint) {
                    readyStableCycles = 0;
                    yield new GateDecision(false, "ShooterNotAtSetpoint");
                }
                readyStableCycles++;
                yield readyStableCycles >= Constants.SHOOTING_GATE_READY_DEBOUNCE_CYCLES
                        ? new GateDecision(true, "")
                        : new GateDecision(false, "ReadinessDebounce");
            }
            case SHOOTER_AND_AIM -> {
                if (shooterAtSetpoint && aimReady) {
                    readyStableCycles++;
                    yield readyStableCycles >= Constants.SHOOTING_GATE_READY_DEBOUNCE_CYCLES
                            ? new GateDecision(true, "")
                            : new GateDecision(false, "ReadinessDebounce");
                }
                readyStableCycles = 0;
                if (!shooterAtSetpoint && !aimReady) {
                    yield new GateDecision(false, "ShooterNotAtSetpoint+AimNotReady");
                }
                if (!shooterAtSetpoint) {
                    yield new GateDecision(false, "ShooterNotAtSetpoint");
                }
                yield new GateDecision(false, "AimNotReady");
            }
        };
    }

    private record GateDecision(boolean gateOpen, String blockReason) {}
}
