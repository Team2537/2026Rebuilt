package frc.robot.coordination.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ReadinessDiagnostics;
import frc.robot.subsystems.shooter.ShooterConstants;
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
    private final ShotGate shotGate;
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
        this.shotGate = new ShotGate(feedGateMode);
    }

    public Command aimForDistance(DoubleSupplier distanceMetersSupplier) {
        return aimForShot(() -> createDistanceOnlySolution(distanceMetersSupplier.getAsDouble()))
                .withName("ShootCoordinatorAim");
    }

    public Command aimForShot(Supplier<ShotSolution> shotSolutionSupplier) {
        return Commands.runEnd(
                        () -> executeAim(shotSolutionSupplier),
                        this::stopAimOutputs,
                        shooter)
                .withName("ShootCoordinatorAim");
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
        return shootForDistance(
                        distanceMetersSupplier,
                        aimReadySupplier,
                        manualFeedOverrideSupplier,
                        () -> true)
                .withName("ShootCoordinatorShootForDistance");
    }

    public Command shootForDistance(
            DoubleSupplier distanceMetersSupplier,
            BooleanSupplier aimReadySupplier,
            BooleanSupplier manualFeedOverrideSupplier,
            BooleanSupplier automaticFeedEnabledSupplier) {
        return shootForShot(
                        () -> createDistanceOnlySolution(distanceMetersSupplier.getAsDouble()),
                        aimReadySupplier,
                        manualFeedOverrideSupplier,
                        automaticFeedEnabledSupplier)
                .withName("ShootCoordinatorShootForDistance");
    }

    public Command shootForShot(
            Supplier<ShotSolution> shotSolutionSupplier,
            BooleanSupplier aimReadySupplier,
            BooleanSupplier manualFeedOverrideSupplier,
            BooleanSupplier automaticFeedEnabledSupplier) {
        return Commands.runEnd(
                        () -> executeShoot(
                                shotSolutionSupplier,
                                aimReadySupplier,
                                manualFeedOverrideSupplier,
                                automaticFeedEnabledSupplier),
                        this::stopAllOutputs,
                        shooter,
                        transfer)
                .withName("ShootCoordinatorShootForShot");
    }

    public Command manualFeedCommand() {
        return Commands.runEnd(
                        this::applyManualFeedOutputs,
                        this::stopFeedOutputs,
                        shooter,
                        transfer)
                .withName("ShootCoordinatorManualFeed");
    }

    private void executeAim(Supplier<ShotSolution> shotSolutionSupplier) {
        ShotSolution solution = Objects.requireNonNullElse(shotSolutionSupplier.get(), ShotSolution.invalid(new Pose2d()));
        if (solution.valid() && solution.shooterSetpoint() != null) {
            shooter.setTargets(solution.shooterSetpoint());
            shooter.setActiveReadinessProfile(solution.shooterRpmTolerance(), readinessLabelFor(solution));
            Shooter.ReadinessDiagnostics readiness = shooter.getReadinessDiagnosticsNow(solution.shooterRpmTolerance());
            shooter.publishActiveReadiness(
                    readiness,
                    solution.shooterRpmTolerance(),
                    readinessLabelFor(solution));
        } else {
            shooter.resetActiveReadinessProfile();
            shooter.stopAll();
            shooter.publishActiveReadiness(
                    shooter.getReadinessDiagnosticsNow(),
                    ShooterConstants.scoreShooterRpmTolerance(),
                    "SCORE");
        }

        Logger.recordOutput("Shooting/Intent", solution.intent().name());
        Logger.recordOutput("Shooting/DistanceMeters", solution.distanceMeters());
        Logger.recordOutput("Shooting/DistanceValid", solution.valid());
        Logger.recordOutput("Shooting/TargetPose", solution.targetPose());
        Logger.recordOutput("Shooting/ShooterAimOnly", true);
    }

    private void executeShoot(
            Supplier<ShotSolution> shotSolutionSupplier,
            BooleanSupplier aimReadySupplier,
            BooleanSupplier manualFeedOverrideSupplier,
            BooleanSupplier automaticFeedEnabledSupplier) {
        ShotSolution solution = Objects.requireNonNullElse(shotSolutionSupplier.get(), ShotSolution.invalid(new Pose2d()));
        if (solution.valid() && solution.shooterSetpoint() != null) {
            shooter.setTargets(solution.shooterSetpoint());
            shooter.setActiveReadinessProfile(solution.shooterRpmTolerance(), readinessLabelFor(solution));
        } else {
            shooter.resetActiveReadinessProfile();
            shooter.stopAll();
            shooter.publishActiveReadiness(
                    shooter.getReadinessDiagnosticsNow(),
                    ShooterConstants.scoreShooterRpmTolerance(),
                    "SCORE");
        }

        ReadinessDiagnostics readiness = solution.valid()
                ? shooter.getReadinessDiagnosticsNow(solution.shooterRpmTolerance())
                : shooter.getReadinessDiagnosticsNow();
        if (solution.valid()) {
            shooter.publishActiveReadiness(
                    readiness,
                    solution.shooterRpmTolerance(),
                    readinessLabelFor(solution));
        }
        boolean aimReady = aimReadySupplier.getAsBoolean();
        boolean manualFeedOverride = manualFeedOverrideSupplier.getAsBoolean();
        boolean automaticFeedEnabled = automaticFeedEnabledSupplier.getAsBoolean();
        ShotGate.GateDecision gateDecision = shotGate.update(
                solution.valid(),
                solution.movingShot(),
                readiness.atSetpoint(),
                aimReady,
                manualFeedOverride,
                automaticFeedEnabled);

        applyFeedOutputs(gateDecision.gateOpen());
        Logger.recordOutput("Shooting/ShooterAimOnly", false);

        logShootingState(
                solution,
                readiness.atSetpoint(),
                aimReady,
                manualFeedOverride,
                automaticFeedEnabled,
                gateDecision);
    }

    private ShotSolution createDistanceOnlySolution(double distanceMeters) {
        if (!Double.isFinite(distanceMeters)) {
            return ShotSolution.invalid(new Pose2d());
        }
        Shooter.ShotSetpoint setpoint = shooter.calculateSetpointForDistance(distanceMeters);
        return new ShotSolution(
                true,
                ShotIntent.SCORE,
                false,
                new Pose2d(),
                distanceMeters,
                null,
                null,
                0.0,
                AutoAimHeadingConfig.aimToleranceRad(),
                AutoAimHeadingConfig.aimReleaseToleranceRad(),
                ShooterConstants.scoreShooterRpmTolerance(),
                setpoint);
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
            ShotSolution solution,
            boolean shooterAtSetpoint,
            boolean aimReady,
            boolean manualFeedOverride,
            boolean automaticFeedEnabled,
            ShotGate.GateDecision gateDecision) {
        Logger.recordOutput("Shooting/DistanceMeters", solution.distanceMeters());
        Logger.recordOutput("Shooting/DistanceValid", solution.valid());
        Logger.recordOutput("Shooting/FeedGateMode", feedGateMode.name());
        Logger.recordOutput("Shooting/FeedGatePolicy", "UnifiedShotSolution");
        Logger.recordOutput("Shooting/Intent", solution.intent().name());
        Logger.recordOutput("Shooting/AimToleranceRad", solution.headingToleranceRad());
        Logger.recordOutput("Shooting/ShooterRpmTolerance", solution.shooterRpmTolerance());
        Logger.recordOutput("Shooting/ShooterAtSetpoint", shooterAtSetpoint);
        Logger.recordOutput("Shooting/AimReady", aimReady);
        Logger.recordOutput("Shooting/ManualFeedOverride", manualFeedOverride);
        Logger.recordOutput("Shooting/AutomaticFeedEnabled", automaticFeedEnabled);
        Logger.recordOutput("Shooting/GateOpen", gateDecision.gateOpen());
        Logger.recordOutput("Shooting/BlockReason", gateDecision.blockReason());
        Logger.recordOutput("Shooting/ReadyStableCycles", shotGate.getReadyStableCycles());
        Logger.recordOutput("Shooting/ReadyDropStableCycles", shotGate.getReadyDropStableCycles());

        String state;
        if (manualFeedOverride && gateDecision.gateOpen()) {
            state = "MANUAL_FEED_OVERRIDE";
        } else if (!automaticFeedEnabled) {
            state = "AIM_ONLY_OVERRIDE";
        } else if (!solution.valid()) {
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
        stopCoordinatorOutputsPreservingShooterTargets();
    }

    private void stopAimOutputs() {
        stopCoordinatorOutputsPreservingShooterTargets();
    }

    private void stopCoordinatorOutputsPreservingShooterTargets() {
        shotGate.reset();
        activelyFeeding = false;
        shooter.stopKicker();
        shooter.resetActiveReadinessProfile();
        transfer.stopAll();

        Logger.recordOutput("Shooting/ShooterAimOnly", false);
        Logger.recordOutput("Shooting/ShooterAtSetpoint", false);
        Logger.recordOutput("Shooting/AimReady", false);
        Logger.recordOutput("Shooting/ManualFeedOverride", false);
        Logger.recordOutput("Shooting/AutomaticFeedEnabled", false);
        Logger.recordOutput("Shooting/GateOpen", false);
        Logger.recordOutput("Shooting/BlockReason", "CommandEnded");
        Logger.recordOutput("Shooting/ReadyStableCycles", 0);
        Logger.recordOutput("Shooting/ReadyDropStableCycles", 0);
        Logger.recordOutput("Shooting/State", "IDLE");
    }

    public boolean isActivelyFeeding() {
        return activelyFeeding;
    }

    private static String readinessLabelFor(ShotSolution solution) {
        if (solution == null || solution.intent() == ShotIntent.NONE) {
            return "SCORE";
        }
        if (solution.intent() == ShotIntent.PASS) {
            return "PASS";
        }
        return solution.movingShot() ? "MOVING_SCORE" : "SCORE";
    }
}
