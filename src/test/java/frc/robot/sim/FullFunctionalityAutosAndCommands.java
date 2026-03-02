package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.AutoRoutines;
import frc.robot.coordination.shooting.ShootCoordinator;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;

final class FullFunctionalityAutosAndCommands {
    private static final List<String> ALL_AUTO_NAMED_COMMANDS = List.of(
            "DriveStopWithX",
            "IntakeExtend",
            "IntakeRetract",
            "IntakeHome",
            "IntakeSlowRetract",
            "IntakeRoller",
            "IntakeStop",
            "IntakeStopAndRetract",
            "TransferRun",
            "TransferReverse",
            "TransferStop",
            "ShooterAimHub",
            "ShooterShootHub",
            "ShooterShootHubOnMove",
            "ShooterHome",
            "ShooterStop");

    @Test
    void allPathPlannerAutosMatchGoldenMetricsAndExpectedPathSequence() throws IOException {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setAutonomousEnabled();
            context.runCycles(5);

            List<String> autoRoutineNames = AutoRoutines.create(context.drive).stream()
                    .map(AutoRoutines.AutoRoutine::name)
                    .filter(name -> name.startsWith("pp/"))
                    .sorted()
                    .toList();
            assertTrue(!autoRoutineNames.isEmpty(), "Expected at least one PathPlanner auto routine.");

            Set<String> discoveredAutoNames = autoRoutineNames.stream()
                    .map(name -> name.substring("pp/".length()))
                    .collect(java.util.stream.Collectors.toSet());
            FullFunctionalityAutoGoldens.assertExpectedAutoSet(discoveredAutoNames);

            Path deployAutosDir = Filesystem.getDeployDirectory().toPath().resolve("pathplanner").resolve("autos");
            List<String> failures = new ArrayList<>();

            for (String routineName : autoRoutineNames) {
                String autoName = routineName.substring("pp/".length());
                context.clearControllerState();
                context.drive.setPose(edu.wpi.first.math.geometry.Pose2d.kZero);
                CommandScheduler.getInstance().cancelAll();
                context.runCycles(8);

                Path autoFile = deployAutosDir.resolve(autoName + ".auto");
                Set<String> declaredNamedCommands = FullFunctionalityHarness.parseDeclaredNamedCommands(autoFile);
                List<String> declaredPaths = FullFunctionalityHarness.parseDeclaredPathNames(autoFile);

                String wrapperName = "FullFunctionality_PathPlanner_" + autoName;
                Command auto = new PathPlannerAuto(autoName).withName(wrapperName);
                FullFunctionalityHarness.CommandCounts wrapperBefore = context.recorder.getCounts(wrapperName);

                CommandScheduler.getInstance().schedule(auto);
                edu.wpi.first.math.geometry.Pose2d previousPose = context.drive.getPose();
                double distanceMeters = 0.0;
                boolean shooterKickerEverActive = false;
                boolean transferEverMoved = false;
                boolean intakeEverExtended = false;
                boolean finished = false;
                int completedCycles = 0;

                List<String> observedPathSequence = new ArrayList<>();
                String activePath = "";

                for (int i = 0; i < 4500; i++) {
                    context.runCycles(1);
                    completedCycles = i + 1;

                    String currentPath = PathPlannerAuto.currentPathName == null ? "" : PathPlannerAuto.currentPathName;
                    if (!currentPath.equals(activePath)) {
                        activePath = currentPath;
                        if (!activePath.isEmpty()) {
                            observedPathSequence.add(activePath);
                        }
                    }

                    edu.wpi.first.math.geometry.Pose2d currentPose = context.drive.getPose();
                    distanceMeters += currentPose.getTranslation().getDistance(previousPose.getTranslation());
                    previousPose = currentPose;
                    shooterKickerEverActive |= context.shooter.isKickerActive();
                    transferEverMoved |= Math.abs(context.transferInputs.appliedVolts) > 0.5;
                    intakeEverExtended |= context.intake.isExtended();

                    if (!CommandScheduler.getInstance().isScheduled(auto)) {
                        finished = true;
                        break;
                    }
                }
                if (!finished) {
                    CommandScheduler.getInstance().cancel(auto);
                    context.runCycles(10);
                }

                FullFunctionalityHarness.CommandCounts wrapperDelta =
                        context.recorder.getCounts(wrapperName).minus(wrapperBefore);
                double elapsedSec = completedCycles * FullFunctionalityHarness.LOOP_PERIOD_SEC;
                edu.wpi.first.math.geometry.Pose2d finalPose = context.drive.getPose();
                ChassisSpeeds finalSpeeds = context.drive.getMeasuredChassisSpeeds();
                double finalLinearSpeed = Math.hypot(finalSpeeds.vxMetersPerSecond, finalSpeeds.vyMetersPerSecond);

                if (!finished) {
                    failures.add(FullFunctionalityHarness.formatExpectedVsActual(
                            "PathPlanner auto should finish within 90s sim time",
                            String.format(Locale.US, "%s finished=true", autoName),
                            String.format(Locale.US, "%s finished=false elapsedSec=%.2f", autoName, elapsedSec)));
                }
                if (!declaredPaths.equals(observedPathSequence)) {
                    failures.add(FullFunctionalityHarness.formatExpectedVsActual(
                            "Observed auto path sequence should match .auto file declaration",
                            declaredPaths,
                            observedPathSequence));
                }
                if (wrapperDelta.starts() != 1 || wrapperDelta.finishes() + wrapperDelta.interrupts() != 1 || wrapperDelta.interrupts() != 0) {
                    failures.add(FullFunctionalityHarness.formatExpectedVsActual(
                            "Top-level auto wrapper should run exactly once and finish cleanly",
                            "starts=1 finishes=1 interrupts=0",
                            String.format(Locale.US, "%s %s", autoName, wrapperDelta)));
                }
                if (finalLinearSpeed > 0.35 || Math.abs(finalSpeeds.omegaRadiansPerSecond) > 0.7) {
                    failures.add(FullFunctionalityHarness.formatExpectedVsActual(
                            "Auto should settle near stationary chassis speeds at completion",
                            "linear<=0.35 m/s and |omega|<=0.7 rad/s",
                            String.format(
                                    Locale.US,
                                    "auto=%s linear=%.3f omega=%.3f",
                                    autoName,
                                    finalLinearSpeed,
                                    finalSpeeds.omegaRadiansPerSecond)));
                }
                for (String declaredName : declaredNamedCommands) {
                    if (!NamedCommands.hasCommand(declaredName)) {
                        failures.add(FullFunctionalityHarness.formatExpectedVsActual(
                                "Declared named command should be registered",
                                declaredName + " present in NamedCommands",
                                declaredName + " missing in NamedCommands"));
                    }
                }

                try {
                    FullFunctionalityAutoGoldens.assertGolden(
                            autoName,
                            elapsedSec,
                            distanceMeters,
                            finalPose.getX(),
                            finalPose.getY(),
                            finalPose.getRotation().getDegrees(),
                            shooterKickerEverActive,
                            transferEverMoved,
                            intakeEverExtended);
                } catch (AssertionError assertionError) {
                    failures.add("auto=" + autoName + " " + assertionError.getMessage());
                }
            }

            assertTrue(
                    failures.isEmpty(),
                    "PathPlanner auto golden-metric mismatches:\n" + String.join("\n", failures));
        }
    }

    @Test
    void syntheticFullFunctionalityAutoRunsToCompletion() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setAutonomousEnabled();
            context.drive.setPose(edu.wpi.first.math.geometry.Pose2d.kZero);
            context.runCycles(5);

            Command syntheticAuto = Commands.sequence(
                    FullFunctionalityHarness.namedCommand("ShooterHome"),
                    FullFunctionalityHarness.namedCommand("IntakeHome"),
                    Commands.deadline(
                            new PathPlannerAuto("right sweeper"),
                            FullFunctionalityHarness.namedCommand("ShooterShootHubOnMove").withTimeout(2.5)),
                    FullFunctionalityHarness.namedCommand("ShooterStop"),
                    FullFunctionalityHarness.namedCommand("TransferStop"),
                    FullFunctionalityHarness.namedCommand("DriveStopWithX"))
                    .withName("FullFunctionalitySyntheticAuto");

            FullFunctionalityHarness.CommandCounts syntheticBefore =
                    context.recorder.getCounts("FullFunctionalitySyntheticAuto");
            CommandScheduler.getInstance().schedule(syntheticAuto);
            boolean finished = context.runUntil(() -> !CommandScheduler.getInstance().isScheduled(syntheticAuto), 3000);
            if (!finished) {
                CommandScheduler.getInstance().cancel(syntheticAuto);
                context.runCycles(10);
            }

            FullFunctionalityHarness.CommandCounts syntheticDelta =
                    context.recorder.getCounts("FullFunctionalitySyntheticAuto").minus(syntheticBefore);
            assertTrue(
                    finished,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Synthetic auto should complete",
                            "finished=true",
                            String.format(Locale.US, "finished=%s delta=%s", finished, syntheticDelta)));
            assertEquals(
                    1,
                    syntheticDelta.starts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Synthetic auto should start once", "starts=1", syntheticDelta));
            assertEquals(
                    1,
                    syntheticDelta.finishes(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Synthetic auto should finish once", "finishes=1", syntheticDelta));
            assertEquals(
                    0,
                    syntheticDelta.interrupts(),
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Synthetic auto should not be interrupted", "interrupts=0", syntheticDelta));
        }
    }

    @Test
    void allNamedAutoCommandsExecuteWithCorrectLifecycleAndBehavior() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setAutonomousEnabled();
            context.runCycles(10);

            Set<String> continuousNamedCommands = Set.of(
                    "IntakeRoller",
                    "TransferRun",
                    "TransferReverse",
                    "ShooterAimHub",
                    "ShooterShootHub",
                    "ShooterShootHubOnMove");

            List<String> failures = new ArrayList<>();
            for (String named : ALL_AUTO_NAMED_COMMANDS) {
                context.intake.setExtended(false);
                context.transfer.stopAll();
                context.shooter.stopAll();
                context.runCycles(12);

                if (named.equals("IntakeRetract")
                        || named.equals("IntakeHome")
                        || named.equals("IntakeSlowRetract")
                        || named.equals("IntakeStopAndRetract")) {
                    context.intake.setExtended(true);
                    context.runCycles(100);
                }
                if (named.equals("ShooterHome")) {
                    context.shooter.setTargets(2200.0, 2200.0, Units.degreesToRadians(70.0));
                    context.runCycles(100);
                }

                String lifecycleName = "FullFunctionalityNamed_" + named;
                Command command = FullFunctionalityHarness.namedCommand(named).withName(lifecycleName);
                FullFunctionalityHarness.CommandCounts before = context.recorder.getCounts(lifecycleName);

                boolean kickerEverActive = false;
                double maxTransferAbsVolts = 0.0;
                double minTransferVolts = Double.POSITIVE_INFINITY;
                double maxTransferVolts = Double.NEGATIVE_INFINITY;
                double maxRollerAbsVolts = 0.0;
                double maxShooterTargetRpm = 0.0;
                boolean sawLeftNegativeIntakeVolts = false;
                boolean sawRightNegativeIntakeVolts = false;
                boolean sawLeftHomingCurrentThreshold = false;
                boolean sawRightHomingCurrentThreshold = false;
                boolean sawNegativeHoodVolts = false;
                boolean sawHoodHomingCurrentThreshold = false;

                CommandScheduler.getInstance().schedule(command);
                if (continuousNamedCommands.contains(named)) {
                    for (int i = 0; i < 120; i++) {
                        context.runCycles(1);
                        kickerEverActive |= context.shooter.isKickerActive();
                        maxTransferAbsVolts = Math.max(maxTransferAbsVolts, Math.abs(context.transferInputs.appliedVolts));
                        minTransferVolts = Math.min(minTransferVolts, context.transferInputs.appliedVolts);
                        maxTransferVolts = Math.max(maxTransferVolts, context.transferInputs.appliedVolts);
                        maxRollerAbsVolts = Math.max(maxRollerAbsVolts, Math.abs(context.intakeInputs.rollerAppliedVolts));
                        maxShooterTargetRpm = Math.max(maxShooterTargetRpm, Math.abs(context.shooter.getTargetAverageShooterRpm()));
                    }
                    CommandScheduler.getInstance().cancel(command);
                    context.runCycles(8);
                } else {
                    boolean finished = false;
                    for (int i = 0; i < 700; i++) {
                        context.runCycles(1);
                        kickerEverActive |= context.shooter.isKickerActive();
                        maxTransferAbsVolts = Math.max(maxTransferAbsVolts, Math.abs(context.transferInputs.appliedVolts));
                        minTransferVolts = Math.min(minTransferVolts, context.transferInputs.appliedVolts);
                        maxTransferVolts = Math.max(maxTransferVolts, context.transferInputs.appliedVolts);
                        maxRollerAbsVolts = Math.max(maxRollerAbsVolts, Math.abs(context.intakeInputs.rollerAppliedVolts));
                        maxShooterTargetRpm = Math.max(maxShooterTargetRpm, Math.abs(context.shooter.getTargetAverageShooterRpm()));

                        sawLeftNegativeIntakeVolts |= context.intakeInputs.leftAppliedVolts < -0.5;
                        sawRightNegativeIntakeVolts |= context.intakeInputs.rightAppliedVolts < -0.5;
                        sawLeftHomingCurrentThreshold |=
                                Math.abs(context.intakeInputs.leftStatorCurrentAmps) > IntakeConstants.HOMING_CURRENT_THRESHOLD_AMPS;
                        sawRightHomingCurrentThreshold |=
                                Math.abs(context.intakeInputs.rightStatorCurrentAmps) > IntakeConstants.HOMING_CURRENT_THRESHOLD_AMPS;
                        sawNegativeHoodVolts |= context.shooterInputs.hoodAppliedVolts < -8.0;
                        sawHoodHomingCurrentThreshold |=
                                Math.abs(context.shooterInputs.hoodStatorCurrentAmps) > ShooterConstants.HOMING_CURRENT_THRESHOLD_AMPS;

                        if (!CommandScheduler.getInstance().isScheduled(command)) {
                            finished = true;
                            break;
                        }
                    }
                    if (!finished) {
                        CommandScheduler.getInstance().cancel(command);
                        context.runCycles(8);
                        failures.add(FullFunctionalityHarness.formatExpectedVsActual(
                                "Finite named command should finish without forced cancel",
                                lifecycleName + " finishes naturally",
                                lifecycleName + " required cancel"));
                    }
                }

                FullFunctionalityHarness.CommandCounts delta = context.recorder.getCounts(lifecycleName).minus(before);
                if (delta.starts() == 0) {
                    failures.add(FullFunctionalityHarness.formatExpectedVsActual(
                            "Named command should start when directly scheduled",
                            lifecycleName + " starts>=1",
                            lifecycleName + " starts=0"));
                }
                if (continuousNamedCommands.contains(named)) {
                    if (delta.interrupts() == 0) {
                        failures.add(FullFunctionalityHarness.formatExpectedVsActual(
                                "Continuous named command should be interrupted when canceled",
                                lifecycleName + " interrupts>=1",
                                lifecycleName + " " + delta));
                    }
                } else if (delta.finishes() == 0) {
                    failures.add(FullFunctionalityHarness.formatExpectedVsActual(
                            "Finite named command should finish",
                            lifecycleName + " finishes>=1",
                            lifecycleName + " " + delta));
                }
                if (delta.finishes() + delta.interrupts() != delta.starts()) {
                    failures.add(FullFunctionalityHarness.formatExpectedVsActual(
                            "Named command lifecycle should close",
                            lifecycleName + " ends==starts",
                            lifecycleName + " " + delta));
                }

                switch (named) {
                    case "IntakeExtend" -> {
                        double leftRot = Units.radiansToRotations(context.intakeInputs.leftPositionRad);
                        if (!context.intake.isExtended()) {
                            failures.add("IntakeExtend expected intake.isExtended()=true but was false");
                        }
                        if (Math.abs(leftRot - IntakeConstants.EXTENDED_POSITION_ROT) > 0.6) {
                            failures.add(FullFunctionalityHarness.formatExpectedVsActual(
                                    "IntakeExtend should reach extended encoder target",
                                    IntakeConstants.EXTENDED_POSITION_ROT + "±0.6 rot",
                                    leftRot));
                        }
                    }
                    case "IntakeRetract", "IntakeStopAndRetract", "IntakeHome", "IntakeSlowRetract" -> {
                        if (named.equals("IntakeHome")) {
                            if (!sawLeftNegativeIntakeVolts || !sawRightNegativeIntakeVolts) {
                                failures.add("IntakeHome should apply negative voltage to both sides during homing.");
                            }
                            if (!sawLeftHomingCurrentThreshold || !sawRightHomingCurrentThreshold) {
                                failures.add("IntakeHome should detect homing current threshold on both sides.");
                            }
                            context.runCycles(160);
                        }
                        double leftRot = Units.radiansToRotations(context.intakeInputs.leftPositionRad);
                        if (context.intake.isExtended()) {
                            failures.add(named + " expected intake.isExtended()=false but was true");
                        }
                        double tolerance = named.equals("IntakeSlowRetract") ? 1.0 : 0.7;
                        if (Math.abs(leftRot - IntakeConstants.RETRACTED_POSITION_ROT) > tolerance) {
                            failures.add(FullFunctionalityHarness.formatExpectedVsActual(
                                    named + " should leave intake near retracted target",
                                    IntakeConstants.RETRACTED_POSITION_ROT + "±" + tolerance + " rot",
                                    leftRot));
                        }
                    }
                    case "IntakeRoller" -> {
                        if (maxRollerAbsVolts <= 1.0) {
                            failures.add(FullFunctionalityHarness.formatExpectedVsActual(
                                    "IntakeRoller should drive roller motor",
                                    "maxAbsRollerVolts>1.0",
                                    maxRollerAbsVolts));
                        }
                    }
                    case "TransferRun" -> {
                        if (maxTransferVolts <= 1.0) {
                            failures.add(FullFunctionalityHarness.formatExpectedVsActual(
                                    "TransferRun should command positive transfer voltage",
                                    "maxTransferVolts>1.0",
                                    maxTransferVolts));
                        }
                    }
                    case "TransferReverse" -> {
                        if (minTransferVolts >= -1.0) {
                            failures.add(FullFunctionalityHarness.formatExpectedVsActual(
                                    "TransferReverse should command negative transfer voltage",
                                    "minTransferVolts<-1.0",
                                    minTransferVolts));
                        }
                    }
                    case "TransferStop", "IntakeStop", "ShooterStop" -> {
                        if (maxTransferAbsVolts > 0.3 && named.equals("TransferStop")) {
                            failures.add(FullFunctionalityHarness.formatExpectedVsActual(
                                    "TransferStop should keep transfer stationary",
                                    "maxAbsTransferVolts<=0.3",
                                    maxTransferAbsVolts));
                        }
                        if (named.equals("ShooterStop") && Math.abs(context.shooter.getTargetAverageShooterRpm()) > 1e-6) {
                            failures.add(FullFunctionalityHarness.formatExpectedVsActual(
                                    "ShooterStop should zero shooter target",
                                    "targetAverageRpm=0",
                                    context.shooter.getTargetAverageShooterRpm()));
                        }
                    }
                    case "ShooterAimHub" -> {
                        if (maxShooterTargetRpm < 300.0) {
                            failures.add(FullFunctionalityHarness.formatExpectedVsActual(
                                    "ShooterAimHub should spin shooter to nontrivial rpm",
                                    "maxShooterTargetRpm>=300",
                                    maxShooterTargetRpm));
                        }
                        if (kickerEverActive) {
                            failures.add("ShooterAimHub should not activate kicker, but kicker became active.");
                        }
                    }
                    case "ShooterShootHub" -> {
                        if (!kickerEverActive) {
                            failures.add(named + " should activate kicker at least once but never did.");
                        }
                        if (maxTransferAbsVolts < 1.0) {
                            failures.add(named + " should run transfer while shooting but transfer never moved.");
                        }
                    }
                    case "ShooterShootHubOnMove" -> {
                        if (maxShooterTargetRpm < 300.0) {
                            failures.add(FullFunctionalityHarness.formatExpectedVsActual(
                                    "ShooterShootHubOnMove should at least spin shooter targets",
                                    "maxShooterTargetRpm>=300",
                                    maxShooterTargetRpm));
                        }
                    }
                    case "ShooterHome" -> {
                        if (!sawNegativeHoodVolts) {
                            failures.add("ShooterHome should apply negative hood homing voltage.");
                        }
                        if (!sawHoodHomingCurrentThreshold) {
                            failures.add("ShooterHome should detect hood homing current threshold.");
                        }
                        double preAimRad = ShooterConstants.SHOT_MAP_HOOD_ANGLE_DEG.length > 0
                                ? Units.degreesToRadians(ShooterConstants.SHOT_MAP_HOOD_ANGLE_DEG[0])
                                : ShooterConstants.HOOD_MIN_ANGLE_RAD;
                        if (Math.abs(context.shooterInputs.hoodPositionRad - preAimRad) > Units.degreesToRadians(2.0)) {
                            failures.add(FullFunctionalityHarness.formatExpectedVsActual(
                                    "ShooterHome should settle near pre-aim hood angle",
                                    Math.toDegrees(preAimRad) + "deg ±2.0deg",
                                    Math.toDegrees(context.shooterInputs.hoodPositionRad) + "deg"));
                        }
                    }
                    case "DriveStopWithX" -> {
                        ChassisSpeeds speeds = context.drive.getMeasuredChassisSpeeds();
                        double linearNorm = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
                        if (linearNorm > 0.2 || Math.abs(speeds.omegaRadiansPerSecond) > 0.4) {
                            failures.add(FullFunctionalityHarness.formatExpectedVsActual(
                                    "DriveStopWithX should leave chassis near zero velocity",
                                    "linear<0.2 m/s and |omega|<0.4 rad/s",
                                    String.format(Locale.US, "linear=%.3f omega=%.3f", linearNorm, speeds.omegaRadiansPerSecond)));
                        }
                    }
                    default -> {
                        // Already covered by lifecycle assertions.
                    }
                }
            }

            assertTrue(
                    failures.isEmpty(),
                    "Named command lifecycle/behavior mismatches:\n" + String.join("\n", failures));
        }
    }

    @Test
    void shootCoordinatorGatingTransitionsBehaveCorrectly() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(20);

            ShootCoordinator shootCoordinator = new ShootCoordinator(context.shooter, context.transfer);
            AtomicReference<Double> distance = new AtomicReference<>(4.0);
            Command shoot = shootCoordinator
                    .shootForDistance(() -> distance.get(), () -> true)
                    .withName("FF_ShootCoordinatorGating");

            CommandScheduler.getInstance().schedule(shoot);
            boolean everAtSetpoint = false;
            int gateOpenCycle = -1;
            int readyCycle = -1;
            for (int i = 0; i < 500; i++) {
                context.runCycles(1);
                boolean readyNow = context.shooter.readyToFire();
                if (readyNow) {
                    everAtSetpoint = true;
                    if (readyCycle < 0) {
                        readyCycle = i;
                    }
                }
                if (Math.abs(context.transferInputs.appliedVolts) > 1.0 && gateOpenCycle < 0) {
                    gateOpenCycle = i;
                }
            }

            assertTrue(everAtSetpoint, "ShootCoordinator should eventually report shooter readyToFire in sim.");
            assertTrue(gateOpenCycle >= 0, "ShootCoordinator should eventually open feed gate / run transfer.");
            assertTrue(
                    gateOpenCycle >= readyCycle,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Feed gate should not open before shooter is ready",
                            "gateOpenCycle >= readyCycle",
                            String.format(Locale.US, "gateOpenCycle=%d readyCycle=%d", gateOpenCycle, readyCycle)));

            distance.set(Double.NaN);
            context.runCycles(2);
            assertTrue(
                    Math.abs(context.transferInputs.appliedVolts) < 1e-6 && !context.shooter.isKickerActive(),
                    "Invalid distance should immediately close feed gate and deactivate kicker.");

            CommandScheduler.getInstance().cancel(shoot);
            context.runCycles(4);
            assertTrue(
                    Math.abs(context.transferInputs.appliedVolts) < 1e-6 && !context.shooter.isKickerActive(),
                    "Canceling ShootCoordinator command should stop transfer and deactivate kicker.");
        }
    }

    @Test
    void dashboardPublishedCommandsExecuteWithExpectedLifecycle() {
        try (FullFunctionalityHarness.Context context = new FullFunctionalityHarness.Context(false)) {
            context.setTeleopEnabled();
            context.container.teleopInit();
            context.runCycles(20);

            runDashboardOneShot(context, "Actions/IntakeHome", "DashboardIntakeHome");
            runDashboardOneShot(context, "Actions/ShooterHome", "DashboardShooterHome");
            runDashboardOneShot(context, "Actions/StopManipulators", "DashboardStopManipulators");
            runDashboardOneShot(context, "Actions/ShooterStop", "DashboardShooterStop");
            runDashboardOneShot(context, "Actions/DriveStopWithX", "DashboardDriveStopWithX");
            runDashboardOneShot(
                    context,
                    "Actions/DriveResetOdometryAndHeading",
                    "DashboardDriveResetOdometryAndHeading");
            runDashboardOneShot(
                    context,
                    "Actions/DriveSetOdometryFromUnifiedVision",
                    "DashboardDriveSetOdometryFromUnifiedVision");

            SmartDashboard.putBoolean("Overrides/OverrideAutoAim", true);
            SmartDashboard.putNumber("Overrides/AimDistanceMeters", 2.6);
            SmartDashboard.putBoolean("Overrides/DisableVision", true);
            context.runCycles(6);
            runDashboardOneShot(context, "Overrides/ClearAll", "DashboardClearAllOverrides");
            assertTrue(
                    !SmartDashboard.getBoolean("Overrides/OverrideAutoAim", true),
                    "Expected dashboard clear-all command to reset OverrideAutoAim=false.");
            assertTrue(
                    !SmartDashboard.getBoolean("Overrides/DisableVision", true),
                    "Expected dashboard clear-all command to reset DisableVision=false.");

            Command dashboardTransfer = FullFunctionalityHarness.getDashboardCommand("Actions/TransferRun");
            FullFunctionalityHarness.CommandCounts transferBefore =
                    context.recorder.getCounts("DashboardTransferRun");
            CommandScheduler.getInstance().schedule(dashboardTransfer);
            context.runCycles(40);
            assertTrue(
                    context.transferInputs.appliedVolts > 1.0,
                    "DashboardTransferRun should drive transfer positive while scheduled.");
            CommandScheduler.getInstance().cancel(dashboardTransfer);
            context.runCycles(8);
            FullFunctionalityHarness.CommandCounts transferDelta =
                    context.recorder.getCounts("DashboardTransferRun").minus(transferBefore);
            assertTrue(
                    transferDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "DashboardTransferRun should start when scheduled", "starts>=1", transferDelta));
            assertTrue(
                    transferDelta.interrupts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "Canceling DashboardTransferRun should interrupt it",
                            "interrupts>=1",
                            transferDelta));

            SmartDashboard.putBoolean("Shooter/SysId/Enabled", false);
            context.runCycles(6);
            runDashboardOneShot(context, "Shooter/SysId/QuasistaticForward", "ShooterSysIdQuasistaticForward");
            runDashboardOneShot(context, "Shooter/SysId/QuasistaticReverse", "ShooterSysIdQuasistaticReverse");
            runDashboardOneShot(context, "Shooter/SysId/DynamicForward", "ShooterSysIdDynamicForward");
            runDashboardOneShot(context, "Shooter/SysId/DynamicReverse", "ShooterSysIdDynamicReverse");

            SmartDashboard.putBoolean("Shooter/SysId/Enabled", true);
            context.runCycles(6);
            Command quasistaticForward =
                    FullFunctionalityHarness.getDashboardCommand("Shooter/SysId/QuasistaticForward");
            FullFunctionalityHarness.CommandCounts sysIdBefore =
                    context.recorder.getCounts("ShooterSysIdQuasistaticForward");
            CommandScheduler.getInstance().schedule(quasistaticForward);
            context.runCycles(80);
            if (CommandScheduler.getInstance().isScheduled(quasistaticForward)) {
                CommandScheduler.getInstance().cancel(quasistaticForward);
                context.runCycles(8);
            }
            FullFunctionalityHarness.CommandCounts sysIdDelta =
                    context.recorder.getCounts("ShooterSysIdQuasistaticForward").minus(sysIdBefore);
            assertTrue(
                    sysIdDelta.starts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "ShooterSysIdQuasistaticForward should start when enabled",
                            "starts>=1",
                            sysIdDelta));
            assertTrue(
                    sysIdDelta.finishes() + sysIdDelta.interrupts() >= 1,
                    FullFunctionalityHarness.formatExpectedVsActual(
                            "ShooterSysIdQuasistaticForward should end via finish or interrupt",
                            "finishes+interrupts>=1",
                            sysIdDelta));
        }
    }

    private static void runDashboardOneShot(
            FullFunctionalityHarness.Context context,
            String key,
            String lifecycleName) {
        Command command = FullFunctionalityHarness.getDashboardCommand(key);
        FullFunctionalityHarness.CommandCounts before = context.recorder.getCounts(lifecycleName);
        CommandScheduler.getInstance().schedule(command);
        boolean finished = context.runUntil(() -> !CommandScheduler.getInstance().isScheduled(command), 700);
        if (!finished) {
            CommandScheduler.getInstance().cancel(command);
            context.runCycles(8);
        }
        FullFunctionalityHarness.CommandCounts delta = context.recorder.getCounts(lifecycleName).minus(before);
        assertTrue(
                delta.starts() >= 1,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Dashboard command should start",
                        lifecycleName + " starts>=1",
                        delta));
        assertTrue(
                delta.finishes() >= 1,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Dashboard one-shot command should finish",
                        lifecycleName + " finishes>=1",
                        delta));
        assertEquals(
                0,
                delta.interrupts(),
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Dashboard one-shot command should not be interrupted",
                        lifecycleName + " interrupts=0",
                        delta));
    }
}
