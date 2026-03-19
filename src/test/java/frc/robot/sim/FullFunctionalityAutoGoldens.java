package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.MathUtil;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

final class FullFunctionalityAutoGoldens {
    private FullFunctionalityAutoGoldens() {}

    private static final double ELAPSED_TOLERANCE_SEC = 0.60;
    private static final double DISTANCE_TOLERANCE_M = 0.06;
    private static final double POSE_TOLERANCE_M = 0.05;
    private static final double HEADING_TOLERANCE_DEG = 3.0;

    record AutoGolden(
            double elapsedSec,
            double distanceMeters,
            double finalX,
            double finalY,
            double finalHeadingDeg,
            boolean shooterKickerEverActive,
            boolean transferEverMoved,
            boolean intakeEverExtended) {}

    static final Map<String, AutoGolden> BY_NAME = Map.ofEntries(
            Map.entry(
                    "Along Alliance Moving Shot",
                    new AutoGolden(7.120, 13.537406, 8.000063, 5.012921, 22.234247, true, true, false)));

    static void assertExpectedAutoSet(Set<String> autoNames) {
        Set<String> missingTrackedAutos = new HashSet<>(BY_NAME.keySet());
        missingTrackedAutos.removeAll(autoNames);
        assertEquals(
                Set.of(),
                missingTrackedAutos,
                "Tracked PathPlanner autos are missing. Update FullFunctionalityAutoGoldens if comp autos were removed/renamed.");
    }

    static boolean isTrackedAuto(String autoName) {
        return BY_NAME.containsKey(autoName);
    }

    static boolean isUntrackedCompAuto(String autoName) {
        return !isTrackedAuto(autoName);
    }

    static void assertGolden(
            String autoName,
            double elapsedSec,
            double distanceMeters,
            double finalX,
            double finalY,
            double finalHeadingDeg,
            boolean shooterKickerEverActive,
            boolean transferEverMoved,
            boolean intakeEverExtended) {
        AutoGolden golden = BY_NAME.get(autoName);
        assertTrue(golden != null, "Missing golden metrics for auto: " + autoName);

        assertTrue(
                Math.abs(elapsedSec - golden.elapsedSec()) <= ELAPSED_TOLERANCE_SEC,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Auto elapsed time should match golden",
                        golden.elapsedSec() + "±" + ELAPSED_TOLERANCE_SEC,
                        elapsedSec));
        assertTrue(
                Math.abs(distanceMeters - golden.distanceMeters()) <= DISTANCE_TOLERANCE_M,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Auto travel distance should match golden",
                        golden.distanceMeters() + "±" + DISTANCE_TOLERANCE_M,
                        distanceMeters));
        assertTrue(
                Math.abs(finalX - golden.finalX()) <= POSE_TOLERANCE_M,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Auto final X should match golden",
                        golden.finalX() + "±" + POSE_TOLERANCE_M,
                        finalX));
        assertTrue(
                Math.abs(finalY - golden.finalY()) <= POSE_TOLERANCE_M,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Auto final Y should match golden",
                        golden.finalY() + "±" + POSE_TOLERANCE_M,
                        finalY));
        assertTrue(
                Math.abs(Math.toDegrees(MathUtil.angleModulus(
                        Math.toRadians(finalHeadingDeg - golden.finalHeadingDeg())))) <= HEADING_TOLERANCE_DEG,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Auto final heading should match golden",
                        golden.finalHeadingDeg() + "±" + HEADING_TOLERANCE_DEG,
                        finalHeadingDeg));
        assertEquals(
                golden.shooterKickerEverActive(),
                shooterKickerEverActive,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Shooter kicker usage should match golden",
                        golden.shooterKickerEverActive(),
                        shooterKickerEverActive));
        assertEquals(
                golden.transferEverMoved(),
                transferEverMoved,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Transfer activity should match golden",
                        golden.transferEverMoved(),
                        transferEverMoved));
        assertEquals(
                golden.intakeEverExtended(),
                intakeEverExtended,
                FullFunctionalityHarness.formatExpectedVsActual(
                        "Intake extension activity should match golden",
                        golden.intakeEverExtended(),
                        intakeEverExtended));
    }
}
