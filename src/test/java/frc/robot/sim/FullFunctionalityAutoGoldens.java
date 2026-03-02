package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Map;
import java.util.Set;

final class FullFunctionalityAutoGoldens {
    private FullFunctionalityAutoGoldens() {}

    private static final double ELAPSED_TOLERANCE_SEC = 0.05;
    private static final double DISTANCE_TOLERANCE_M = 0.05;
    private static final double POSE_TOLERANCE_M = 0.04;
    private static final double HEADING_TOLERANCE_DEG = 1.0;

    record AutoGolden(
            double elapsedSec,
            double distanceMeters,
            double finalX,
            double finalY,
            double finalHeadingDeg,
            boolean shooterKickerEverActive,
            boolean transferEverMoved,
            boolean intakeEverExtended) {}

    static final Map<String, AutoGolden> BY_NAME = Map.of(
            "Along Alliance Moving Shot",
                    new AutoGolden(9.740, 13.527169, 7.999617, 5.016771, 26.487923, true, true, false),
            "brag",
                    new AutoGolden(7.260, 5.297118, 2.577218, 6.393992, 131.395182, true, true, false),
            "mid back up then score",
                    new AutoGolden(11.900, 1.046438, 2.507755, 2.060382, -141.028122, true, true, false),
            "right mid cycle",
                    new AutoGolden(20.620, 12.411352, 3.010898, 1.311304, -125.421394, true, true, true),
            "right mid rush cycle",
                    new AutoGolden(22.540, 12.403353, 3.010899, 1.311305, -125.356403, true, true, true),
            "right rush event auto",
                    new AutoGolden(19.600, 13.697500, 2.709103, 1.716465, -133.948806, true, true, false),
            "right sweeper",
                    new AutoGolden(6.820, 9.442767, 8.076786, 6.064262, 33.505692, false, false, false));

    static void assertExpectedAutoSet(Set<String> autoNames) {
        assertEquals(
                BY_NAME.keySet(),
                autoNames,
                "PathPlanner auto set changed. Update FullFunctionalityAutoGoldens for new/removed autos.");
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
                Math.abs(finalHeadingDeg - golden.finalHeadingDeg()) <= HEADING_TOLERANCE_DEG,
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
