package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.MathUtil;
import java.util.Map;
import java.util.Set;

final class FullFunctionalityAutoGoldens {
    private FullFunctionalityAutoGoldens() {}

    private static final double ELAPSED_TOLERANCE_SEC = 0.05;
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
                    new AutoGolden(8.020, 13.537406, 8.000063, 5.012921, 22.234247, true, true, false)),
            Map.entry(
                    "brag",
                    new AutoGolden(7.000, 5.309856, 2.576406, 6.392854, 128.309389, true, true, false)),
            Map.entry(
                    "left mid cycle",
                    new AutoGolden(18.880, 12.490841, 3.014197, 6.755853, 117.788933, true, true, true)),
            Map.entry(
                    "left mid rush cycle",
                    new AutoGolden(20.820, 12.449354, 3.005534, 6.755621, 120.507662, true, true, true)),
            Map.entry(
                    "left rush event auto",
                    new AutoGolden(17.960, 13.732693, 2.704229, 6.359771, 129.414236, true, true, false)),
            Map.entry(
                    "left sweeper",
                    new AutoGolden(5.420, 9.457444, 8.076133, 1.999514, -33.685364, false, false, false)),
            Map.entry(
                    "mid back up then score",
                    new AutoGolden(11.620, 1.039678, 2.555260, 3.987686, -179.999729, true, true, false)),
            Map.entry(
                    "right mid cycle",
                    new AutoGolden(19.080, 12.115259, 3.014197, 1.314147, -121.956178, true, true, true)),
            Map.entry(
                    "right mid rush cycle",
                    new AutoGolden(20.620, 12.070692, 3.005534, 1.314379, -120.507662, true, true, true)),
            Map.entry(
                    "right rush event auto",
                    new AutoGolden(17.960, 13.732693, 2.704229, 1.710229, -129.414236, true, true, false)),
            Map.entry(
                    "right sweeper",
                    new AutoGolden(5.420, 9.457444, 8.076133, 6.070486, 33.685364, false, false, false)));

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
