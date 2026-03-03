package frc.robot.sim;

import org.junit.platform.suite.api.SelectClasses;
import org.junit.platform.suite.api.Suite;

@Suite
@SelectClasses({
    FullFunctionalityModeAndHoming.class,
    FullFunctionalityBindings.class,
    FullFunctionalityAutosAndCommands.class,
    FullFunctionalityDriveAndAutoStress.class,
    FullFunctionalityIntakeTransferStress.class,
    FullFunctionalitySmartRetractDuringShoot.class,
    FullFunctionalityShooterStress.class,
    FullFunctionalityLifecycleInvariants.class,
    FullFunctionalityFaultInjectionDisconnects.class,
    FullFunctionalityAutoSelectionAndVisionPathologies.class,
    FullFunctionalityChaosMonkeyDivergences.class,
    FullFunctionalityUncoveredPathExploration.class
})
class FullFunctionalityTest {}
