package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class IntakeStateRecoveryTest {
    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setTest(false);
        DriverStationSim.setEStop(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
    }

    @AfterEach
    void tearDown() {
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    @Test
    void stopAllAllowsExtensionStateToRecoverFromSensors() {
        Intake intake = new Intake(new IntakeIOSim());

        intake.setExtended(true);
        runPeriodicCycles(intake, 250);
        assertTrue(intake.isExtended(), "Precondition failed: intake should reach extended state in sim.");

        intake.stopAll();
        runPeriodicCycles(intake, 5);

        assertTrue(
                intake.isExtended(),
                "Expected intake extension state to recover from sensors after stopAll().");
    }

    private static void runPeriodicCycles(Intake intake, int cycles) {
        for (int i = 0; i < cycles; i++) {
            DriverStationSim.notifyNewData();
            intake.periodic();
        }
    }
}
