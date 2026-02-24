package frc.robot.autos;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.transfer.TransferIOSim;
import java.util.stream.Collectors;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class PathPlannerNamedAutoSimTest {
    private static final String AUTO_NAME = "Named Three Path Cycle";

    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        SimHooks.pauseTiming();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setTest(false);
        DriverStationSim.setEStop(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        CommandScheduler.getInstance().cancelAll();
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        SimHooks.resumeTiming();
    }

    @Test
    void namedThreePathCycleRunsToCompletionInSim() {
        GyroIOSim gyro = new GyroIOSim(Drive.getModuleTranslations());
        Drive drive = new Drive(
                gyro,
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        gyro.setModulePositionsSupplier(drive::getModulePositionsForSim);

        Shooter shooter = new Shooter(new ShooterIOSim());
        Transfer transfer = new Transfer(new TransferIOSim());
        Intake intake = new Intake(new IntakeIOSim());

        AutoNamedCommands.registerAll(drive, shooter, transfer, intake);
        assertTrue(
                AutoRoutines.create(drive).stream()
                        .map(AutoRoutines.AutoRoutine::name)
                        .collect(Collectors.toSet())
                        .contains("pp/" + AUTO_NAME),
                "Auto selector routines did not include new PathPlanner auto: " + AUTO_NAME);

        Command auto = new PathPlannerAuto(AUTO_NAME).withName("TestAuto_" + AUTO_NAME);
        CommandScheduler.getInstance().schedule(auto);

        double distanceTraveledMeters = 0.0;
        Pose2d previousPose = drive.getPose();
        boolean intakeEverExtended = intake.isExtended();
        boolean shooterKickerEverActive = shooter.isKickerActive();
        int iterationsUntilFinish = -1;
        boolean finished = false;

        for (int i = 0; i < 4500; i++) {
            SimHooks.stepTiming(0.02);
            DriverStationSim.notifyNewData();
            CommandScheduler.getInstance().run();

            Pose2d currentPose = drive.getPose();
            distanceTraveledMeters += currentPose.getTranslation().getDistance(previousPose.getTranslation());
            previousPose = currentPose;
            intakeEverExtended |= intake.isExtended();
            shooterKickerEverActive |= shooter.isKickerActive();

            if (!CommandScheduler.getInstance().isScheduled(auto)) {
                finished = true;
                iterationsUntilFinish = i + 1;
                break;
            }
        }

        assertTrue(finished, "PathPlanner auto did not finish in simulation.");
        double elapsedSeconds = iterationsUntilFinish * 0.02;
        assertTrue(elapsedSeconds > 8.0,
                "Auto finished too quickly for homing + path sequence. elapsedSeconds=" + elapsedSeconds);
        assertTrue(distanceTraveledMeters > 4.0,
                "Drive did not appear to follow paths. distanceTraveledMeters=" + distanceTraveledMeters);
        assertTrue(intakeEverExtended,
                "Intake never became extended, IntakeExtend named command may not have run.");
        assertTrue(!intake.isExtended(),
                "Intake should be retracted at auto end due to IntakeStopAndRetract named command.");
        assertTrue(shooterKickerEverActive,
                "Shooter kicker never became active, shoot cycle may not have fed any game piece.");

        Pose2d expectedEndPose = new Pose2d(2.531965277777778, 5.732934968171296, previousPose.getRotation());
        Translation2d endDelta = previousPose.getTranslation().minus(expectedEndPose.getTranslation());
        assertTrue(
                endDelta.getNorm() < 1.25,
                "Final pose was too far from score path endpoint. finalPose=" + previousPose + " expectedTranslation="
                        + expectedEndPose.getTranslation() + " deltaMeters=" + endDelta.getNorm());
    }
}
