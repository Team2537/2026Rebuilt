package frc.robot.autos;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.coordination.shooting.ShootCoordinator;
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
    private static final String AUTO_NAME = "right rush hp";

    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        AutoBuilder.resetForTesting();
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
        AutoBuilder.resetForTesting();
        SimHooks.resumeTiming();
    }

    @Test
    void namedPathPlannerAutoRunsToCompletionInSim() {
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
        ShootCoordinator shootCoordinator = new ShootCoordinator(shooter, transfer);

        frc.robot.RobotState.initialize(drive);
        AutoNamedCommands.registerAll(drive, shooter, transfer, intake, shootCoordinator);
        assertTrue(
                AutoRoutines.create(drive).stream()
                        .map(AutoRoutines.AutoRoutine::name)
                        .collect(Collectors.toSet())
                        .contains("pp/" + AUTO_NAME),
                "Auto selector routines did not include expected PathPlanner auto: " + AUTO_NAME);

        Command auto = new PathPlannerAuto(AUTO_NAME).withName("TestAuto_" + AUTO_NAME);
        CommandScheduler.getInstance().schedule(auto);

        double distanceTraveledMeters = 0.0;
        Pose2d previousPose = drive.getPose();
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
            shooterKickerEverActive |= shooter.isKickerActive();

            if (!CommandScheduler.getInstance().isScheduled(auto)) {
                finished = true;
                iterationsUntilFinish = i + 1;
                break;
            }
        }

        assertTrue(finished, "PathPlanner auto did not finish in simulation.");
        double elapsedSeconds = iterationsUntilFinish * 0.02;
        assertTrue(elapsedSeconds > 4.0,
                "Auto finished too quickly for a real path-following routine. elapsedSeconds=" + elapsedSeconds);
        assertTrue(distanceTraveledMeters > 4.0,
                "Drive did not appear to follow paths. distanceTraveledMeters=" + distanceTraveledMeters);
        assertTrue(shooterKickerEverActive,
                "Shooter kicker never became active, shoot cycle may not have fed any game piece.");
        assertTrue(previousPose.getTranslation().getNorm() > 1.0,
                "Final pose should have moved meaningfully from the origin. finalPose=" + previousPose);
    }

    @Test
    void rightRushEventAutoExtendsIntakeDuringPath() {
        assertRushEventAutoExtendsIntakeDuringPath("right rush event auto");
    }

    @Test
    void leftRushEventAutoExtendsIntakeDuringPath() {
        assertRushEventAutoExtendsIntakeDuringPath("left rush event auto");
    }

    private static void assertRushEventAutoExtendsIntakeDuringPath(String autoName) {
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
        ShootCoordinator shootCoordinator = new ShootCoordinator(shooter, transfer);

        frc.robot.RobotState.initialize(drive);
        AutoNamedCommands.registerAll(drive, shooter, transfer, intake, shootCoordinator);

        Command auto = new PathPlannerAuto(autoName).withName("TestAuto_" + autoName);
        CommandScheduler.getInstance().schedule(auto);

        boolean intakeEverExtended = intake.isExtended();
        for (int i = 0; i < 2000; i++) {
            SimHooks.stepTiming(0.02);
            DriverStationSim.notifyNewData();
            CommandScheduler.getInstance().run();
            intakeEverExtended |= intake.isExtended();

            if (!CommandScheduler.getInstance().isScheduled(auto) && intakeEverExtended) {
                break;
            }
        }

        assertTrue(
                intakeEverExtended,
                autoName + " never extended the intake. The path's intake event marker may not be bound.");
    }
}
