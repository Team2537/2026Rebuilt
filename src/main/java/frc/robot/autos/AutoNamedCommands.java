package frc.robot.autos;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.coordination.shooting.ShootCoordinator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transfer.Transfer;

/** Registers PathPlanner named commands used by autonomous routines. */
public final class AutoNamedCommands {
    private AutoNamedCommands() {}

    public static void registerAll(
            Drive drive,
            Shooter shooter,
            Transfer transfer,
            Intake intake,
            ShootCoordinator shootCoordinator) {
        register(
                "DriveStopWithX",
                Commands.runOnce(drive::stopWithX, drive).withName("AutoNamed_DriveStopWithX"));

        register("IntakeExtend", intake.extendCommand().withName("AutoNamed_IntakeExtend"));
        register("IntakeRetract", intake.retractCommand().withName("AutoNamed_IntakeRetract"));
        register("IntakeHome", intake.homeCommand().withName("AutoNamed_IntakeHome"));
        register("IntakeRoller", intake.spinRoller().withName("AutoNamed_IntakeRoller"));
        register("IntakeStop", intake.stopCommand().withName("AutoNamed_IntakeStop"));
        register(
                "IntakeStopAndRetract",
                intake.stopAndRetractCommand().withName("AutoNamed_IntakeStopAndRetract"));

        register("TransferRun", transfer.runCommand().withName("AutoNamed_TransferRun"));
        register("TransferReverse", transfer.reverseCommand().withName("AutoNamed_TransferReverse"));
        register("TransferStop", transfer.stopCommand().withName("AutoNamed_TransferStop"));

        register(
                "ShooterAimHub",
                AutoCommands.aimForHub(drive, shooter, shootCoordinator).withName("AutoNamed_ShooterAimHub"));
        register(
                "ShooterShootHub",
                AutoCommands.shootHub(drive, shooter, shootCoordinator).withName("AutoNamed_ShooterShootHub"));
        register("ShooterHome", shooter.homeCommand().withName("AutoNamed_ShooterHome"));
        register("ShooterStop", shooter.stopCommand().withName("AutoNamed_ShooterStop"));
    }

    private static void register(String name, Command command) {
        NamedCommands.registerCommand(name, command);
    }
}
