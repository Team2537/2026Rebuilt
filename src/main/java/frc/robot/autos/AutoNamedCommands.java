package frc.robot.autos;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transfer.Transfer;

/** Registers PathPlanner named commands used by autonomous routines. */
public final class AutoNamedCommands {
    private AutoNamedCommands() {}

    public static void registerAll(Drive drive, Shooter shooter, Transfer transfer, Intake intake) {
        // Keep commands primitive and composable so PathPlanner command groups define timing.
        NamedCommands.registerCommand(
                "DriveStopWithX",
                Commands.runOnce(drive::stopWithX, drive).withName("AutoNamed_DriveStopWithX"));

        NamedCommands.registerCommand("IntakeExtend", intake.extendCommand().withName("AutoNamed_IntakeExtend"));
        NamedCommands.registerCommand("IntakeRetract", intake.retractCommand().withName("AutoNamed_IntakeRetract"));
        NamedCommands.registerCommand("IntakeHome", intake.homeCommand().withName("AutoNamed_IntakeHome"));
        NamedCommands.registerCommand("IntakeRoller", intake.spinRoller().withName("AutoNamed_IntakeRoller"));
        NamedCommands.registerCommand("IntakeStop", intake.stopCommand().withName("AutoNamed_IntakeStop"));
        NamedCommands.registerCommand(
                "IntakeStopAndRetract",
                intake.stopAndRetractCommand().withName("AutoNamed_IntakeStopAndRetract"));

        NamedCommands.registerCommand("TransferRun", transfer.runCommand().withName("AutoNamed_TransferRun"));
        NamedCommands.registerCommand("TransferReverse", transfer.reverseCommand().withName("AutoNamed_TransferReverse"));
        NamedCommands.registerCommand("TransferStop", transfer.stopCommand().withName("AutoNamed_TransferStop"));

        NamedCommands.registerCommand(
                "ShooterAimHub",
                AutoCommands.aimForHub(drive, shooter).withName("AutoNamed_ShooterAimHub"));
        NamedCommands.registerCommand(
                "ShooterShootHub",
                AutoCommands.shootHub(drive, shooter, transfer).withName("AutoNamed_ShooterShootHub"));
        NamedCommands.registerCommand("ShooterHome", shooter.homeCommand().withName("AutoNamed_ShooterHome"));
        NamedCommands.registerCommand("ShooterStop", shooter.stopCommand().withName("AutoNamed_ShooterStop"));
    }
}
