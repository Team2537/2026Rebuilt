package frc.robot.subsystems.transfer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ElasticNotifications;
import org.littletonrobotics.junction.Logger;

public class Transfer extends SubsystemBase {
    private final TransferIO io;
    private final TransferIOInputsAutoLogged inputs = new TransferIOInputsAutoLogged();
    private double commandedPercent = 0.0;
    private int jamCurrentCycles = 0;
    private boolean jamDetected = false;
    private double lastJamNotificationSec = Double.NEGATIVE_INFINITY;

    public Transfer(TransferIO io) {
        super("transfer");
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Transfer", inputs);

        if (DriverStation.isDisabled()) {
            stopAll();
            jamCurrentCycles = 0;
            jamDetected = false;
            return;
        }

        if (Math.abs(commandedPercent) > 1e-4
                && Math.abs(inputs.supplyCurrentAmps) >= TransferConstants.JAM_CURRENT_THRESHOLD_AMPS) {
            jamCurrentCycles++;
        } else {
            jamCurrentCycles = 0;
            jamDetected = false;
        }

        if (!jamDetected && jamCurrentCycles >= TransferConstants.JAM_DETECT_CYCLES) {
            jamDetected = true;
            double nowSec = Timer.getFPGATimestamp();
            if (nowSec - lastJamNotificationSec >= TransferConstants.JAM_NOTIFY_COOLDOWN_SEC) {
                lastJamNotificationSec = nowSec;
                ElasticNotifications.sendWarning(
                        "Transfer",
                        String.format(
                                "Possible jam detected: %.1fA while commanded %.0f%%",
                                Math.abs(inputs.supplyCurrentAmps),
                                commandedPercent * 100.0));
            }
        }

        Logger.recordOutput("Transfer/JamCurrentCycles", jamCurrentCycles);
        Logger.recordOutput("Transfer/JamDetected", jamDetected);
    }

    public void setPercent(double percent) {
        double clampedPercent = MathUtil.clamp(percent, -1.0, 1.0);
        commandedPercent = clampedPercent;
        io.setPercent(clampedPercent);
        Logger.recordOutput("Transfer/CommandedPercent", clampedPercent);
    }

    public Command runCommand() {
        return Commands.runEnd(
                () -> setPercent(TransferConstants.RUN_TRANSFER_PERCENT),
                this::stopAll,
                this).withName("TransferRun");
    }

    public Command reverseCommand() {
        return Commands.runEnd(
                () -> setPercent(-TransferConstants.RUN_TRANSFER_PERCENT),
                this::stopAll,
                this).withName("TransferReverse");
    }

    public Command stopCommand() {
        return Commands.runOnce(this::stopAll, this).withName("TransferStop");
    }

    public void stopAll() {
        commandedPercent = 0.0;
        io.stop();
        Logger.recordOutput("Transfer/CommandedPercent", 0.0);
    }
}
