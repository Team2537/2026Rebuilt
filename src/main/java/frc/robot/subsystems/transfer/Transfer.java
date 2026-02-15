package frc.robot.subsystems.transfer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Transfer extends SubsystemBase {
    private final TransferIO io;
    private final TransferIOInputsAutoLogged inputs = new TransferIOInputsAutoLogged();
    private double targetPercent = TransferConstants.DEFAULT_TRANSFER_PERCENT;

    public Transfer(TransferIO io) {
        super("transfer");
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Transfer", inputs);

        if (DriverStation.isDisabled()) {
            io.stop();
        } else {
            io.setPercent(targetPercent);
        }

        Logger.recordOutput("Transfer/TargetPercent", targetPercent);
        Logger.recordOutput("Transfer/Running", isRunning());
    }

    public boolean isRunning() {
        return Math.abs(targetPercent - TransferConstants.DEFAULT_TRANSFER_PERCENT) > 1e-6;
    }

    public void setRunning(boolean running) {
        targetPercent = running
                ? TransferConstants.RUN_TRANSFER_PERCENT
                : TransferConstants.DEFAULT_TRANSFER_PERCENT;
    }

    public void toggleRunning() {
        setRunning(!isRunning());
    }

    public Command toggleCommand() {
        return Commands.runOnce(this::toggleRunning, this).withName("TransferToggle");
    }

    public void stopAll() {
        setRunning(false);
        io.stop();
    }
}
