package frc.robot.subsystems.transfer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Transfer extends SubsystemBase {
    private final TransferIO io;
    private final TransferIOInputsAutoLogged inputs = new TransferIOInputsAutoLogged();

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
        }
    }

    public Command runCommand() {
        return Commands.runEnd(() -> io.setPercent(TransferConstants.RUN_TRANSFER_PERCENT), io::stop, this).withName("TransferRun");
    }

    public Command reverseCommand() {
        return Commands.runEnd(() -> io.setPercent(-TransferConstants.RUN_TRANSFER_PERCENT), io::stop, this).withName("TransferReverse");
    }

    public Command runWhen(BooleanSupplier enabledSupplier) {
        return Commands.runEnd(
                () -> io.setPercent(enabledSupplier.getAsBoolean() ? TransferConstants.RUN_TRANSFER_PERCENT : 0.0),
                io::stop,
                this)
                .withName("TransferRunWhen");
    }

    public Command stopCommand() {
        return Commands.runOnce(this::stopAll, this).withName("TransferStop");
    }

    public void stopAll() {
        io.stop();
    }
}
