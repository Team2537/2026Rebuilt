package frc.robot.subsystems.transfer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    public void setPercent(double percent) {
        double clampedPercent = MathUtil.clamp(percent, -1.0, 1.0);
        io.setPercent(clampedPercent);
        Logger.recordOutput("Transfer/CommandedPercent", clampedPercent);
    }

    public Command runCommand() {
        return Commands.runEnd(() -> setPercent(TransferConstants.RUN_TRANSFER_PERCENT), io::stop, this).withName("TransferRun");
    }

    public Command reverseCommand() {
        return Commands.runEnd(() -> setPercent(-TransferConstants.RUN_TRANSFER_PERCENT), io::stop, this).withName("TransferReverse");
    }

    public Command stopCommand() {
        return Commands.runOnce(this::stopAll, this).withName("TransferStop");
    }

    public void stopAll() {
        io.stop();
    }
}
