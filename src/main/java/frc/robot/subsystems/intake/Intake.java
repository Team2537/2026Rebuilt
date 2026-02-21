package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private static final double HOMING_WAIT_TIMEOUT_SEC = 3.0;

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private boolean extended = false;

    public Intake(IntakeIO io) {
        super("intake");
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        if (DriverStation.isDisabled()) {
            io.stop();
        }

        Logger.recordOutput("Intake/Extended", extended);
    }

    public void setExtended(boolean extended) {
        this.extended = extended;
        if (extended) {
            io.extend();
        } else {
            io.retract();
        }
    }

    public Command retractCommand() {
        return Commands.runOnce(() -> setExtended(false), this).withName("IntakeRetract");
    }

    public Command extendCommand() {
        return Commands.runOnce(() -> setExtended(true), this).withName("IntakeExtend");
    }

    public Command homeCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> io.home(), this),
            Commands.waitUntil(() -> inputs.leftStatorCurrentAmps > IntakeConstants.HOMING_CURRENT_THRESHOLD_AMPS)
                    .withTimeout(HOMING_WAIT_TIMEOUT_SEC)
                    .withName("IntakeHomeWaitUntil"),
            Commands.runOnce(() -> io.stop(), this),
            Commands.either(
                Commands.runOnce(() -> io.resetEncoders(), this),
                Commands.runOnce(() -> DriverStation.reportWarning("Intake homing timed out before current threshold; skipping encoder reset.", false)),
                () -> inputs.leftStatorCurrentAmps > IntakeConstants.HOMING_CURRENT_THRESHOLD_AMPS
            ),
            Commands.runOnce(() -> setExtended(false), this)
        ).withName("IntakeHome");
    }

    public Command spinRoller() {
        return this.runEnd(
                () -> io.setRollerRpm(IntakeConstants.ROLLER_RPM),
                io::stop)
                .withName("IntakeSpinRoller");
    }

    public void stopAll() {
        io.stop();
    }
}
