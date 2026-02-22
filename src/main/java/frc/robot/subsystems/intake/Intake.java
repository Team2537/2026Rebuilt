package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
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

    public void setExtended(boolean isExtended) {
        this.extended = isExtended;
        if (isExtended) {
            io.extend();
        } else {
            io.retract();
        }
    }

    public BooleanSupplier isExtended() {
        return () -> extended;
    }

    public Command retractCommand() {
        return Commands.runOnce(() -> setExtended(false), this).withName("IntakeRetract");
    }

    public Command slowRetractCommand() {
        return Commands.runOnce(
                () -> {
                    extended = false;
                    io.slowRetract();
                },
                this).withName("IntakeSlowRetract");
    }

    public Command extendCommand() {
        return Commands.runOnce(() -> setExtended(true), this).withName("IntakeExtend");
    }

    public Command homeCommand() {
        BooleanSupplier atHomingStop =
                () -> Math.abs(inputs.leftStatorCurrentAmps) > IntakeConstants.HOMING_CURRENT_THRESHOLD_AMPS;
        return Commands.sequence(
            Commands.runOnce(() -> io.home(), this),
            Commands.waitUntil(atHomingStop)
                    .withTimeout(HOMING_WAIT_TIMEOUT_SEC)
                    .withName("IntakeHomeWaitUntil"),
            Commands.runOnce(() -> io.stop(), this),
            Commands.either(
                Commands.sequence(
                    Commands.runOnce(() -> io.resetEncoders(), this),
                    Commands.runOnce(() -> setExtended(false), this)),
                Commands.runOnce(
                        () -> {
                            // Keep state retracted to prevent background roller spin, but do not
                            // command a position move when homing never established a valid zero.
                            extended = false;
                            DriverStation.reportWarning(
                                    "Intake homing timed out before current threshold; skipping encoder reset/retract.",
                                    false);
                        },
                        this),
                atHomingStop))
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
                .finallyDo(interrupted -> io.stop())
                .withName("IntakeHome");
    }

    public Command spinRoller() {
        return this.runEnd(
                () -> io.setRollerRpm(IntakeConstants.ROLLER_RPM),
                io::stop)
                .withName("IntakeSpinRoller");
    }

    public Command spinRollerSlow() {
        return this.runEnd(
                () -> io.setRollerRpm(IntakeConstants.SLOW_ROLLER_RPM),
                io::stop)
                .withName("IntakeSpinRollerSlow");
    }

    public Command backgroundCommand() {
        return spinRollerSlow().onlyIf(isExtended()).withName("IntakeBackground");
    }

    public Command stopCommand() {
        return Commands.runOnce(this::stopAll, this).withName("IntakeStop");
    }

    public void stopAll() {
        extended = false;
        io.stop();
    }
}
