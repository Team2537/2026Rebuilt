package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

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

    public boolean isExtended() {
        return extended;
    }

    public Command retractCommand() {
        return moveToPositionCommand(false).withName("IntakeRetract");
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
        return moveToPositionCommand(true).withName("IntakeExtend");
    }

    public Command toggleExtendedCommand() {
        return Commands.either(
                retractCommand(),
                extendCommand(),
                this::isExtended)
                .withName("IntakeToggleExtended");
    }

    public Command homeCommand() {
        return Commands.sequence(
                Commands.parallel(homeLeftCommand(), homeRightCommand()).withName("IntakeHomeBoth"),
                Commands.sequence(
                        Commands.waitSeconds(2),
                        Commands.runOnce(() -> io.resetEncoders(), this),
                        Commands.runOnce(() -> setExtended(false), this)))
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
                .finallyDo(interrupted -> io.stop())
                .withName("IntakeHome");
    }

    private Command homeLeftCommand() {
        BooleanSupplier atHomingStop =
                () -> Math.abs(inputs.leftStatorCurrentAmps) > IntakeConstants.HOMING_CURRENT_THRESHOLD_AMPS;
        return Commands.sequence(
                Commands.runOnce(io::homeLeft),
                Commands.waitUntil(atHomingStop)
                        .withTimeout(IntakeConstants.HOMING_WAIT_TIMEOUT_SEC)
                        .withName("IntakeHomeLeftWaitUntil"),
                Commands.runOnce(() ->
                    io.stopLeft()
                ),
                Commands.either(
                        Commands.none().withName("IntakeHomeLeftSuccess"),
                        Commands.runOnce(
                                () -> DriverStation.reportWarning(
                                        "Left intake homing timed out before current threshold; continuing.",
                                        false)),
                        atHomingStop))
                .withName("IntakeHomeLeft");
    }

    private Command homeRightCommand() {
        BooleanSupplier atHomingStop =
                () -> Math.abs(inputs.rightStatorCurrentAmps) > IntakeConstants.HOMING_CURRENT_THRESHOLD_AMPS;
        return Commands.sequence(
                Commands.runOnce(io::homeRight, this),
                Commands.waitUntil(atHomingStop)
                        .withTimeout(IntakeConstants.HOMING_WAIT_TIMEOUT_SEC)
                        .withName("IntakeHomeRightWaitUntil"),
                Commands.runOnce(() ->
                    io.stopRight()
                ),
                Commands.either(
                        Commands.none().withName("IntakeHomeRightSuccess"),
                        Commands.runOnce(
                                () -> DriverStation.reportWarning(
                                        "Right intake homing timed out before current threshold; continuing.",
                                        false),
                                this),
                        atHomingStop))
                .withName("IntakeHomeRight");
    }

    public Command spinRoller() {
        return this.runEnd(
                () -> io.setRollerRpm(IntakeConstants.ROLLER_RPM),
                io::stopRoller)
                .withName("IntakeSpinRoller");
    }

    public Command spinRollerSlow() {
        return this.runEnd(
                () -> io.setRollerRpm(IntakeConstants.SLOW_ROLLER_RPM),
                io::stopRoller)
                .withName("IntakeSpinRollerSlow");
    }

    public Command backgroundCommand() {
        return Commands.runEnd(
                () -> {
                    if (isExtended()) {
                        io.setRollerRpm(IntakeConstants.SLOW_ROLLER_RPM);
                    } else {
                        io.stopRoller();
                    }
                },
                io::stopRoller,
                this).withName("IntakeBackground");
    }

    public Command stopCommand() {
        return Commands.runOnce(io::stop, this).withName("IntakeStop");
    }

    public Command stopAndRetractCommand() {
        return Commands.sequence(
                retractCommand(),
                stopCommand())
                .withName("IntakeStopAndRetract");
    }

    public void stopAll() {
        io.stop();
    }

    private Command moveToPositionCommand(boolean shouldExtend) {
        double targetRot = shouldExtend ? IntakeConstants.EXTENDED_POSITION_ROT : IntakeConstants.RETRACTED_POSITION_ROT;
        return Commands.sequence(
                Commands.runOnce(() -> setExtended(shouldExtend), this),
                Commands.waitUntil(() -> isAtTargetPosition(targetRot))
                        .withTimeout(IntakeConstants.MOVE_TIMEOUT_SEC)
                        .withName("IntakeWaitForPosition"));
    }

    private boolean isAtTargetPosition(double leftTargetRot) {
        double leftPositionRot = Units.radiansToRotations(inputs.leftPositionRad);
        double rightPositionRot = Units.radiansToRotations(inputs.rightPositionRad);
        double rightTargetRot = IntakeConstants.RIGHT_OPPOSES_LEFT ? -leftTargetRot : leftTargetRot;
        return Math.abs(leftPositionRot - leftTargetRot) <= IntakeConstants.POSITION_TOLERANCE_ROT
                && Math.abs(rightPositionRot - rightTargetRot) <= IntakeConstants.POSITION_TOLERANCE_ROT;
    }
}
