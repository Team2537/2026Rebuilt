package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private enum MotionState {
        UNKNOWN,
        RETRACTED,
        EXTENDED,
        MOVING_TO_RETRACTED,
        MOVING_TO_EXTENDED,
        HOMING
    }

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private MotionState motionState = MotionState.RETRACTED;
    private boolean requestedExtended = false;
    private boolean leftHomeSucceeded = false;
    private boolean rightHomeSucceeded = false;
    private MotionState preHomeMotionState = MotionState.RETRACTED;
    private boolean preHomeRequestedExtended = false;

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

        updateMotionStateFromSensors();
        Logger.recordOutput("Intake/Extended", isExtended());
        Logger.recordOutput("Intake/RequestedExtended", requestedExtended);
        Logger.recordOutput("Intake/MotionState", motionState.name());
    }

    public void setExtended(boolean isExtended) {
        requestedExtended = isExtended;
        motionState = isExtended ? MotionState.EXTENDED : MotionState.RETRACTED;
        requestIntakePosition(
                isExtended ? IntakeConstants.EXTENDED_POSITION_ROT : IntakeConstants.RETRACTED_POSITION_ROT,
                IntakeConstants.INTAKE_VELOCITY,
                IntakeConstants.INTAKE_ACCELERATION,
                IntakeConstants.INTAKE_MAX_VOLTS);
    }

    public boolean isExtended() {
        return motionState == MotionState.EXTENDED;
    }

    public Command retractCommand() {
        return moveToPositionCommand(
                false,
                IntakeConstants.INTAKE_VELOCITY,
                IntakeConstants.INTAKE_ACCELERATION,
                IntakeConstants.INTAKE_MAX_VOLTS,
                "IntakeWaitForPosition").withName("IntakeRetract");
    }

    public Command slowRetractCommand() {
        return moveToPositionCommand(
                false,
                IntakeConstants.SLOW_INTAKE_VELOCITY,
                IntakeConstants.SLOW_INTAKE_ACCELERATION,
                IntakeConstants.SLOW_RETRACT_MAX_VOLTS,
                "IntakeWaitForSlowRetract").withName("IntakeSlowRetract");
    }

    public Command extendCommand() {
        return moveToPositionCommand(
                true,
                IntakeConstants.INTAKE_VELOCITY,
                IntakeConstants.INTAKE_ACCELERATION,
                IntakeConstants.INTAKE_MAX_VOLTS,
                "IntakeWaitForPosition").withName("IntakeExtend");
    }

    public Command toggleExtendedCommand() {
        return Commands.either(
                retractCommand(),
                extendCommand(),
                this::isExtendRequested)
                .withName("IntakeToggleExtended");
    }

    public Command homeCommand() {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    preHomeMotionState = motionState;
                    preHomeRequestedExtended = requestedExtended;
                    motionState = MotionState.HOMING;
                    leftHomeSucceeded = false;
                    rightHomeSucceeded = false;
                }, this),
                Commands.parallel(homeLeftCommand(), homeRightCommand()).withName("IntakeHomeBoth"),
                Commands.either(
                        Commands.sequence(
                                Commands.waitSeconds(0.1),
                                Commands.runOnce(io::resetEncoders, this),
                                Commands.runOnce(
                                        () -> beginMotionToPosition(
                                                false,
                                                IntakeConstants.RETRACTED_POSITION_ROT,
                                                IntakeConstants.INTAKE_VELOCITY,
                                                IntakeConstants.INTAKE_ACCELERATION,
                                                IntakeConstants.INTAKE_MAX_VOLTS),
                                        this)),
                        Commands.runOnce(
                                () -> {
                                    motionState = preHomeMotionState;
                                    requestedExtended = preHomeRequestedExtended;
                                    DriverStation.reportWarning(
                                            "Intake homing did not hit both current thresholds; skipping encoder reset/retract.",
                                            false);
                                },
                                this),
                        this::didHomeSucceed))
                .finallyDo(interrupted -> {
                    if (interrupted) {
                        io.stop();
                        motionState = MotionState.UNKNOWN;
                    }
                })
                .withName("IntakeHome");
    }

    private Command homeLeftCommand() {
        return homeSideCommand(
                () -> io.setLeftIntakeVoltage(homingVoltage()),
                io::stopLeftIntake,
                () -> Math.abs(inputs.leftStatorCurrentAmps) > IntakeConstants.HOMING_CURRENT_THRESHOLD_AMPS,
                (success) -> leftHomeSucceeded = success,
                () -> leftHomeSucceeded,
                "IntakeHomeLeftWaitUntil",
                "Left intake homing timed out before current threshold.",
                "IntakeHomeLeft");
    }

    private Command homeRightCommand() {
        return homeSideCommand(
                () -> io.setRightIntakeVoltage(homingVoltage()),
                io::stopRightIntake,
                () -> Math.abs(inputs.rightStatorCurrentAmps) > IntakeConstants.HOMING_CURRENT_THRESHOLD_AMPS,
                (success) -> rightHomeSucceeded = success,
                () -> rightHomeSucceeded,
                "IntakeHomeRightWaitUntil",
                "Right intake homing timed out before current threshold.",
                "IntakeHomeRight");
    }

    private Command homeSideCommand(
            Runnable homeAction,
            Runnable stopAction,
            BooleanSupplier atHomingStop,
            java.util.function.Consumer<Boolean> setSuccess,
            BooleanSupplier didSucceed,
            String waitCommandName,
            String timeoutWarning,
            String commandName) {
        return Commands.sequence(
                        Commands.runOnce(() -> setSuccess.accept(false)),
                        Commands.runOnce(homeAction),
                        Commands.sequence(
                                        Commands.waitUntil(atHomingStop).withName(waitCommandName),
                                        Commands.runOnce(() -> setSuccess.accept(true)))
                                .withTimeout(homingWaitTimeoutSec()),
                        Commands.runOnce(stopAction),
                        Commands.runOnce(
                                () -> {
                                    if (!didSucceed.getAsBoolean()) {
                                        DriverStation.reportWarning(timeoutWarning, false);
                                    }
                                }))
                .withName(commandName);
    }

    private static double homingWaitTimeoutSec() {
        return IntakeConstants.HOMING_WAIT_TIMEOUT_SEC;
    }

    private static double homingVoltage() {
        return RobotBase.isSimulation() ? -12.0 : IntakeConstants.HOMING_VOLTAGE;
    }

    public Command spinRoller() {
        return this.runEnd(
                () -> io.setRollerRpm(IntakeConstants.ROLLER_RPM),
                io::stopRoller)
                .withName("IntakeSpinRoller");
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
        return Commands.runOnce(() -> {
            io.stop();
            motionState = MotionState.UNKNOWN;
        }, this).withName("IntakeStop");
    }

    public Command stopAndRetractCommand() {
        return Commands.sequence(
                retractCommand(),
                stopCommand())
                .withName("IntakeStopAndRetract");
    }

    public void stopAll() {
        io.stop();
        motionState = MotionState.UNKNOWN;
    }

    private Command moveToPositionCommand(
            boolean shouldExtend,
            double velocityRotPerSec,
            double accelerationRotPerSecSq,
            double maxVolts,
            String waitCommandName) {
        double targetRot = shouldExtend ? IntakeConstants.EXTENDED_POSITION_ROT : IntakeConstants.RETRACTED_POSITION_ROT;
        return Commands.sequence(
                Commands.runOnce(
                        () -> beginMotionToPosition(
                                shouldExtend,
                                targetRot,
                                velocityRotPerSec,
                                accelerationRotPerSecSq,
                                maxVolts),
                        this),
                Commands.waitUntil(() -> isAtTargetPosition(targetRot))
                        .withTimeout(IntakeConstants.MOVE_TIMEOUT_SEC)
                        .withName(waitCommandName),
                Commands.runOnce(() -> completeMotionState(shouldExtend, targetRot), this));
    }

    private void requestIntakePosition(
            double leftTargetRot,
            double velocityRotPerSec,
            double accelerationRotPerSecSq,
            double maxVolts) {
        io.setIntakePosition(leftTargetRot, velocityRotPerSec, accelerationRotPerSecSq, maxVolts);
    }

    private boolean isAtTargetPosition(double leftTargetRot) {
        double leftPositionRot = Units.radiansToRotations(inputs.leftPositionRad);
        double rightPositionRot = Units.radiansToRotations(inputs.rightPositionRad);
        double rightTargetRot = IntakeConstants.RIGHT_OPPOSES_LEFT ? -leftTargetRot : leftTargetRot;
        return Math.abs(leftPositionRot - leftTargetRot) <= IntakeConstants.POSITION_TOLERANCE_ROT
                && Math.abs(rightPositionRot - rightTargetRot) <= IntakeConstants.POSITION_TOLERANCE_ROT;
    }

    private void beginMotionToPosition(
            boolean shouldExtend,
            double targetRot,
            double velocityRotPerSec,
            double accelerationRotPerSecSq,
            double maxVolts) {
        requestedExtended = shouldExtend;
        motionState = shouldExtend ? MotionState.MOVING_TO_EXTENDED : MotionState.MOVING_TO_RETRACTED;
        requestIntakePosition(targetRot, velocityRotPerSec, accelerationRotPerSecSq, maxVolts);
    }

    private void completeMotionState(boolean shouldExtend, double targetRot) {
        if (isAtTargetPosition(targetRot)) {
            motionState = shouldExtend ? MotionState.EXTENDED : MotionState.RETRACTED;
            return;
        }
        motionState = shouldExtend ? MotionState.MOVING_TO_EXTENDED : MotionState.MOVING_TO_RETRACTED;
    }

    private boolean didHomeSucceed() {
        return leftHomeSucceeded && rightHomeSucceeded;
    }

    private boolean isExtendRequested() {
        return requestedExtended;
    }

    private void updateMotionStateFromSensors() {
        if (motionState == MotionState.HOMING) {
            return;
        }
        if (!requestedExtended && isAtTargetPosition(IntakeConstants.RETRACTED_POSITION_ROT)) {
            motionState = MotionState.RETRACTED;
            return;
        }
        if (requestedExtended && isAtTargetPosition(IntakeConstants.EXTENDED_POSITION_ROT)) {
            motionState = MotionState.EXTENDED;
            return;
        }
        if (requestedExtended && motionState == MotionState.EXTENDED) {
            return;
        }
        if (!requestedExtended && motionState == MotionState.RETRACTED) {
            return;
        }
        if (motionState != MotionState.MOVING_TO_EXTENDED && motionState != MotionState.MOVING_TO_RETRACTED) {
            motionState = MotionState.UNKNOWN;
        }
    }
}
