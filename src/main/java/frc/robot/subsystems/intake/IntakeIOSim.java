package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Simulation implementation of intake IO. */
public class IntakeIOSim implements IntakeIO {
    private static final double LOOP_PERIOD_SEC = 0.02;
    private static final double MAX_VOLTS = IntakeConstants.INTAKE_MAX_VOLTS;
    private static final double MAX_ROLLER_RPM = IntakeConstants.ROLLER_MAX_RPM;
    private static final double HOMING_STATOR_CURRENT_AMPS = IntakeConstants.HOMING_CURRENT_THRESHOLD_AMPS + 5.0;
    private static final double RIGHT_MOTOR_INVERSION = 1.0;

    private static final DCMotor INTAKE_GEARBOX = DCMotor.getKrakenX60Foc(1);
    private static final DCMotor ROLLER_GEARBOX = DCMotor.getKrakenX60Foc(1);

    private static final double ROLLER_KV_VOLTS_PER_RPM = MAX_VOLTS / MAX_ROLLER_RPM;

    private final DCMotorSim leftIntakeSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(INTAKE_GEARBOX, 0.015, 1.0),
            INTAKE_GEARBOX);
    private final DCMotorSim rightIntakeSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(INTAKE_GEARBOX, 0.015, 1.0),
            INTAKE_GEARBOX);
    private final DCMotorSim rollerSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(ROLLER_GEARBOX, 0.003, 1.0),
            ROLLER_GEARBOX);

    private final PIDController leftIntakePositionController = new PIDController(2.0, 0.0, 0.5);
    private final PIDController rightIntakePositionController = new PIDController(2.0, 0.0, 0.5);
    private final PIDController rollerVelocityController = new PIDController(0.004, 0.0, 0.0);

    private double leftTargetIntakePositionRot = IntakeConstants.RETRACTED_POSITION_ROT;
    private double rightTargetIntakePositionRot = applyRightAlignment(IntakeConstants.RETRACTED_POSITION_ROT);
    private double targetRollerRpm = 0.0;
    private double leftIntakeVoltageLimit = MAX_VOLTS;
    private double rightIntakeVoltageLimit = MAX_VOLTS;

    private boolean leftIntakePositionClosedLoop = false;
    private boolean rightIntakePositionClosedLoop = false;
    private boolean leftIntakeOpenLoop = false;
    private boolean rightIntakeOpenLoop = false;
    private boolean rollerVelocityClosedLoop = false;

    private double leftRequestedVolts = 0.0;
    private double rightRequestedVolts = 0.0;
    private boolean leftAtRetractStop = false;
    private boolean rightAtRetractStop = false;

    private double leftAppliedVolts = 0.0;
    private double rightAppliedVolts = 0.0;
    private double rollerAppliedVolts = 0.0;
    private double leftPositionOffsetRad = 0.0;
    private double rightPositionOffsetRad = 0.0;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        double leftPositionRot = Units.radiansToRotations(leftIntakeSim.getAngularPositionRad() - leftPositionOffsetRad);
        double rightPositionRot = Units.radiansToRotations(rightIntakeSim.getAngularPositionRad() - rightPositionOffsetRad);

        if (leftIntakeOpenLoop) {
            leftAtRetractStop = leftRequestedVolts < 0.0 && leftPositionRot <= IntakeConstants.RETRACTED_POSITION_ROT;
            leftAppliedVolts = leftAtRetractStop ? 0.0 : MathUtil.clamp(leftRequestedVolts, -MAX_VOLTS, MAX_VOLTS);
        } else if (leftIntakePositionClosedLoop) {
            leftAtRetractStop = false;
            leftAppliedVolts = MathUtil.clamp(
                    leftIntakePositionController.calculate(leftPositionRot, leftTargetIntakePositionRot),
                    -leftIntakeVoltageLimit,
                    leftIntakeVoltageLimit);
        } else {
            leftAtRetractStop = false;
            leftAppliedVolts = 0.0;
        }

        if (rightIntakeOpenLoop) {
            rightAtRetractStop = rightRequestedVolts < 0.0
                    && applyRightAlignment(rightPositionRot) <= IntakeConstants.RETRACTED_POSITION_ROT;
            rightAppliedVolts = rightAtRetractStop ? 0.0 : MathUtil.clamp(rightRequestedVolts, -MAX_VOLTS, MAX_VOLTS);
        } else if (rightIntakePositionClosedLoop) {
            rightAtRetractStop = false;
            rightAppliedVolts = MathUtil.clamp(
                    rightIntakePositionController.calculate(rightPositionRot, rightTargetIntakePositionRot),
                    -rightIntakeVoltageLimit,
                    rightIntakeVoltageLimit);
        } else {
            rightAtRetractStop = false;
            rightAppliedVolts = 0.0;
        }

        rollerAppliedVolts = rollerVelocityClosedLoop
                ? MathUtil.clamp(
                        ROLLER_KV_VOLTS_PER_RPM * targetRollerRpm
                                + rollerVelocityController.calculate(rollerSim.getAngularVelocityRPM(), targetRollerRpm),
                        -MAX_VOLTS,
                        MAX_VOLTS)
                : 0.0;

        leftIntakeSim.setInputVoltage(leftAppliedVolts);
        rightIntakeSim.setInputVoltage(rightAppliedVolts * RIGHT_MOTOR_INVERSION);
        rollerSim.setInputVoltage(rollerAppliedVolts);

        leftIntakeSim.update(LOOP_PERIOD_SEC);
        rightIntakeSim.update(LOOP_PERIOD_SEC);
        rollerSim.update(LOOP_PERIOD_SEC);

        double leftSupplyCurrentAmps = Math.abs(leftIntakeSim.getCurrentDrawAmps());
        double rightSupplyCurrentAmps = Math.abs(rightIntakeSim.getCurrentDrawAmps());
        double leftStatorCurrentAmps = leftAtRetractStop ? HOMING_STATOR_CURRENT_AMPS : leftSupplyCurrentAmps;
        double rightStatorCurrentAmps = rightAtRetractStop ? HOMING_STATOR_CURRENT_AMPS : rightSupplyCurrentAmps;

        inputs.leftAppliedVolts = leftAppliedVolts;
        inputs.leftPositionRad = leftIntakeSim.getAngularPositionRad() - leftPositionOffsetRad;
        inputs.leftSupplyCurrentAmps = leftSupplyCurrentAmps;
        inputs.leftStatorCurrentAmps = leftStatorCurrentAmps;
        inputs.leftVelocityRpm = leftIntakeSim.getAngularVelocityRPM();

        inputs.rightAppliedVolts = rightAppliedVolts;
        inputs.rightPositionRad = -(rightIntakeSim.getAngularPositionRad() - rightPositionOffsetRad);
        inputs.rightSupplyCurrentAmps = rightSupplyCurrentAmps;
        inputs.rightStatorCurrentAmps = rightStatorCurrentAmps;
        inputs.rightVelocityRpm = -rightIntakeSim.getAngularVelocityRPM();

        inputs.rollerAppliedVolts = rollerAppliedVolts;
        inputs.rollerPositionRad = rollerSim.getAngularPositionRad();
        inputs.rollerSupplyCurrentAmps = Math.abs(rollerSim.getCurrentDrawAmps());
        inputs.rollerStatorCurrentAmps = Math.abs(rollerSim.getCurrentDrawAmps());
        inputs.rollerVelocityRpm = rollerSim.getAngularVelocityRPM();
    }

    @Override
    public void setRollerRpm(double rpm) {
        targetRollerRpm = MathUtil.clamp(rpm, -MAX_ROLLER_RPM, MAX_ROLLER_RPM);
        rollerVelocityClosedLoop = true;
    }

    @Override
    public void setIntakePosition(
            double leftTargetRot,
            double velocityRotPerSec,
            double accelerationRotPerSecSq,
            double maxVolts) {
        leftIntakeOpenLoop = false;
        rightIntakeOpenLoop = false;
        leftRequestedVolts = 0.0;
        rightRequestedVolts = 0.0;
        leftAtRetractStop = false;
        rightAtRetractStop = false;

        leftTargetIntakePositionRot = leftTargetRot;
        rightTargetIntakePositionRot = applyRightAlignment(leftTargetRot);
        leftIntakeVoltageLimit = MathUtil.clamp(Math.abs(maxVolts), 0.0, MAX_VOLTS);
        rightIntakeVoltageLimit = leftIntakeVoltageLimit;
        leftIntakePositionClosedLoop = true;
        rightIntakePositionClosedLoop = true;
    }

    @Override
    public void setLeftIntakeVoltage(double volts) {
        leftIntakePositionClosedLoop = false;
        leftIntakeOpenLoop = true;
        leftRequestedVolts = MathUtil.clamp(volts, -MAX_VOLTS, MAX_VOLTS);
        leftAtRetractStop = false;
        leftIntakeVoltageLimit = MAX_VOLTS;
    }

    @Override
    public void setRightIntakeVoltage(double volts) {
        rightIntakePositionClosedLoop = false;
        rightIntakeOpenLoop = true;
        rightRequestedVolts = MathUtil.clamp(volts, -MAX_VOLTS, MAX_VOLTS);
        rightAtRetractStop = false;
        rightIntakeVoltageLimit = MAX_VOLTS;
    }

    @Override
    public void resetEncoders() {
        leftPositionOffsetRad = leftIntakeSim.getAngularPositionRad();
        rightPositionOffsetRad = rightIntakeSim.getAngularPositionRad();
    }

    @Override
    public void stop() {
        stopIntake();
        stopRoller();
    }

    @Override
    public void stopIntake() {
        stopLeftIntake();
        stopRightIntake();
    }

    @Override
    public void stopLeftIntake() {
        leftIntakePositionClosedLoop = false;
        leftIntakeOpenLoop = false;
        leftRequestedVolts = 0.0;
        leftAtRetractStop = false;
        leftIntakeVoltageLimit = MAX_VOLTS;
        leftIntakePositionController.reset();
    }

    @Override
    public void stopRightIntake() {
        rightIntakePositionClosedLoop = false;
        rightIntakeOpenLoop = false;
        rightRequestedVolts = 0.0;
        rightAtRetractStop = false;
        rightIntakeVoltageLimit = MAX_VOLTS;
        rightIntakePositionController.reset();
    }

    @Override
    public void stopRoller() {
        rollerVelocityClosedLoop = false;
        targetRollerRpm = 0.0;
        rollerVelocityController.reset();
    }

    private static double applyRightAlignment(double leftReference) {
        return IntakeConstants.RIGHT_OPPOSES_LEFT ? leftReference : -leftReference;
    }
}
