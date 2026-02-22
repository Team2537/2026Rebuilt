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
    private static final double MAX_VOLTS = 12.0;
    private static final double MAX_ROLLER_RPM = 6500.0;
    private static final double SLOW_RETRACT_VOLTS_LIMIT = 4.0;
    private static final double HOMING_VOLTS = -1.0;
    private static final double HOMING_STATOR_CURRENT_AMPS = IntakeConstants.HOMING_CURRENT_THRESHOLD_AMPS + 5.0;

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

    private final PIDController leftIntakePositionController = new PIDController(3.0, 0.0, 0.05);
    private final PIDController rightIntakePositionController = new PIDController(3.0, 0.0, 0.05);
    private final PIDController rollerVelocityController = new PIDController(0.004, 0.0, 0.0);

    private double leftTargetIntakePositionRot = IntakeConstants.RETRACTED_POSITION_ROT;
    private double rightTargetIntakePositionRot = applyRightAlignment(IntakeConstants.RETRACTED_POSITION_ROT);
    private double targetRollerRpm = 0.0;
    private double leftIntakeVoltageLimit = MAX_VOLTS;
    private double rightIntakeVoltageLimit = MAX_VOLTS;

    private boolean leftIntakePositionClosedLoop = false;
    private boolean rightIntakePositionClosedLoop = false;
    private boolean rollerVelocityClosedLoop = false;
    private boolean leftHomingActive = false;
    private boolean rightHomingActive = false;
    private boolean leftHomingAtStop = false;
    private boolean rightHomingAtStop = false;

    private double leftAppliedVolts = 0.0;
    private double rightAppliedVolts = 0.0;
    private double rollerAppliedVolts = 0.0;
    private double leftPositionOffsetRad = 0.0;
    private double rightPositionOffsetRad = 0.0;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        double leftPositionRot = Units.radiansToRotations(leftIntakeSim.getAngularPositionRad() - leftPositionOffsetRad);
        double rightPositionRot = Units.radiansToRotations(rightIntakeSim.getAngularPositionRad() - rightPositionOffsetRad);

        if (leftHomingActive) {
            if (leftPositionRot <= IntakeConstants.RETRACTED_POSITION_ROT) {
                leftHomingAtStop = true;
            }
            leftAppliedVolts = leftHomingAtStop ? 0.0 : HOMING_VOLTS;
        } else if (leftIntakePositionClosedLoop) {
            leftAppliedVolts = MathUtil.clamp(
                    leftIntakePositionController.calculate(leftPositionRot, leftTargetIntakePositionRot),
                    -leftIntakeVoltageLimit,
                    leftIntakeVoltageLimit);
        } else {
            leftAppliedVolts = 0.0;
        }

        if (rightHomingActive) {
            if ((applyRightAlignment(rightPositionRot) <= IntakeConstants.RETRACTED_POSITION_ROT)) {
                rightHomingAtStop = true;
            }
            rightAppliedVolts = rightHomingAtStop ? 0.0 : applyRightAlignment(HOMING_VOLTS);
        } else if (rightIntakePositionClosedLoop) {
            rightAppliedVolts = MathUtil.clamp(
                    rightIntakePositionController.calculate(rightPositionRot, rightTargetIntakePositionRot),
                    -rightIntakeVoltageLimit,
                    rightIntakeVoltageLimit);
        } else {
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
        rightIntakeSim.setInputVoltage(rightAppliedVolts);
        rollerSim.setInputVoltage(rollerAppliedVolts);

        leftIntakeSim.update(LOOP_PERIOD_SEC);
        rightIntakeSim.update(LOOP_PERIOD_SEC);
        rollerSim.update(LOOP_PERIOD_SEC);

        double leftSupplyCurrentAmps = Math.abs(leftIntakeSim.getCurrentDrawAmps());
        double rightSupplyCurrentAmps = Math.abs(rightIntakeSim.getCurrentDrawAmps());
        double leftStatorCurrentAmps = leftHomingAtStop ? HOMING_STATOR_CURRENT_AMPS : leftSupplyCurrentAmps;
        double rightStatorCurrentAmps = rightHomingAtStop ? HOMING_STATOR_CURRENT_AMPS : rightSupplyCurrentAmps;

        inputs.leftAppliedVolts = leftAppliedVolts;
        inputs.leftPositionRad = leftIntakeSim.getAngularPositionRad() - leftPositionOffsetRad;
        inputs.leftSupplyCurrentAmps = leftSupplyCurrentAmps;
        inputs.leftStatorCurrentAmps = leftStatorCurrentAmps;
        inputs.leftVelocityRpm = leftIntakeSim.getAngularVelocityRPM();

        inputs.rightAppliedVolts = rightAppliedVolts;
        inputs.rightPositionRad = rightIntakeSim.getAngularPositionRad() - rightPositionOffsetRad;
        inputs.rightSupplyCurrentAmps = rightSupplyCurrentAmps;
        inputs.rightStatorCurrentAmps = rightStatorCurrentAmps;
        inputs.rightVelocityRpm = rightIntakeSim.getAngularVelocityRPM();

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
    public void retract() {
        leftHomingActive = false;
        rightHomingActive = false;
        leftHomingAtStop = false;
        rightHomingAtStop = false;
        leftTargetIntakePositionRot = IntakeConstants.RETRACTED_POSITION_ROT;
        rightTargetIntakePositionRot = applyRightAlignment(IntakeConstants.RETRACTED_POSITION_ROT);
        leftIntakeVoltageLimit = MAX_VOLTS;
        rightIntakeVoltageLimit = MAX_VOLTS;
        leftIntakePositionClosedLoop = true;
        rightIntakePositionClosedLoop = true;
    }

    @Override
    public void slowRetract() {
        leftHomingActive = false;
        rightHomingActive = false;
        leftHomingAtStop = false;
        rightHomingAtStop = false;
        leftTargetIntakePositionRot = IntakeConstants.RETRACTED_POSITION_ROT;
        rightTargetIntakePositionRot = applyRightAlignment(IntakeConstants.RETRACTED_POSITION_ROT);
        leftIntakeVoltageLimit = SLOW_RETRACT_VOLTS_LIMIT;
        rightIntakeVoltageLimit = SLOW_RETRACT_VOLTS_LIMIT;
        leftIntakePositionClosedLoop = true;
        rightIntakePositionClosedLoop = true;
    }

    @Override
    public void extend() {
        leftHomingActive = false;
        rightHomingActive = false;
        leftHomingAtStop = false;
        rightHomingAtStop = false;
        leftTargetIntakePositionRot = IntakeConstants.EXTENDED_POSITION_ROT;
        rightTargetIntakePositionRot = applyRightAlignment(IntakeConstants.EXTENDED_POSITION_ROT);
        leftIntakeVoltageLimit = MAX_VOLTS;
        rightIntakeVoltageLimit = MAX_VOLTS;
        leftIntakePositionClosedLoop = true;
        rightIntakePositionClosedLoop = true;
    }

    @Override
    public void home() {
        leftIntakePositionClosedLoop = false;
        rightIntakePositionClosedLoop = false;
        leftIntakeVoltageLimit = MAX_VOLTS;
        rightIntakeVoltageLimit = MAX_VOLTS;
        leftHomingActive = true;
        rightHomingActive = true;
        leftHomingAtStop = false;
        rightHomingAtStop = false;
    }

    @Override
    public void resetEncoders() {
        leftPositionOffsetRad = leftIntakeSim.getAngularPositionRad();
        rightPositionOffsetRad = rightIntakeSim.getAngularPositionRad();
    }

    @Override
    public void stop() {
        leftIntakePositionClosedLoop = false;
        rightIntakePositionClosedLoop = false;
        rollerVelocityClosedLoop = false;
        leftHomingActive = false;
        rightHomingActive = false;
        leftHomingAtStop = false;
        rightHomingAtStop = false;
        leftIntakeVoltageLimit = MAX_VOLTS;
        rightIntakeVoltageLimit = MAX_VOLTS;
        targetRollerRpm = 0.0;
        leftIntakePositionController.reset();
        rightIntakePositionController.reset();
        rollerVelocityController.reset();
    }

    private static double applyRightAlignment(double leftReference) {
        return IntakeConstants.RIGHT_OPPOSES_LEFT ? -leftReference : leftReference;
    }
}
