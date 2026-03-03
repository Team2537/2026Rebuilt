package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Simulation implementation of shooter IO. */
public class ShooterIOSim implements ShooterIO {
    private static final double LOOP_PERIOD_SEC = 0.02;
    private static final DCMotor SHOOTER_GEARBOX = DCMotor.getKrakenX60Foc(1);
    private static final DCMotor HOOD_GEARBOX = DCMotor.getKrakenX60Foc(1);
    private static final DCMotor KICKER_GEARBOX = DCMotor.getKrakenX60Foc(1);
    private static final double SHOOTER_KV_VOLTS_PER_RPM =
            ShooterConstants.MAX_OUTPUT_VOLTS / ShooterConstants.SHOOTER_MAX_RPM;
    private static final double AMBIENT_TEMP_C = 25.0;
    private static final double HOOD_HOMING_STATOR_CURRENT_AMPS =
            ShooterConstants.HOMING_CURRENT_THRESHOLD_AMPS + 5.0;
    private static final double HOOD_HOMING_STOP_ANGLE_RAD = ShooterConstants.HOOD_MIN_ANGLE_RAD + 0.001;
    // Tuned for similar setpoint settle behavior in sim vs real without instability.
    private static final double SIM_SHOOTER_KP = 0.02;
    private static final double SIM_SHOOTER_KI = 0.008;
    private static final double SIM_SHOOTER_KD = 0.0;
    private static final double SIM_HOOD_KP = 28.0;
    private static final double SIM_HOOD_KI = 0.0;
    private static final double SIM_HOOD_KD = 0.35;

    private final DCMotorSim leftShooterSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    SHOOTER_GEARBOX,
                    0.0025,
                    ShooterConstants.SHOOTER_SENSOR_TO_MECHANISM_RATIO),
            SHOOTER_GEARBOX);
    private final DCMotorSim rightShooterSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    SHOOTER_GEARBOX,
                    0.0025,
                    ShooterConstants.SHOOTER_SENSOR_TO_MECHANISM_RATIO),
            SHOOTER_GEARBOX);
    private final DCMotorSim hoodSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    HOOD_GEARBOX,
                    0.015,
                    ShooterConstants.HOOD_SENSOR_TO_MECHANISM_RATIO),
            HOOD_GEARBOX);
    private final DCMotorSim kickerSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    KICKER_GEARBOX,
                    0.002,
                    ShooterConstants.KICKER_SENSOR_TO_MECHANISM_RATIO),
            KICKER_GEARBOX);

    private final PIDController leftVelocityController =
            new PIDController(SIM_SHOOTER_KP, SIM_SHOOTER_KI, SIM_SHOOTER_KD);
    private final PIDController rightVelocityController =
            new PIDController(SIM_SHOOTER_KP, SIM_SHOOTER_KI, SIM_SHOOTER_KD);
    private final PIDController hoodPositionController =
            new PIDController(SIM_HOOD_KP, SIM_HOOD_KI, SIM_HOOD_KD);

    private enum KickerControlMode {
        OFF,
        TORQUE,
        VOLTAGE
    }

    private double targetLeftRpm = 0.0;
    private double targetRightRpm = 0.0;
    private double targetHoodAngleRad = ShooterConstants.HOOD_MIN_ANGLE_RAD;
    private double kickerOutput = 0.0;
    private KickerControlMode kickerMode = KickerControlMode.OFF;

    private boolean leftClosedLoop = false;
    private boolean rightClosedLoop = false;
    private boolean hoodClosedLoop = false;
    private boolean hoodHomingActive = false;
    private boolean hoodHomingAtStop = false;
    private int hoodHomingCycles = 0;

    private double leftOpenLoopVolts = 0.0;
    private double rightOpenLoopVolts = 0.0;
    private double leftAppliedVolts = 0.0;
    private double rightAppliedVolts = 0.0;
    private double hoodOpenLoopVolts = 0.0;
    private double hoodAppliedVolts = 0.0;
    private double kickerAppliedVolts = 0.0;
    private double hoodPositionOffsetRad = 0.0;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        leftAppliedVolts = leftClosedLoop
                ? SHOOTER_KV_VOLTS_PER_RPM * targetLeftRpm
                        + leftVelocityController.calculate(leftShooterSim.getAngularVelocityRPM(), targetLeftRpm)
                : leftOpenLoopVolts;
        leftAppliedVolts = clampOutputVolts(leftAppliedVolts);

        rightAppliedVolts = rightClosedLoop
                ? SHOOTER_KV_VOLTS_PER_RPM * targetRightRpm
                        + rightVelocityController.calculate(rightShooterSim.getAngularVelocityRPM(), targetRightRpm)
                : rightOpenLoopVolts;
        rightAppliedVolts = clampOutputVolts(rightAppliedVolts);

        double hoodPositionRad = hoodSim.getAngularPositionRad() - hoodPositionOffsetRad;
        double requestedHoodVolts = hoodClosedLoop
                ? hoodPositionController.calculate(hoodPositionRad, targetHoodAngleRad)
                : hoodOpenLoopVolts;
        if (hoodHomingActive) {
            hoodHomingCycles++;
            if (hoodHomingCycles > 1 && hoodPositionRad <= HOOD_HOMING_STOP_ANGLE_RAD) {
                hoodHomingAtStop = true;
            }
            requestedHoodVolts = hoodHomingAtStop ? 0.0 : requestedHoodVolts;
        }
        hoodAppliedVolts = requestedHoodVolts;
        hoodAppliedVolts = clampOutputVolts(hoodAppliedVolts);

        kickerAppliedVolts = switch (kickerMode) {
            case OFF -> 0.0;
            case TORQUE -> kickerOutput / ShooterConstants.KICKER_MAX_TORQUE_CURRENT_AMPS * ShooterConstants.MAX_OUTPUT_VOLTS;
            case VOLTAGE -> kickerOutput;
        };
        kickerAppliedVolts = clampOutputVolts(kickerAppliedVolts);

        leftShooterSim.setInputVoltage(leftAppliedVolts);
        rightShooterSim.setInputVoltage(rightAppliedVolts);
        hoodSim.setInputVoltage(hoodAppliedVolts);
        kickerSim.setInputVoltage(kickerAppliedVolts);

        leftShooterSim.update(LOOP_PERIOD_SEC);
        rightShooterSim.update(LOOP_PERIOD_SEC);
        hoodSim.update(LOOP_PERIOD_SEC);
        kickerSim.update(LOOP_PERIOD_SEC);

        inputs.shooterLeftPositionRad = leftShooterSim.getAngularPositionRad();
        inputs.shooterLeftVelocityRpm = leftShooterSim.getAngularVelocityRPM();
        inputs.shooterLeftAppliedVolts = leftAppliedVolts;
        inputs.shooterLeftSupplyCurrentAmps = Math.abs(leftShooterSim.getCurrentDrawAmps());
        inputs.shooterLeftTempCelsius = AMBIENT_TEMP_C;

        inputs.shooterRightPositionRad = rightShooterSim.getAngularPositionRad();
        inputs.shooterRightVelocityRpm = rightShooterSim.getAngularVelocityRPM();
        inputs.shooterRightAppliedVolts = rightAppliedVolts;
        inputs.shooterRightSupplyCurrentAmps = Math.abs(rightShooterSim.getCurrentDrawAmps());
        inputs.shooterRightTempCelsius = AMBIENT_TEMP_C;

        inputs.hoodPositionRad = hoodPositionRad;
        inputs.hoodVelocityRpm = hoodSim.getAngularVelocityRPM();
        inputs.hoodAppliedVolts = hoodAppliedVolts;
        double hoodSupplyCurrentAmps = Math.abs(hoodSim.getCurrentDrawAmps());
        inputs.hoodSupplyCurrentAmps = hoodSupplyCurrentAmps;
        inputs.hoodStatorCurrentAmps = hoodHomingActive && hoodHomingAtStop
                ? HOOD_HOMING_STATOR_CURRENT_AMPS
                : hoodSupplyCurrentAmps;
        inputs.hoodTempCelsius = AMBIENT_TEMP_C;

        inputs.kickerPositionRad = kickerSim.getAngularPositionRad();
        inputs.kickerVelocityRpm = kickerSim.getAngularVelocityRPM();
        inputs.kickerAppliedVolts = kickerAppliedVolts;
        inputs.kickerSupplyCurrentAmps = Math.abs(kickerSim.getCurrentDrawAmps());
        inputs.kickerTempCelsius = AMBIENT_TEMP_C;
    }

    @Override
    public void setLeftVelocity(double rpm) {
        targetLeftRpm = rpm;
        leftClosedLoop = true;
        leftOpenLoopVolts = 0.0;
    }

    @Override
    public void setRightVelocity(double rpm) {
        targetRightRpm = rpm;
        rightClosedLoop = true;
        rightOpenLoopVolts = 0.0;
    }

    @Override
    public void setLeftVoltage(double volts) {
        leftClosedLoop = false;
        leftOpenLoopVolts = volts;
    }

    @Override
    public void setRightVoltage(double volts) {
        rightClosedLoop = false;
        rightOpenLoopVolts = volts;
    }

    @Override
    public void setHoodAngle(double angle) {
        targetHoodAngleRad = angle;
        hoodClosedLoop = true;
        hoodOpenLoopVolts = 0.0;
        hoodHomingActive = false;
        hoodHomingAtStop = false;
        hoodHomingCycles = 0;
    }

    @Override
    public void setKickerTorque(double torqueCurrentAmps) {
        kickerOutput = torqueCurrentAmps;
        kickerMode = KickerControlMode.TORQUE;
    }

    @Override
    public void setKickerVoltage(double volts) {
        kickerOutput = volts;
        kickerMode = KickerControlMode.VOLTAGE;
    }

    @Override
    public void stop() {
        leftClosedLoop = false;
        rightClosedLoop = false;
        hoodClosedLoop = false;
        hoodOpenLoopVolts = 0.0;
        hoodHomingActive = false;
        hoodHomingAtStop = false;
        hoodHomingCycles = 0;
        kickerMode = KickerControlMode.OFF;

        targetLeftRpm = 0.0;
        targetRightRpm = 0.0;
        leftOpenLoopVolts = 0.0;
        rightOpenLoopVolts = 0.0;
        kickerOutput = 0.0;

        leftVelocityController.reset();
        rightVelocityController.reset();
        hoodPositionController.reset();
    }

    @Override
    public void resetHoodEncoder() {
        hoodPositionOffsetRad = hoodSim.getAngularPositionRad();
    }

    @Override
    public void setHoodVoltage(double volts) {
        hoodClosedLoop = false;
        hoodOpenLoopVolts = volts;
        hoodHomingActive = volts < 0.0;
        if (!hoodHomingActive) {
            hoodHomingAtStop = false;
            hoodHomingCycles = 0;
        } else {
            hoodHomingAtStop = false;
            hoodHomingCycles = 0;
        }
    }

    private static double clampOutputVolts(double volts) {
        return MathUtil.clamp(volts, -ShooterConstants.MAX_OUTPUT_VOLTS, ShooterConstants.MAX_OUTPUT_VOLTS);
    }
}
