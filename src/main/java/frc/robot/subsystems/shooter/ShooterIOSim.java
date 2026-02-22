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

    private final PIDController leftVelocityController = new PIDController(0.004, 0.0, 0.0);
    private final PIDController rightVelocityController = new PIDController(0.004, 0.0, 0.0);
    private final PIDController hoodPositionController = new PIDController(16.0, 0.0, 0.15);

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

    private double leftAppliedVolts = 0.0;
    private double rightAppliedVolts = 0.0;
    private double hoodAppliedVolts = 0.0;
    private double kickerAppliedVolts = 0.0;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        leftAppliedVolts = leftClosedLoop
                ? SHOOTER_KV_VOLTS_PER_RPM * targetLeftRpm
                        + leftVelocityController.calculate(leftShooterSim.getAngularVelocityRPM(), targetLeftRpm)
                : 0.0;
        leftAppliedVolts = clampOutputVolts(leftAppliedVolts);

        rightAppliedVolts = rightClosedLoop
                ? SHOOTER_KV_VOLTS_PER_RPM * targetRightRpm
                        + rightVelocityController.calculate(rightShooterSim.getAngularVelocityRPM(), targetRightRpm)
                : 0.0;
        rightAppliedVolts = clampOutputVolts(rightAppliedVolts);

        hoodAppliedVolts = hoodClosedLoop
                ? hoodPositionController.calculate(hoodSim.getAngularPositionRad(), targetHoodAngleRad)
                : 0.0;
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
        inputs.shooterLeftTempCelcius = AMBIENT_TEMP_C;

        inputs.shooterRightPositionRad = rightShooterSim.getAngularPositionRad();
        inputs.shooterRightVelocityRpm = rightShooterSim.getAngularVelocityRPM();
        inputs.shooterRightAppliedVolts = rightAppliedVolts;
        inputs.shooterRightSupplyCurrentAmps = Math.abs(rightShooterSim.getCurrentDrawAmps());
        inputs.shooterRightTempCelcius = AMBIENT_TEMP_C;

        inputs.hoodPositionRad = hoodSim.getAngularPositionRad();
        inputs.hoodVelocityRpm = hoodSim.getAngularVelocityRPM();
        inputs.hoodAppliedVolts = hoodAppliedVolts;
        inputs.hoodSupplyCurrentAmps = Math.abs(hoodSim.getCurrentDrawAmps());
        inputs.hoodTempCelcius = AMBIENT_TEMP_C;

        inputs.kickerPositionRad = kickerSim.getAngularPositionRad();
        inputs.kickerVelocityRpm = kickerSim.getAngularVelocityRPM();
        inputs.kickerAppliedVolts = kickerAppliedVolts;
        inputs.kickerSupplyCurrentAmps = Math.abs(kickerSim.getCurrentDrawAmps());
        inputs.kickerTempCelcius = AMBIENT_TEMP_C;
    }

    @Override
    public void setLeftVelocity(double rpm) {
        targetLeftRpm = rpm;
        leftClosedLoop = true;
    }

    @Override
    public void setRightVelocity(double rpm) {
        targetRightRpm = rpm;
        rightClosedLoop = true;
    }

    @Override
    public void setHoodAngle(double angle) {
        targetHoodAngleRad = angle;
        hoodClosedLoop = true;
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
        kickerMode = KickerControlMode.OFF;

        targetLeftRpm = 0.0;
        targetRightRpm = 0.0;
        kickerOutput = 0.0;

        leftVelocityController.reset();
        rightVelocityController.reset();
        hoodPositionController.reset();
    }

    private static double clampOutputVolts(double volts) {
        return MathUtil.clamp(volts, -ShooterConstants.MAX_OUTPUT_VOLTS, ShooterConstants.MAX_OUTPUT_VOLTS);
    }
}
