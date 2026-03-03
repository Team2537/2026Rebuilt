package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {
    private static final CANBus MECHANISM_CAN_BUS = new CANBus(Constants.MECHANISM_CAN_BUS);

    private final TalonFX rollerMotor = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID, MECHANISM_CAN_BUS);
    private final TalonFX leftIntakeMotor = new TalonFX(IntakeConstants.LEFT_MOTOR_ID, MECHANISM_CAN_BUS);
    private final TalonFX rightIntakeMotor = new TalonFX(IntakeConstants.RIGHT_MOTOR_ID, MECHANISM_CAN_BUS);

    private final VelocityTorqueCurrentFOC rollerVelocityRequest = new VelocityTorqueCurrentFOC(0.0);
    private final DynamicMotionMagicVoltage leftPositionRequest =
            new DynamicMotionMagicVoltage(
                    0.0, IntakeConstants.INTAKE_VELOCITY, IntakeConstants.INTAKE_ACCELERATION);
    private final DynamicMotionMagicVoltage rightPositionRequest =
            new DynamicMotionMagicVoltage(
                    0.0, IntakeConstants.INTAKE_VELOCITY, IntakeConstants.INTAKE_ACCELERATION);
    private final VoltageOut leftVoltageRequest = new VoltageOut(0.0);
    private final VoltageOut rightVoltageRequest = new VoltageOut(0.0);
    private final NeutralOut neutralRequest = new NeutralOut();

    private final StatusSignal<?> leftPosition;
    private final StatusSignal<?> leftVelocity;
    private final StatusSignal<?> leftAppliedVolts;
    private final StatusSignal<?> leftSupplyCurrent;
    private final StatusSignal<?> leftStatorCurrent;

    private final StatusSignal<?> rightPosition;
    private final StatusSignal<?> rightVelocity;
    private final StatusSignal<?> rightAppliedVolts;
    private final StatusSignal<?> rightSupplyCurrent;
    private final StatusSignal<?> rightStatorCurrent;

    private final StatusSignal<?> rollerPosition;
    private final StatusSignal<?> rollerVelocity;
    private final StatusSignal<?> rollerAppliedVolts;
    private final StatusSignal<?> rollerSupplyCurrent;
    private final StatusSignal<?> rollerStatorCurrent;

    public IntakeIOReal() {
        configureLeftMotor();
        configureRightMotor();
        configureRollerMotor();

        rollerVelocityRequest.Slot = 0;

        leftIntakeMotor.setPosition(0.0);
        rightIntakeMotor.setPosition(0.0);

        leftPosition = leftIntakeMotor.getPosition();
        leftVelocity = leftIntakeMotor.getVelocity();
        leftAppliedVolts = leftIntakeMotor.getMotorVoltage();
        leftSupplyCurrent = leftIntakeMotor.getSupplyCurrent();
        leftStatorCurrent = leftIntakeMotor.getStatorCurrent();

        rightPosition = rightIntakeMotor.getPosition();
        rightVelocity = rightIntakeMotor.getVelocity();
        rightAppliedVolts = rightIntakeMotor.getMotorVoltage();
        rightSupplyCurrent = rightIntakeMotor.getSupplyCurrent();
        rightStatorCurrent = rightIntakeMotor.getStatorCurrent();

        rollerPosition = rollerMotor.getPosition();
        rollerVelocity = rollerMotor.getVelocity();
        rollerAppliedVolts = rollerMotor.getMotorVoltage();
        rollerSupplyCurrent = rollerMotor.getSupplyCurrent();
        rollerStatorCurrent = rollerMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                IntakeConstants.STATUS_UPDATE_HZ,
                leftPosition,
                leftVelocity,
                leftAppliedVolts,
                leftSupplyCurrent,
                leftStatorCurrent,
                rightPosition,
                rightVelocity,
                rightAppliedVolts,
                rightSupplyCurrent,
                rightStatorCurrent,
                rollerPosition,
                rollerVelocity,
                rollerAppliedVolts,
                rollerSupplyCurrent,
                rollerStatorCurrent);
        ParentDevice.optimizeBusUtilizationForAll(leftIntakeMotor, rightIntakeMotor, rollerMotor);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                leftPosition,
                leftVelocity,
                leftAppliedVolts,
                leftSupplyCurrent,
                leftStatorCurrent,
                rightPosition,
                rightVelocity,
                rightAppliedVolts,
                rightSupplyCurrent,
                rightStatorCurrent,
                rollerPosition,
                rollerVelocity,
                rollerAppliedVolts,
                rollerSupplyCurrent,
                rollerStatorCurrent);

        inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
        inputs.leftPositionRad = Units.rotationsToRadians(leftPosition.getValueAsDouble());
        inputs.leftSupplyCurrentAmps = leftSupplyCurrent.getValueAsDouble();
        inputs.leftStatorCurrentAmps = leftStatorCurrent.getValueAsDouble();
        inputs.leftVelocityRpm = leftVelocity.getValueAsDouble() * 60.0;

        inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
        inputs.rightPositionRad = Units.rotationsToRadians(rightPosition.getValueAsDouble());
        inputs.rightSupplyCurrentAmps = rightSupplyCurrent.getValueAsDouble();
        inputs.rightStatorCurrentAmps = rightStatorCurrent.getValueAsDouble();
        inputs.rightVelocityRpm = rightVelocity.getValueAsDouble() * 60.0;

        inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
        inputs.rollerPositionRad = Units.rotationsToRadians(rollerPosition.getValueAsDouble());
        inputs.rollerSupplyCurrentAmps = rollerSupplyCurrent.getValueAsDouble();
        inputs.rollerStatorCurrentAmps = rollerStatorCurrent.getValueAsDouble();
        inputs.rollerVelocityRpm = rollerVelocity.getValueAsDouble() * 60.0;

    }

    @Override
    public void setRollerRpm(double rpm) {
        double clampedRpm = MathUtil.clamp(rpm, -IntakeConstants.ROLLER_MAX_RPM, IntakeConstants.ROLLER_MAX_RPM);
        rollerMotor.setControl(rollerVelocityRequest.withVelocity(clampedRpm / 60.0));
    }

    @Override
    public void setIntakePosition(
            double leftTargetRot,
            double velocityRotPerSec,
            double accelerationRotPerSecSq,
            double maxVolts) {
        setIntakePositionInternal(leftTargetRot, velocityRotPerSec, accelerationRotPerSecSq, maxVolts);
    }

    @Override
    public void resetEncoders() {
        leftIntakeMotor.setPosition(0.0);
        rightIntakeMotor.setPosition(0.0);
    }

    @Override
    public void setLeftIntakeVoltage(double volts) {
        leftIntakeMotor.setControl(leftVoltageRequest.withOutput(MathUtil.clamp(volts, -12.0, 12.0)));
    }

    @Override
    public void setRightIntakeVoltage(double volts) {
        rightIntakeMotor.setControl(rightVoltageRequest.withOutput(MathUtil.clamp(volts, -12.0, 12.0)));
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
        leftIntakeMotor.setControl(neutralRequest);
    }

    @Override
    public void stopRightIntake() {
        rightIntakeMotor.setControl(neutralRequest);
    }

    @Override
    public void stopRoller() {
        rollerMotor.setControl(neutralRequest);
    }

    private void configureLeftMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = IntakeConstants.LEFT_INTAKE_INVERTED
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.Slot0 = new Slot0Configs()
                .withKP(IntakeConstants.INTAKE_KP)
                .withKI(IntakeConstants.INTAKE_KI)
                .withKD(IntakeConstants.INTAKE_KD)
                .withKS(IntakeConstants.INTAKE_KS)
                .withKV(IntakeConstants.INTAKE_KV);
        config.MotionMagic = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(IntakeConstants.INTAKE_VELOCITY)
                .withMotionMagicAcceleration(IntakeConstants.INTAKE_ACCELERATION);
        config.CurrentLimits.StatorCurrentLimit = IntakeConstants.INTAKE_STATOR_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.INTAKE_SUPPLY_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        tryUntilOk(5, () -> leftIntakeMotor.getConfigurator().apply(config, 0.25));
    }

    private void setIntakePositionInternal(double leftTargetRot, double velocity, double acceleration, double maxVolts) {
        setIntakeMotionProfile(velocity, acceleration);
        setIntakeVoltageLimit(maxVolts);
        leftPositionRequest.withVelocity(velocity).withAcceleration(acceleration);
        rightPositionRequest.withVelocity(velocity).withAcceleration(acceleration);
        leftIntakeMotor.setControl(leftPositionRequest.withPosition(leftTargetRot));
        rightIntakeMotor.setControl(rightPositionRequest.withPosition(applyRightAlignment(leftTargetRot)));
    }

    private void setIntakeMotionProfile(double velocity, double acceleration) {
        if (lastIntakeVelocity == velocity && lastIntakeAcceleration == acceleration) {
            return;
        }

        var motionMagicConfig = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(velocity)
                .withMotionMagicAcceleration(acceleration);
        tryUntilOk(5, () -> leftIntakeMotor.getConfigurator().apply(motionMagicConfig, 0.25));
        tryUntilOk(5, () -> rightIntakeMotor.getConfigurator().apply(motionMagicConfig, 0.25));
        lastIntakeVelocity = velocity;
        lastIntakeAcceleration = acceleration;
    }

    private double applyRightAlignment(double leftReference) {
        return IntakeConstants.RIGHT_OPPOSES_LEFT ? leftReference : -leftReference;
    }

    private double lastIntakeVelocity = Double.NaN;
    private double lastIntakeAcceleration = Double.NaN;
    private double lastIntakeMaxVolts = Double.NaN;

    private void setIntakeVoltageLimit(double maxVolts) {
        double clampedVolts = MathUtil.clamp(Math.abs(maxVolts), 0.0, 12.0);
        if (lastIntakeMaxVolts == clampedVolts) {
            return;
        }
        VoltageConfigs config = new VoltageConfigs()
                .withPeakForwardVoltage(clampedVolts)
                .withPeakReverseVoltage(-clampedVolts);
        tryUntilOk(5, () -> leftIntakeMotor.getConfigurator().apply(config, 0.25));
        tryUntilOk(5, () -> rightIntakeMotor.getConfigurator().apply(config, 0.25));
        lastIntakeMaxVolts = clampedVolts;
    }

    private void configureRightMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0 = new Slot0Configs()
                .withKP(IntakeConstants.INTAKE_KP)
                .withKI(IntakeConstants.INTAKE_KI)
                .withKD(IntakeConstants.INTAKE_KD)
                .withKS(IntakeConstants.INTAKE_KS)
                .withKV(IntakeConstants.INTAKE_KV);
        config.CurrentLimits.StatorCurrentLimit = IntakeConstants.INTAKE_STATOR_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.INTAKE_SUPPLY_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotionMagic = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(IntakeConstants.INTAKE_VELOCITY)
                .withMotionMagicAcceleration(IntakeConstants.INTAKE_ACCELERATION);
        tryUntilOk(5, () -> rightIntakeMotor.getConfigurator().apply(config, 0.25));
    }

    private void configureRollerMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = IntakeConstants.ROLLER_INVERTED
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = IntakeConstants.ROLLER_SENSOR_TO_MECHANISM_RATIO;
        config.Slot0 = new Slot0Configs()
                .withKP(IntakeConstants.ROLLER_KP)
                .withKI(IntakeConstants.ROLLER_KI)
                .withKD(IntakeConstants.ROLLER_KD)
                .withKS(IntakeConstants.ROLLER_KS)
                .withKV(IntakeConstants.ROLLER_KV);
        config.CurrentLimits.StatorCurrentLimit = IntakeConstants.ROLLER_STATOR_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.ROLLER_SUPPLY_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(config, 0.25));
    }
}
