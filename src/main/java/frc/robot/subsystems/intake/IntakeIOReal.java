package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
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
    private final DynamicMotionMagicVoltage leftPositionRequest = new DynamicMotionMagicVoltage(0.0, IntakeConstants.INTAKE_VELOCITY,IntakeConstants.INTAKE_ACCELERATION);
    private final VoltageOut leftVoltageRequest = new VoltageOut(0.0);
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
    public void retract() {
        setLeftCurrentLimits(true);
        leftPositionRequest.Velocity = IntakeConstants.INTAKE_VELOCITY;
        leftPositionRequest.Acceleration = IntakeConstants.INTAKE_ACCELERATION;
        leftIntakeMotor.setControl(leftPositionRequest.withPosition(IntakeConstants.RETRACTED_POSITION_ROT));
    }

    @Override
    public void extend() {
        setLeftCurrentLimits(true);
        leftPositionRequest.Velocity = IntakeConstants.INTAKE_VELOCITY;
        leftPositionRequest.Acceleration = IntakeConstants.INTAKE_ACCELERATION;
        leftIntakeMotor.setControl(leftPositionRequest.withPosition(IntakeConstants.EXTENDED_POSITION_ROT));
        setLeftCurrentLimits(false);
    }

    @Override
    public void slowRetract() {
        setLeftCurrentLimits(true);
        leftPositionRequest.Velocity = IntakeConstants.SLOW_INTAKE_VELOCITY;
        leftPositionRequest.Acceleration = IntakeConstants.SLOW_INTAKE_ACCELERATION;
        leftIntakeMotor.setControl(leftPositionRequest.withPosition(IntakeConstants.RETRACTED_POSITION_ROT));
    }

    @Override
    public void resetEncoders() {
        leftIntakeMotor.setPosition(0.0);
        rightIntakeMotor.setPosition(0.0);
    }

    @Override
    public void home() {
        setLeftCurrentLimits(true);
        leftIntakeMotor.setControl(leftVoltageRequest.withOutput(-1.0));
    }

    @Override
    public void stop() {
        leftIntakeMotor.setControl(neutralRequest);
        rightIntakeMotor.setControl(neutralRequest);
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
        config.CurrentLimits.StatorCurrentLimit = IntakeConstants.INTAKE_STATOR_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.INTAKE_SUPPLY_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        tryUntilOk(5, () -> leftIntakeMotor.getConfigurator().apply(config, 0.25));
    }

    private void setLeftCurrentLimits(boolean high) {
        var limits = new CurrentLimitsConfigs();
        limits.StatorCurrentLimit = high
                ? IntakeConstants.INTAKE_STATOR_CURRENT_LIMIT_AMPS
                : IntakeConstants.INTAKE_STATOR_CURRENT_LIMIT_AMPS_LOW;
        limits.StatorCurrentLimitEnable = true;
        limits.SupplyCurrentLimit = high
                ? IntakeConstants.INTAKE_SUPPLY_CURRENT_LIMIT_AMPS
                : IntakeConstants.INTAKE_SUPPLY_CURRENT_LIMIT_AMPS_LOW;
        limits.SupplyCurrentLimitEnable = true;
        tryUntilOk(5, () -> leftIntakeMotor.getConfigurator().apply(limits, 0.25));
    }

    private void configureRightMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = IntakeConstants.INTAKE_STATOR_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.INTAKE_SUPPLY_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        tryUntilOk(5, () -> rightIntakeMotor.getConfigurator().apply(config, 0.25));
        rightIntakeMotor.setControl(new Follower(
                leftIntakeMotor.getDeviceID(),
                IntakeConstants.RIGHT_OPPOSES_LEFT ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
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
