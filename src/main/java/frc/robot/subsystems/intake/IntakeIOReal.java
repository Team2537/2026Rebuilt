package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

public class IntakeIOReal implements IntakeIO {
    private final TalonFX rollerMotor = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID, TunerConstants.kCANBus);
    private final TalonFX leftIntakeMotor = new TalonFX(IntakeConstants.LEFT_MOTOR_ID, TunerConstants.kCANBus);
    private final TalonFX rightIntakeMotor = new TalonFX(IntakeConstants.RIGHT_MOTOR_ID, TunerConstants.kCANBus);

    private final VelocityTorqueCurrentFOC rollerVelocityRequest = new VelocityTorqueCurrentFOC(0.0);
    private final MotionMagicVoltage leftPositionRequest = new MotionMagicVoltage(0.0);
    private final NeutralOut neutralRequest = new NeutralOut();

    private final StatusSignal<?> leftPosition;
    private final StatusSignal<?> leftVelocity;
    private final StatusSignal<?> leftAppliedVolts;
    private final StatusSignal<?> leftSupplyCurrent;

    private final StatusSignal<?> rightPosition;
    private final StatusSignal<?> rightVelocity;
    private final StatusSignal<?> rightAppliedVolts;
    private final StatusSignal<?> rightSupplyCurrent;

    private final StatusSignal<?> rollerPosition;
    private final StatusSignal<?> rollerVelocity;
    private final StatusSignal<?> rollerAppliedVolts;
    private final StatusSignal<?> rollerSupplyCurrent;

    public IntakeIOReal() {
        configureLeftMotor();
        configureRightMotor();
        configureRollerMotor();

        leftIntakeMotor.setPosition(0.0);
        rightIntakeMotor.setPosition(0.0);

        leftPosition = leftIntakeMotor.getPosition();
        leftVelocity = leftIntakeMotor.getVelocity();
        leftAppliedVolts = leftIntakeMotor.getMotorVoltage();
        leftSupplyCurrent = leftIntakeMotor.getSupplyCurrent();

        rightPosition = rightIntakeMotor.getPosition();
        rightVelocity = rightIntakeMotor.getVelocity();
        rightAppliedVolts = rightIntakeMotor.getMotorVoltage();
        rightSupplyCurrent = rightIntakeMotor.getSupplyCurrent();

        rollerPosition = rollerMotor.getPosition();
        rollerVelocity = rollerMotor.getVelocity();
        rollerAppliedVolts = rollerMotor.getMotorVoltage();
        rollerSupplyCurrent = rollerMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                IntakeConstants.STATUS_UPDATE_HZ,
                leftPosition,
                leftVelocity,
                leftAppliedVolts,
                leftSupplyCurrent,
                rightPosition,
                rightVelocity,
                rightAppliedVolts,
                rightSupplyCurrent,
                rollerPosition,
                rollerVelocity,
                rollerAppliedVolts,
                rollerSupplyCurrent);
        ParentDevice.optimizeBusUtilizationForAll(leftIntakeMotor, rightIntakeMotor, rollerMotor);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                leftPosition,
                leftVelocity,
                leftAppliedVolts,
                leftSupplyCurrent,
                rightPosition,
                rightVelocity,
                rightAppliedVolts,
                rightSupplyCurrent,
                rollerPosition,
                rollerVelocity,
                rollerAppliedVolts,
                rollerSupplyCurrent);

        inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
        inputs.leftPositionRad = Units.rotationsToRadians(leftPosition.getValueAsDouble());
        inputs.leftCurrentAmps = leftSupplyCurrent.getValueAsDouble();
        inputs.leftVelocityRpm = leftVelocity.getValueAsDouble() * 60.0;

        inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
        inputs.rightPositionRad = Units.rotationsToRadians(rightPosition.getValueAsDouble());
        inputs.rightCurrentAmps = rightSupplyCurrent.getValueAsDouble();
        inputs.rightVelocityRpm = rightVelocity.getValueAsDouble() * 60.0;

        inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
        inputs.rollerPositionRad = Units.rotationsToRadians(rollerPosition.getValueAsDouble());
        inputs.rollerCurrentAmps = rollerSupplyCurrent.getValueAsDouble();
        inputs.rollerVelocityRpm = rollerVelocity.getValueAsDouble() * 60.0;
    }

    @Override
    public void setRollerRpm(double rpm) {
        rollerMotor.setControl(rollerVelocityRequest.withVelocity(rpm / 60.0));
    }

    @Override
    public void retract() {
        leftIntakeMotor.setControl(leftPositionRequest.withPosition(IntakeConstants.RETRACTED_POSITION_ROT));
    }

    @Override
    public void extend() {
        leftIntakeMotor.setControl(leftPositionRequest.withPosition(IntakeConstants.EXTENDED_POSITION_ROT));
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
        config.MotionMagic = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(IntakeConstants.INTAKE_VELOCITY)
                .withMotionMagicAcceleration(IntakeConstants.INTAKE_ACCELERATION);
        config.CurrentLimits.StatorCurrentLimit = IntakeConstants.INTAKE_STATOR_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.INTAKE_SUPPLY_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        tryUntilOk(5, () -> leftIntakeMotor.getConfigurator().apply(config, 0.25));
    }

    private void configureRightMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
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
        config.CurrentLimits.StatorCurrentLimit = IntakeConstants.ROLLER_STATOR_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.ROLLER_SUPPLY_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(config, 0.25));
    }
}
