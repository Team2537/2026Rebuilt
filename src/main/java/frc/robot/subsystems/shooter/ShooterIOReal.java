package frc.robot.subsystems.shooter;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ShooterIOReal implements ShooterIO {
    private static final CANBus MECHANISM_CAN_BUS = new CANBus(Constants.MECHANISM_CAN_BUS);

    private final TalonFX leftShooterMotor =
            new TalonFX(ShooterConstants.LEFT_SHOOTER_MOTOR_ID, MECHANISM_CAN_BUS);
    private final TalonFX rightShooterMotor =
            new TalonFX(ShooterConstants.RIGHT_SHOOTER_MOTOR_ID, MECHANISM_CAN_BUS);
    private final TalonFX hoodMotor = new TalonFX(ShooterConstants.HOOD_MOTOR_ID, MECHANISM_CAN_BUS);
    private final TalonFX kickerMotor = new TalonFX(ShooterConstants.KICKER_MOTOR_ID, MECHANISM_CAN_BUS);

    private final VelocityTorqueCurrentFOC leftVelocityRequest = new VelocityTorqueCurrentFOC(0.0);
    private final VelocityTorqueCurrentFOC rightVelocityRequest = new VelocityTorqueCurrentFOC(0.0);
    private final PositionVoltage hoodPositionRequest = new PositionVoltage(0.0);
    private final TorqueCurrentFOC kickerTorqueRequest = new TorqueCurrentFOC(0.0);
    private final VoltageOut kickerVoltageRequest = new VoltageOut(0.0);
    private final NeutralOut neutralRequest = new NeutralOut();

    private final StatusSignal<?> leftPosition;
    private final StatusSignal<?> leftVelocity;
    private final StatusSignal<?> leftAppliedVolts;
    private final StatusSignal<?> leftSupplyCurrent;
    private final StatusSignal<?> leftTemp;

    private final StatusSignal<?> rightPosition;
    private final StatusSignal<?> rightVelocity;
    private final StatusSignal<?> rightAppliedVolts;
    private final StatusSignal<?> rightSupplyCurrent;
    private final StatusSignal<?> rightTemp;

    private final StatusSignal<?> hoodPosition;
    private final StatusSignal<?> hoodVelocity;
    private final StatusSignal<?> hoodAppliedVolts;
    private final StatusSignal<?> hoodSupplyCurrent;
    private final StatusSignal<?> hoodTemp;

    private final StatusSignal<?> kickerPosition;
    private final StatusSignal<?> kickerVelocity;
    private final StatusSignal<?> kickerAppliedVolts;
    private final StatusSignal<?> kickerSupplyCurrent;
    private final StatusSignal<?> kickerTemp;

    public ShooterIOReal() {
        configureShooterMotor(leftShooterMotor, ShooterConstants.LEFT_SHOOTER_INVERTED);
        configureShooterMotor(rightShooterMotor, ShooterConstants.RIGHT_SHOOTER_INVERTED);
        configureHoodMotor();
        configureKickerMotor();

        leftVelocityRequest.Slot = 0;
        rightVelocityRequest.Slot = 0;
        hoodPositionRequest.Slot = 0;
        hoodPositionRequest.EnableFOC = true;
        kickerTorqueRequest.Deadband = 1.0;
        kickerTorqueRequest.MaxAbsDutyCycle = 1.0;

        leftPosition = leftShooterMotor.getPosition();
        leftVelocity = leftShooterMotor.getVelocity();
        leftAppliedVolts = leftShooterMotor.getMotorVoltage();
        leftSupplyCurrent = leftShooterMotor.getSupplyCurrent();
        leftTemp = leftShooterMotor.getDeviceTemp();

        rightPosition = rightShooterMotor.getPosition();
        rightVelocity = rightShooterMotor.getVelocity();
        rightAppliedVolts = rightShooterMotor.getMotorVoltage();
        rightSupplyCurrent = rightShooterMotor.getSupplyCurrent();
        rightTemp = rightShooterMotor.getDeviceTemp();

        hoodPosition = hoodMotor.getPosition();
        hoodVelocity = hoodMotor.getVelocity();
        hoodAppliedVolts = hoodMotor.getMotorVoltage();
        hoodSupplyCurrent = hoodMotor.getSupplyCurrent();
        hoodTemp = hoodMotor.getDeviceTemp();

        kickerPosition = kickerMotor.getPosition();
        kickerVelocity = kickerMotor.getVelocity();
        kickerAppliedVolts = kickerMotor.getMotorVoltage();
        kickerSupplyCurrent = kickerMotor.getSupplyCurrent();
        kickerTemp = kickerMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                ShooterConstants.STATUS_UPDATE_HZ,
                leftPosition,
                leftVelocity,
                leftAppliedVolts,
                leftSupplyCurrent,
                leftTemp,
                rightPosition,
                rightVelocity,
                rightAppliedVolts,
                rightSupplyCurrent,
                rightTemp,
                hoodPosition,
                hoodVelocity,
                hoodAppliedVolts,
                hoodSupplyCurrent,
                hoodTemp,
                kickerPosition,
                kickerVelocity,
                kickerAppliedVolts,
                kickerSupplyCurrent,
                kickerTemp);
        ParentDevice.optimizeBusUtilizationForAll(leftShooterMotor, rightShooterMotor, hoodMotor, kickerMotor);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                leftPosition,
                leftVelocity,
                leftAppliedVolts,
                leftSupplyCurrent,
                leftTemp,
                rightPosition,
                rightVelocity,
                rightAppliedVolts,
                rightSupplyCurrent,
                rightTemp,
                hoodPosition,
                hoodVelocity,
                hoodAppliedVolts,
                hoodSupplyCurrent,
                hoodTemp,
                kickerPosition,
                kickerVelocity,
                kickerAppliedVolts,
                kickerSupplyCurrent,
                kickerTemp);

        inputs.shooterLeftPositionRad = Units.rotationsToRadians(leftPosition.getValueAsDouble());
        inputs.shooterLeftVelocityRpm = leftVelocity.getValueAsDouble() * 60.0;
        inputs.shooterLeftAppliedVolts = leftAppliedVolts.getValueAsDouble();
        inputs.shooterLeftSupplyCurrentAmps = leftSupplyCurrent.getValueAsDouble();
        inputs.shooterLeftTempCelcius = leftTemp.getValueAsDouble();

        inputs.shooterRightPositionRad = Units.rotationsToRadians(rightPosition.getValueAsDouble());
        inputs.shooterRightVelocityRpm = rightVelocity.getValueAsDouble() * 60.0;
        inputs.shooterRightAppliedVolts = rightAppliedVolts.getValueAsDouble();
        inputs.shooterRightSupplyCurrentAmps = rightSupplyCurrent.getValueAsDouble();
        inputs.shooterRightTempCelcius = rightTemp.getValueAsDouble();

        inputs.hoodPositionRad = Units.rotationsToRadians(hoodPosition.getValueAsDouble());
        inputs.hoodVelocityRpm = hoodVelocity.getValueAsDouble() * 60.0;
        inputs.hoodAppliedVolts = hoodAppliedVolts.getValueAsDouble();
        inputs.hoodSupplyCurrentAmps = hoodSupplyCurrent.getValueAsDouble();
        inputs.hoodTempCelcius = hoodTemp.getValueAsDouble();

        inputs.kickerPositionRad = Units.rotationsToRadians(kickerPosition.getValueAsDouble());
        inputs.kickerVelocityRpm = kickerVelocity.getValueAsDouble() * 60.0;
        inputs.kickerAppliedVolts = kickerAppliedVolts.getValueAsDouble();
        inputs.kickerSupplyCurrentAmps = kickerSupplyCurrent.getValueAsDouble();
        inputs.kickerTempCelcius = kickerTemp.getValueAsDouble();
    }

    @Override
    public void setLeftVelocity(double rpm) {
        double clampedRpm = MathUtil.clamp(rpm, -ShooterConstants.SHOOTER_MAX_RPM, ShooterConstants.SHOOTER_MAX_RPM);
        leftShooterMotor.setControl(leftVelocityRequest.withVelocity(clampedRpm / 60.0));
    }

    @Override
    public void setRightVelocity(double rpm) {
        double clampedRpm = MathUtil.clamp(rpm, -ShooterConstants.SHOOTER_MAX_RPM, ShooterConstants.SHOOTER_MAX_RPM);
        rightShooterMotor.setControl(rightVelocityRequest.withVelocity(clampedRpm / 60.0));
    }

    @Override
    public void setHoodAngle(double angle) {
        double clampedAngle = MathUtil.clamp(
                angle,
                ShooterConstants.HOOD_MIN_ANGLE_RAD,
                ShooterConstants.HOOD_MAX_ANGLE_RAD);
        hoodMotor.setControl(hoodPositionRequest.withPosition(Units.radiansToRotations(clampedAngle)));
    }

    @Override
    public void setKickerTorque(double torque) {
        double clampedTorque = MathUtil.clamp(
                torque,
                -ShooterConstants.KICKER_MAX_TORQUE_CURRENT_AMPS,
                ShooterConstants.KICKER_MAX_TORQUE_CURRENT_AMPS);
        kickerMotor.setControl(kickerTorqueRequest.withOutput(clampedTorque));
    }

    @Override
    public void setKickerVoltage(double volts) {
        double clampedVolts = MathUtil.clamp(
                volts,
                -ShooterConstants.MAX_OUTPUT_VOLTS,
                ShooterConstants.MAX_OUTPUT_VOLTS);
        kickerMotor.setControl(kickerVoltageRequest.withOutput(clampedVolts));
    }

    @Override
    public void stop() {
        leftShooterMotor.setControl(neutralRequest);
        rightShooterMotor.setControl(neutralRequest);
        hoodMotor.setControl(neutralRequest);
        kickerMotor.setControl(neutralRequest);
    }

    private void configureShooterMotor(TalonFX motor, boolean inverted) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = inverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = ShooterConstants.SHOOTER_SENSOR_TO_MECHANISM_RATIO;
        config.Slot0 = new Slot0Configs()
                .withKP(ShooterConstants.SHOOTER_KP)
                .withKI(ShooterConstants.SHOOTER_KI)
                .withKD(ShooterConstants.SHOOTER_KD)
                .withKS(ShooterConstants.SHOOTER_KS)
                .withKV(ShooterConstants.SHOOTER_KV);
        config.CurrentLimits.StatorCurrentLimit = ShooterConstants.SHOOTER_STATOR_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SHOOTER_SUPPLY_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));
    }

    private void configureHoodMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = ShooterConstants.HOOD_INVERTED
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = ShooterConstants.HOOD_SENSOR_TO_MECHANISM_RATIO;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                Units.radiansToRotations(ShooterConstants.HOOD_MIN_ANGLE_RAD);
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                Units.radiansToRotations(ShooterConstants.HOOD_MAX_ANGLE_RAD);
        config.Slot0 = new Slot0Configs()
                .withKP(ShooterConstants.HOOD_KP)
                .withKI(ShooterConstants.HOOD_KI)
                .withKD(ShooterConstants.HOOD_KD)
                .withKS(ShooterConstants.HOOD_KS)
                .withKV(ShooterConstants.HOOD_KV);
        config.CurrentLimits.StatorCurrentLimit = ShooterConstants.HOOD_STATOR_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.HOOD_SUPPLY_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(config, 0.25));
    }

    private void configureKickerMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = ShooterConstants.KICKER_INVERTED
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = ShooterConstants.KICKER_SENSOR_TO_MECHANISM_RATIO;
        config.CurrentLimits.StatorCurrentLimit = ShooterConstants.KICKER_STATOR_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.KICKER_SUPPLY_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.TorqueCurrent.PeakForwardTorqueCurrent = ShooterConstants.KICKER_MAX_TORQUE_CURRENT_AMPS;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -ShooterConstants.KICKER_MAX_TORQUE_CURRENT_AMPS;
        tryUntilOk(5, () -> kickerMotor.getConfigurator().apply(config, 0.25));
    }
}
