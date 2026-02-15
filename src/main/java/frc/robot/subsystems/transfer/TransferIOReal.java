package frc.robot.subsystems.transfer;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

public class TransferIOReal implements TransferIO {
    private final TalonFX transferMotor = new TalonFX(TransferConstants.TRANSFER_MOTOR_ID, TunerConstants.kCANBus);

    private final DutyCycleOut percentRequest = new DutyCycleOut(0.0);
    private final NeutralOut neutralRequest = new NeutralOut();

    private final StatusSignal<?> position;
    private final StatusSignal<?> velocity;
    private final StatusSignal<?> appliedVolts;
    private final StatusSignal<?> supplyCurrent;
    private final StatusSignal<?> temp;

    public TransferIOReal() {
        configureMotor();

        position = transferMotor.getPosition();
        velocity = transferMotor.getVelocity();
        appliedVolts = transferMotor.getMotorVoltage();
        supplyCurrent = transferMotor.getSupplyCurrent();
        temp = transferMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                TransferConstants.STATUS_UPDATE_HZ, position, velocity, appliedVolts, supplyCurrent, temp);
        ParentDevice.optimizeBusUtilizationForAll(transferMotor);
    }

    @Override
    public void updateInputs(TransferIOInputs inputs) {
        BaseStatusSignal.refreshAll(position, velocity, appliedVolts, supplyCurrent, temp);

        inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
        inputs.velocityRpm = velocity.getValueAsDouble() * 60.0;
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.tempCelcius = temp.getValueAsDouble();
    }

    @Override
    public void setPercent(double percent) {
        transferMotor.setControl(percentRequest.withOutput(MathUtil.clamp(percent, -1.0, 1.0)));
    }

    @Override
    public void stop() {
        transferMotor.setControl(neutralRequest);
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = TransferConstants.TRANSFER_INVERTED
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = TransferConstants.SENSOR_TO_MECHANISM_RATIO;
        config.CurrentLimits.StatorCurrentLimit = TransferConstants.STATOR_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = TransferConstants.SUPPLY_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        tryUntilOk(5, () -> transferMotor.getConfigurator().apply(config, 0.25));
    }
}
