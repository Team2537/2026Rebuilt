package frc.robot.subsystems.transfer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Simulation implementation of transfer IO. */
public class TransferIOSim implements TransferIO {
    private static final double LOOP_PERIOD_SEC = 0.02;
    private static final double MAX_VOLTS = 12.0;
    private static final double AMBIENT_TEMP_C = 25.0;
    private static final DCMotor TRANSFER_GEARBOX = DCMotor.getKrakenX60Foc(1);

    private final DCMotorSim transferSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    TRANSFER_GEARBOX,
                    0.003,
                    TransferConstants.SENSOR_TO_MECHANISM_RATIO),
            TRANSFER_GEARBOX);

    private double targetPercent = 0.0;
    private double appliedVolts = 0.0;

    @Override
    public void updateInputs(TransferIOInputs inputs) {
        appliedVolts = MathUtil.clamp(targetPercent, -1.0, 1.0) * MAX_VOLTS;

        transferSim.setInputVoltage(appliedVolts);
        transferSim.update(LOOP_PERIOD_SEC);

        inputs.positionRad = transferSim.getAngularPositionRad();
        inputs.velocityRpm = transferSim.getAngularVelocityRPM();
        inputs.appliedVolts = appliedVolts;
        inputs.supplyCurrentAmps = Math.abs(transferSim.getCurrentDrawAmps());
        inputs.tempCelsius = AMBIENT_TEMP_C;
    }

    @Override
    public void setPercent(double percent) {
        targetPercent = MathUtil.clamp(percent, -1.0, 1.0);
    }

    @Override
    public void stop() {
        targetPercent = 0.0;
    }
}
