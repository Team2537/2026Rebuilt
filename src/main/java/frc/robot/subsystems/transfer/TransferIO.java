package frc.robot.subsystems.transfer;

import org.littletonrobotics.junction.AutoLog;

public interface TransferIO {
    @AutoLog
    class TransferIOInputs {
        public double positionRad = 0.0;
        public double velocityRpm = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

    default void updateInputs(TransferIOInputs inputs) {}

    default void setPercent(double percent) {}

    default void stop() {}
}
