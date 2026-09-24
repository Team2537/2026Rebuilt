package frc.robot.subsystems.transfer;

public interface TransferIO {
    class TransferIOInputs{
        public double supplyCurrentAmps=0.0;
        public double appliedVolts=0.0;
        public double velocityRpm=0.0;
    }

    default void setPercent(double percent){}
    default void updateInputs(TransferIOInputs inputs){}
    default void stop(){}
}