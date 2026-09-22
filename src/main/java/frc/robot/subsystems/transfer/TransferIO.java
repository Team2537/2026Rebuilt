package frc.robot.subsystems.transfer;

public interface TransferIO {
    class TransferIOInputs{
        public double positionRad=0.0;
        public double velocityRpm=0.0;
        public double appliedVolts=0.0;
        public double supplyCurrentAmps=0.0;//note about this later
    }

    default void updateInputs(TransferIOInputs inputs){}
    default void setPercent(double percent) {} //straightforward, no big deal precise speed, but you can also set volts directly
    default void stop(){}
}