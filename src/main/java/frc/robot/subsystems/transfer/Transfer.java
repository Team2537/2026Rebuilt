package frc.robot.subsystems.transfer;

public class Transfer {
    class TransferIOInputs{
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double velocityRpm = 0.0;
        
    }
    default void setPercent(double percent){}
    default void updateInputs(TransferIOInputs inputs){}
    default void stop(){}
}