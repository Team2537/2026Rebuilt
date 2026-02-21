package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double leftAppliedVolts = 0.0;
        public double leftPositionRad = 0.0;
        public double leftSupplyCurrentAmps = 0.0;
        public double leftStatorCurrentAmps = 0.0;
        public double leftVelocityRpm = 0.0;

        public double rightAppliedVolts = 0.0;
        public double rightPositionRad = 0.0;
        public double rightSupplyCurrentAmps = 0.0;
        public double rightStatorCurrentAmps = 0.0;
        public double rightVelocityRpm = 0.0;

        public double rollerAppliedVolts = 0.0;
        public double rollerPositionRad = 0.0;
        public double rollerSupplyCurrentAmps = 0.0;
        public double rollerStatorCurrentAmps = 0.0;
        public double rollerVelocityRpm = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {
    }

    public default void setRollerRpm(double rpm) {
    }
    
    public default void retract(){
    }

    public default void extend(){
    }

    public default void home(){
    }

    public default void slowRetract(){
    }

    public default void resetEncoders(){
    }

    public default void stop(){
    }
}
