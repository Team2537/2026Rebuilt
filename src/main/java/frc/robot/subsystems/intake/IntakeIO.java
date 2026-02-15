package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        double leftAppliedVolts = 0.0;
        double leftPositionRad = 0.0;
        double leftCurrentAmps = 0.0;
        double leftVelocityRpm = 0.0;

        double rightAppliedVolts = 0.0;
        double rightPositionRad = 0.0;
        double rightCurrentAmps = 0.0;
        double rightVelocityRpm = 0.0;

        double rollerAppliedVolts = 0.0;
        double rollerPositionRad = 0.0;
        double rollerCurrentAmps = 0.0;
        double rollerVelocityRpm = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {
    }

    public default void setRollerRpm(double rpm) {
    }
    
    public default void retract(){
    }

    public default void extend(){
    }

    public default void stop(){
    }
}
