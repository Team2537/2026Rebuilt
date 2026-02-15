package frc.robot.subsystems.shooter;
import org.littletonrobotics.junction.AutoLog;

// check interface vs class
public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
       // left motor
       public double shooterLeftPositionRad = 0.0;
       public double shooterLeftVelocityRpm = 0.0;
       public double shooterLeftAppliedVolts = 0.0;
       public double shooterLeftSupplyCurrentAmps = 0.0;
       public double shooterLeftTempCelcius = 0.0;

       // right motor
       public double shooterRightPositionRad = 0.0;
       public double shooterRightVelocityRpm = 0.0;
       public double shooterRightAppliedVolts = 0.0;
       public double shooterRightSupplyCurrentAmps = 0.0;
       public double shooterRightTempCelcius = 0.0;

       // hood motor
       public double hoodPositionRad = 0.0;
       public double hoodVelocityRpm = 0.0;
       public double hoodAppliedVolts = 0.0;
       public double hoodSupplyCurrentAmps = 0.0;
       public double hoodTempCelcius = 0.0;

       // kicker motor
       public double kickerVelocityRpm = 0.0;
       public double kickerPositionRad = 0.0;
       public double kickerAppliedVolts = 0.0;
       public double kickerSupplyCurrentAmps = 0.0;
       public double kickerTempCelcius = 0.0;

    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ShooterIOInputs inputs) {
    }

    // kicker: torque and voltage
    // hood: position (angle)
    // shooter motors: individual velocity control

    // run left motor at specified velocity
    public default void setLeftVelocity(double rpm) {

    }

    // run right motor at specified velocity
    public default void setRightVelocity(double rpm) {

    }

    // set hood angle to specified angle (Radians)
    public default void setHoodAngle(double angle) {

    }

    // set torque-current command (amps) for kicker motor
    public default void setKickerTorque(double torqueCurrentAmps) {

    }
    
    // set voltage of kicker motor
    public default void setKickerVoltage(double volts) {
        
    }
    
    /** Stop all motors. */
    public default void stop() {
    }
}
