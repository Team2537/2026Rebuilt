package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {
        public double shooterLeftPositionRad = 0.0;
        public double shooterLeftVelocityRpm = 0.0;
        public double shooterLeftAppliedVolts = 0.0;
        public double shooterLeftSupplyCurrentAmps = 0.0;
        public double shooterLeftTempCelcius = 0.0;

        public double shooterRightPositionRad = 0.0;
        public double shooterRightVelocityRpm = 0.0;
        public double shooterRightAppliedVolts = 0.0;
        public double shooterRightSupplyCurrentAmps = 0.0;
        public double shooterRightTempCelcius = 0.0;

        public double hoodPositionRad = 0.0;
        public double hoodVelocityRpm = 0.0;
        public double hoodAppliedVolts = 0.0;
        public double hoodSupplyCurrentAmps = 0.0;
        public double hoodTempCelcius = 0.0;

        public double kickerVelocityRpm = 0.0;
        public double kickerPositionRad = 0.0;
        public double kickerAppliedVolts = 0.0;
        public double kickerSupplyCurrentAmps = 0.0;
        public double kickerTempCelcius = 0.0;
    }

    /** Updates the current sensor inputs for logging and control. */
    default void updateInputs(ShooterIOInputs inputs) {}

    /** Sets left shooter velocity in RPM. */
    default void setLeftVelocity(double rpm) {}

    /** Sets right shooter velocity in RPM. */
    default void setRightVelocity(double rpm) {}

    /** Sets hood angle in radians. */
    default void setHoodAngle(double angle) {}

    /** Sets kicker torque-current request in amps. */
    default void setKickerTorque(double torqueCurrentAmps) {}

    /** Sets kicker voltage request in volts. */
    default void setKickerVoltage(double volts) {}

    /** Stops all shooter motors. */
    default void stop() {}
}
