package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    class TurretIOInputs {
        public boolean connected = false;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    default void updateInputs(TurretIOInputs inputs) {}

    default void setVoltage(double volts) {}
}
