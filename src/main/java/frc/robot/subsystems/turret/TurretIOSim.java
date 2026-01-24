package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;

public class TurretIOSim implements TurretIO {
    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);

    private final DCMotorSim turretSim =
            new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(
                            GEARBOX, TurretConstants.MOMENT_OF_INERTIA_KG_M2, TurretConstants.GEAR_RATIO),
                    GEARBOX);

    private double appliedVolts = 0.0;

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        double clampedVolts = MathUtil.clamp(appliedVolts, -TurretConstants.MAX_VOLTAGE, TurretConstants.MAX_VOLTAGE);
        turretSim.setInputVoltage(clampedVolts);
        turretSim.update(Robot.getUpdateRateSec());

        inputs.connected = true;
        inputs.positionRad = turretSim.getAngularPositionRad();
        inputs.velocityRadPerSec = turretSim.getAngularVelocityRadPerSec();
        inputs.appliedVolts = clampedVolts;
        inputs.currentAmps = Math.abs(turretSim.getCurrentDrawAmps());
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = volts;
    }
}
