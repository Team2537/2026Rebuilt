package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MechanismVisualizer;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** Turret subsystem that targets the hub angle with FF + PID control. */
public class Turret extends SubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private final ProfiledPIDController controller;
    private final SimpleMotorFeedforward feedforward;
    private final DoubleSupplier hubYawRadSupplier;

    private double goalRad = 0.0;

    public Turret(TurretIO io, DoubleSupplier hubYawRadSupplier) {
        super("turret");
        this.io = io;
        this.hubYawRadSupplier = hubYawRadSupplier;
        this.controller =
                new ProfiledPIDController(
                        TurretConstants.KP,
                        TurretConstants.KI,
                        TurretConstants.KD,
                        new TrapezoidProfile.Constraints(
                                TurretConstants.MAX_VELOCITY_RAD_PER_SEC, TurretConstants.MAX_ACCEL_RAD_PER_SEC_SQ),
                        Robot.getUpdateRateSec());
        controller.setTolerance(
                TurretConstants.POSITION_TOLERANCE_RAD, TurretConstants.VELOCITY_TOLERANCE_RAD_PER_SEC);
        this.feedforward = new SimpleMotorFeedforward(TurretConstants.KS, TurretConstants.KV);
    }

    public void setGoalRad(double goalRad) {
        this.goalRad = goalRad;
    }

    public void setGoal(Rotation2d goal) {
        setGoalRad(goal.getRadians());
    }

    public double getGoalRad() {
        return goalRad;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(inputs.positionRad);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        double hubYawRad = hubYawRadSupplier.getAsDouble();
        if (Double.isFinite(hubYawRad)) {
            goalRad = hubYawRad;
        }

        if (DriverStation.isDisabled()) {
            io.setVoltage(0.0);
            controller.reset(inputs.positionRad);
        } else {
            double pidVolts = controller.calculate(inputs.positionRad, goalRad);
            double ffVolts = feedforward.calculate(controller.getSetpoint().velocity);
            double outputVolts =
                    MathUtil.clamp(pidVolts + ffVolts, -TurretConstants.MAX_VOLTAGE, TurretConstants.MAX_VOLTAGE);
            io.setVoltage(outputVolts);

            Logger.recordOutput("Turret/GoalRad", goalRad);
            Logger.recordOutput("Turret/SetpointRad", controller.getSetpoint().position);
            Logger.recordOutput("Turret/SetpointVelRadPerSec", controller.getSetpoint().velocity);
            Logger.recordOutput("Turret/PidVolts", pidVolts);
            Logger.recordOutput("Turret/FFVolts", ffVolts);
            Logger.recordOutput("Turret/OutputVolts", outputVolts);
            Logger.recordOutput("Turret/AtGoal", controller.atGoal());
        }

        Logger.recordOutput("Turret/AngleRad", inputs.positionRad);
        Logger.recordOutput("Turret/AngleDeg", Math.toDegrees(inputs.positionRad));
        MechanismVisualizer.setTurretYaw(Rotation2d.fromRadians(inputs.positionRad));
    }
}
