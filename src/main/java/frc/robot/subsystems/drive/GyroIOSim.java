package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;

/** Simulation gyro model driven by swerve module deltas. */
public class GyroIOSim implements GyroIO {
  private final SwerveDriveKinematics kinematics;
  private Supplier<SwerveModulePosition[]> modulePositionsSupplier =
      () -> new SwerveModulePosition[0];

  private Rotation2d yaw = Rotation2d.kZero;
  private SwerveModulePosition[] lastPositions = new SwerveModulePosition[0];
  private double lastTimestampSec = Timer.getFPGATimestamp();

  public GyroIOSim(edu.wpi.first.math.geometry.Translation2d[] moduleTranslations) {
    this.kinematics = new SwerveDriveKinematics(moduleTranslations);
  }

  public void setModulePositionsSupplier(Supplier<SwerveModulePosition[]> supplier) {
    this.modulePositionsSupplier = supplier;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    SwerveModulePosition[] positions = modulePositionsSupplier.get();
    double nowSec = Timer.getFPGATimestamp();

    if (positions == null || positions.length == 0) {
      inputs.connected = true;
      inputs.yawPosition = yaw;
      inputs.yawVelocityRadPerSec = 0.0;
      inputs.odometryYawTimestamps = new double[] { nowSec };
      inputs.odometryYawPositions = new Rotation2d[] { yaw };
      return;
    }

    if (lastPositions.length == 0) {
      lastPositions = copyPositions(positions);
      lastTimestampSec = nowSec;
    }

    SwerveModulePosition[] deltas = new SwerveModulePosition[positions.length];
    for (int i = 0; i < positions.length; i++) {
      deltas[i] =
          new SwerveModulePosition(
              positions[i].distanceMeters - lastPositions[i].distanceMeters, positions[i].angle);
    }

    var twist = kinematics.toTwist2d(deltas);
    yaw = yaw.plus(new Rotation2d(twist.dtheta));

    double dt = Math.max(1e-6, nowSec - lastTimestampSec);
    inputs.connected = true;
    inputs.yawPosition = yaw;
    inputs.yawVelocityRadPerSec = twist.dtheta / dt;
    inputs.odometryYawTimestamps = new double[] { nowSec };
    inputs.odometryYawPositions = new Rotation2d[] { yaw };

    lastPositions = copyPositions(positions);
    lastTimestampSec = nowSec;
  }

  @Override
  public void setYaw(Rotation2d yawAngle) {
    yaw = yawAngle;
  }

  private static SwerveModulePosition[] copyPositions(SwerveModulePosition[] positions) {
    SwerveModulePosition[] copy = new SwerveModulePosition[positions.length];
    for (int i = 0; i < positions.length; i++) {
      copy[i] = new SwerveModulePosition(positions[i].distanceMeters, positions[i].angle);
    }
    return copy;
  }
}
