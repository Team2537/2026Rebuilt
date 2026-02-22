package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

/** Robot hardware/mode description helpers. */
public final class RobotType {
  private RobotType() {}

  public enum Type {
    ROBOT_2026_REBUILT
  }

  public enum Mode {
    REAL(Type.ROBOT_2026_REBUILT),
    SIMULATION(Type.ROBOT_2026_REBUILT),
    REPLAY(Type.ROBOT_2026_REBUILT);

    private final Type defaultRobotType;

    Mode(Type defaultRobotType) {
      this.defaultRobotType = defaultRobotType;
    }

    public Type getDefaultRobotType() {
      return defaultRobotType;
    }
  }

  private static final boolean REPLAY_ENABLED = false;

  public static final Mode MODE =
      RobotBase.isReal() ? Mode.REAL : (REPLAY_ENABLED ? Mode.REPLAY : Mode.SIMULATION);

  public static final Type TYPE = MODE.getDefaultRobotType();

  public static boolean isTuning() {
    return !DriverStation.isFMSAttached();
  }
}
