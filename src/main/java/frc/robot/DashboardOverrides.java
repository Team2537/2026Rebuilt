package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

/** Reads and publishes driver dashboard override entries (auto-aim, distance, vision disable). */
public final class DashboardOverrides {
    private static final String AUTO_AIM_KEY = "Overrides/OverrideAutoAim";
    private static final String AIM_DISTANCE_METERS_KEY = "Overrides/AimDistanceMeters";
    private static final String CLEAR_ALL_KEY = "Overrides/ClearAll";
    private static final String DISABLE_VISION_KEY = "Overrides/DisableVision";
    private static final String STATUS_AUTO_AIM_ACTIVE_KEY = "Overrides/Status/OverrideAutoAimActive";
    private static final String STATUS_AIM_DISTANCE_METERS_KEY = "Overrides/Status/AimDistanceMeters";
    private static final double DEFAULT_AIM_DISTANCE_METERS = 1.5;

    /** Writes default values to SmartDashboard for all override entries. */
    public void init() {
        SmartDashboard.setDefaultBoolean(AUTO_AIM_KEY, false);
        SmartDashboard.setDefaultNumber(AIM_DISTANCE_METERS_KEY, DEFAULT_AIM_DISTANCE_METERS);
        SmartDashboard.setDefaultBoolean(DISABLE_VISION_KEY, false);
        SmartDashboard.putBoolean(STATUS_AUTO_AIM_ACTIVE_KEY, false);
        SmartDashboard.putNumber(STATUS_AIM_DISTANCE_METERS_KEY, DEFAULT_AIM_DISTANCE_METERS);
    }

    /** Reads current override values and publishes status outputs. */
    public void periodic(Vision vision) {
        boolean overrideAutoAimEnabled = isAutoAimEnabled();
        double overrideAimDistanceMeters = getAimDistanceMeters();
        boolean visionDisabled = SmartDashboard.getBoolean(DISABLE_VISION_KEY, false);
        SmartDashboard.putBoolean(STATUS_AUTO_AIM_ACTIVE_KEY, overrideAutoAimEnabled);
        SmartDashboard.putNumber(STATUS_AIM_DISTANCE_METERS_KEY, overrideAimDistanceMeters);
        Logger.recordOutput("dashboard/overrides/autoAimEnabled", overrideAutoAimEnabled);
        Logger.recordOutput("dashboard/overrides/aimDistanceMeters", overrideAimDistanceMeters);
        Logger.recordOutput("dashboard/overrides/visionDisabled", visionDisabled);
        if (vision != null) {
            vision.setDashboardDisabled(visionDisabled);
        }
    }

    /** Returns whether the dashboard auto-aim override is enabled. */
    public boolean isAutoAimEnabled() {
        return SmartDashboard.getBoolean(AUTO_AIM_KEY, false);
    }

    /** Returns the dashboard override aim distance in meters. */
    public double getAimDistanceMeters() {
        double distanceMeters = SmartDashboard.getNumber(AIM_DISTANCE_METERS_KEY, DEFAULT_AIM_DISTANCE_METERS);
        if (Double.isFinite(distanceMeters) && distanceMeters > 0.0) {
            return distanceMeters;
        }
        return DEFAULT_AIM_DISTANCE_METERS;
    }

    /** Returns a command that resets all dashboard overrides to their defaults. */
    public Command clearAllCommand() {
        return Commands.runOnce(() -> {
            SmartDashboard.putBoolean(AUTO_AIM_KEY, false);
            SmartDashboard.putNumber(AIM_DISTANCE_METERS_KEY, DEFAULT_AIM_DISTANCE_METERS);
            SmartDashboard.putBoolean(DISABLE_VISION_KEY, false);
        }).withName("DashboardClearAllOverrides");
    }

    /** Publishes the clear-all override command to the dashboard. */
    public void publishCommands(CommandTelemetry telemetry) {
        SmartDashboard.putData(
                CLEAR_ALL_KEY,
                telemetry.withCommandSource("dashboard.overrides.clearAll", clearAllCommand()));
    }
}
