package frc.robot;

import java.util.EnumSet;

/** Robot-wide constants expressed in standard units. */
public final class Constants {
    public enum Mechanism {
        DRIVE,
        VISION,
        SHOOTER,
        TRANSFER
    }

    private Constants() {}

    /**
     * Enable/disable robot mechanisms at init time.
     *
     * <p>Drive and vision are enabled to support baseline movement and localization.
     * Add/remove entries here to control startup of non-essential mechanisms.
     */
    public static final EnumSet<Mechanism> ENABLED_MECHANISMS =
            EnumSet.of(Mechanism.DRIVE, Mechanism.VISION);

    public static boolean isMechanismEnabled(Mechanism mechanism) {
        return ENABLED_MECHANISMS.contains(mechanism);
    }
}
