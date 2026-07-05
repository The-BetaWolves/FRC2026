package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Single home for the alliance-color decision, including the policy for
 * when the Driver Station hasn't sent an alliance yet (default: blue).
 * Never call DriverStation.getAlliance().get() directly — it throws
 * before the DS connects.
 */
public final class AllianceUtil {
    private AllianceUtil() {}

    public static boolean isRed() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }
}
