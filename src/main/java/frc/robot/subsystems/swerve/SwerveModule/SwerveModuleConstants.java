package frc.robot.subsystems.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveModuleConstants {
    public final int driveMotorId;
    public final int angleMotorId;
    public final int absoluteEncoderId;
    public final boolean driveInverted;
    public final boolean angleInverted;
    public final Rotation2d angleOffset;
    public final Translation2d modulePosition;
    public final String canBusName;

    public SwerveModuleConstants(int driveMotorId, int angleMotorId, int absoluteEncoderId, String canBusName, boolean driveInverted, boolean angleInverted, Rotation2d angleOffset, Translation2d modulePosition) {
        this.driveMotorId = driveMotorId;
        this.angleMotorId = angleMotorId;
        this.absoluteEncoderId = absoluteEncoderId;
        this.driveInverted = driveInverted;
        this.angleInverted = angleInverted;
        this.angleOffset = angleOffset;
        this.modulePosition = modulePosition;
        this.canBusName = canBusName;
    }
}

