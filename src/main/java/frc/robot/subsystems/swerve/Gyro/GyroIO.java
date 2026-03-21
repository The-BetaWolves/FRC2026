package frc.robot.subsystems.swerve.Gyro;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    Rotation2d getYaw();
    void zeroYaw();
    void setYaw(double angle);

    @AutoLog
    public static class GyroIOInputs  {
        public Rotation2d yaw = new Rotation2d();
        public double yawDegrees = 0.0;
        public boolean gyroIsPowered = false;
    }

    public default void updateInputs(GyroIOInputs inputs) {}

    // simRotationRate is for simulation only
    public default void updateInputs(GyroIOInputs inputs,  double simRotationRate) {}
    
}