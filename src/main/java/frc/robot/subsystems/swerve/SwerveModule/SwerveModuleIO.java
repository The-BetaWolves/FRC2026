package frc.robot.subsystems.swerve.SwerveModule;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;

public interface SwerveModuleIO {

    @AutoLog
    public static class SwerveModuleIOInputs {
        public double drivePositionMeters = 0.0;
        public double driveVelocityMetersPerSecond = 0.0;
        public double turnAngleRotations = 0.0;
        public double absoluteAngleDegrees = 0.0;
        public double absoluteAdjustedAngleDegrees = 0.0;
        public double relativeAngleDegrees = 0.0;
        public double desiredMetersPerSecond = 0.0;
        public boolean driveMotorIsPowered = false;
        public boolean angleMotorIsPowered = false;
        public boolean absoluteEncoderIsConnected = false;
    }

    void updateInputs(SwerveModuleIOInputs inputs);
    void setDesiredState(SwerveModuleState state);
    void setDriveMotorVoltage(Voltage voltage);
    Rotation2d getAbsoluteAngle();
    RelativeEncoder getRelativeEncoder();
    double getAppliedOutput();
    void setAzimuth(Rotation2d angle);
}