package frc.robot.subsystems.swerve.SwerveModule;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;

public interface SwerveModuleIO {
    void updateInputs(SwerveModuleIOInputs inputs);
    void setDesiredState(SwerveModuleState state);
    void setDriveMotorVoltage(Voltage voltage);
    Rotation2d getAbsoluteAngle();
    RelativeEncoder getRelativeEncoder();
    double getAppliedOutput();
}