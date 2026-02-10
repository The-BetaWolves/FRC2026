package frc.robot.subsystems.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {
    void updateInputs(SwerveModuleIOInputs inputs);
    void setDesiredState(SwerveModuleState state);
    Rotation2d getAbsoluteAngle();
}