package frc.robot.subsystems.swerve.Odometry;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public interface OdometryIO  {

    Pose2d getPoseMeters();

    public default void updateInputs(OdometryIOInputs inputs, Rotation2d heading, SwerveModulePosition[] swerveModulePositions) {}

    /**
     * Sets the robots position to be at the 0x, 0y, 0 degrees
     * @return Pose2d: The estimated position of the robot. 
     */
    public default void resetPose(Rotation2d heading, SwerveModulePosition[] swerveModulePositions, Pose2d pose) {}

}
