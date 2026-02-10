package frc.robot.subsystems.swerve.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class OdometryReal implements OdometryIO {

    private final SwerveDriveOdometry odometry;
    
    public OdometryReal(SwerveDriveKinematics kinematics, Rotation2d heading, SwerveModulePosition[] swerveModulePositions) {
        odometry = new SwerveDriveOdometry(
            kinematics,
            heading,
            swerveModulePositions
        );
    }

    @Override
    public void updateInputs(OdometryIOInputs inputs, Rotation2d heading, SwerveModulePosition[] swerveModulePositions) {
        inputs.robotPose =  odometry.update(heading, swerveModulePositions);
    }

    public Pose2d updatePose() {
        return new Pose2d();
    }

    public void resetPose(Rotation2d heading, SwerveModulePosition[] swerveModulePositions, Pose2d pose) {
        odometry.resetPosition(heading, swerveModulePositions, pose);
    }

    public Pose2d getPoseMeters() {
        return odometry.getPoseMeters();
    }


}
