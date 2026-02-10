package frc.robot.subsystems.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private double simDrivePositionMeters = 0.0;
    private double simDriveVelocityMps = 0.0;
    private Rotation2d simAngle = new Rotation2d();
    private Rotation2d simAbsoluteAngle = new Rotation2d();
    private SwerveModuleConstants constants;
    private static final double LOOP_PERIOD_SECS = 0.02; // 20 ms loop
    private static final double MAX_ROTATION_SPEED_RAD_PER_SEC = Math.PI; // 180 deg/sec

    public SwerveModuleIOSim(SwerveModuleConstants constants) {
        this.constants = constants;
    }

    public Rotation2d getAbsoluteAngle() {
        return new Rotation2d();
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        // Update drive position
        simDrivePositionMeters += simDriveVelocityMps * LOOP_PERIOD_SECS;

        inputs.drivePositionMeters = simDrivePositionMeters;
        inputs.driveVelocityMetersPerSecond = simDriveVelocityMps;
        inputs.turnAngleRotations = simAngle.getRotations();
        inputs.absoluteAngleDegrees = simAbsoluteAngle.getDegrees();
    }

    @Override
    public void setDesiredState(SwerveModuleState desired) {
        desired.optimize(simAngle);
    
        simDriveVelocityMps = desired.speedMetersPerSecond;
        Rotation2d targetAngle = desired.angle;
    
        double angleDiff = targetAngle.minus(simAngle).getRadians();
        double maxDelta = MAX_ROTATION_SPEED_RAD_PER_SEC * LOOP_PERIOD_SECS;
    
        if (Math.abs(angleDiff) < maxDelta) {
            simAngle = targetAngle;
        } else {
            double step = Math.copySign(maxDelta, angleDiff);
            simAngle = new Rotation2d(simAngle.getRadians() + step);
        }
    
        simAbsoluteAngle = new Rotation2d(
            simAngle.getRadians() + Math.toRadians(constants.angleOffset.getDegrees())
        );
    }
}
