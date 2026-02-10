package frc.robot.services;

import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TurretService {

    public double getSetpointRadians(Pose2d robotPose, Translation2d targetPose) {

        // get the differenece between the target and the robot
        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();

        // find the angle from the bot to the target
        double fieldRelativeRadians = Math.atan2(dy, dx);

        // factor in the gyro
        double robotRelativeRadians = MathUtil.angleModulus(fieldRelativeRadians - robotPose.getRotation().getRadians());
        
        // spin 180 to the back of the bot
        double turretRelativeRadians = MathUtil.angleModulus(robotRelativeRadians - Math.PI);

        // don't let the turret spin too far, clamp the setpoint to our limits
        double setpointRadians = MathUtil.clamp(turretRelativeRadians, -Constants.Turret.turretRotationLimit, Constants.Turret.turretRotationLimit);
        
        return setpointRadians;
    }
}
