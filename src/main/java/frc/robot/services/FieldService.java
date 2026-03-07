// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.services;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

public class FieldService {

    public Translation2d getTargetPose(Pose2d robotPose) {
        
        if (robotPose.getY() < 2) {
            return Constants.Field.blueLeftPose;
        } else if (robotPose.getY() > 4) {
            return Constants.Field.blueRightPose;
        } else {
            return Constants.Field.blueHubPose;
        }
    }

    public double getDistanceToTarget(Pose2d robotPose, Translation2d targetPose) {
        //The distance formula, which calculate the absolute distance between two vector points 
        //The square root of( (x2 - x1)^2 + (y2 - y1)^2 )
        return Math.sqrt(Math.pow(targetPose.getX() - robotPose.getX(), 2) + Math.pow(targetPose.getY() - robotPose.getY(), 2));
    }

    public Translation2d getAdjustedTargetPose(Pose2d robotPose, Translation2d targetPose, ChassisSpeeds robotFieldRelativeVelocity) {
        double distanceToTargetMeters = getDistanceToTarget(robotPose, targetPose);
        double timeOfFlightSeconds = distanceToTargetMeters / Constants.Flywheel.ballSpeedMetersPerSecond;

        Translation2d driftInMeters = new Translation2d(
            robotFieldRelativeVelocity.vxMetersPerSecond * timeOfFlightSeconds,
            robotFieldRelativeVelocity.vyMetersPerSecond * timeOfFlightSeconds
        );

        return targetPose.minus(driftInMeters);
    }


}
