// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.services;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class FieldService {

    public Translation2d getTargetPose(Pose2d robotPose) {
        
        return Constants.Field.realBlueHubPose;
        /*
        if (robotPose.getY() < 2) {
            return Constants.Field.blueLeftPose;
        } else if (robotPose.getY() > 4) {
            return Constants.Field.blueRightPose;
        } else {
            return Constants.Field.blueHubPose;
        }
        */
    }

    public double getDistanceToTarget(Pose2d robotPose, Translation2d targetPose) {
        //The distance formula, which calculate the absolute distance between two vector points 
        //The square root of( (x2 - x1)^2 + (y2 - y1)^2 )
        //return Math.sqrt(Math.pow(targetPose.getX() - robotPose.getX(), 2) + Math.pow(targetPose.getY() - robotPose.getY(), 2));
        return robotPose.getTranslation().getDistance(targetPose);
    }

    public double getDistanceFromTurretToTarget(Pose2d robotPose, Translation2d targetPose) {

        double distanceOfTurretToCenterMeters = 0.19;
        double cos = Math.cos(robotPose.getRotation().getRadians());
        double sin = Math.sin(robotPose.getRotation().getRadians());
        double translationX = -(distanceOfTurretToCenterMeters * cos);
        double translationY = -distanceOfTurretToCenterMeters * sin;

        /*
        SmartDashboard.putNumber("turret distance cos", cos);
        SmartDashboard.putNumber("turret distance sin", sin);
        SmartDashboard.putNumber("turret distance x", translationX);
        SmartDashboard.putNumber("turret distance y", translationY);
         */

        //Gets the point along the circle from x = r*cos(theta) and y = r*sin(theta)
        Translation2d turretHoleRobotRelativePose = new Translation2d(translationX, translationY);

        return robotPose.getTranslation().plus(turretHoleRobotRelativePose).getDistance(targetPose);
    }

    public Translation2d getAdjustedTargetPose(Pose2d robotPose, Translation2d targetPose, ChassisSpeeds robotFieldRelativeVelocity) {
        double distanceToTargetMeters = getDistanceToTarget(robotPose, targetPose);
        double ballSpeedFromSmartdashboard = SmartDashboard.getNumber("ballSpeedConstant", Constants.Flywheel.ballSpeedMetersPerSecond);
        double timeOfFlightSeconds = distanceToTargetMeters / ballSpeedFromSmartdashboard;

        Translation2d driftInMeters = new Translation2d(
            -robotFieldRelativeVelocity.vyMetersPerSecond * timeOfFlightSeconds,
            robotFieldRelativeVelocity.vxMetersPerSecond * timeOfFlightSeconds
        );

        return targetPose.plus(driftInMeters);
    }


}
