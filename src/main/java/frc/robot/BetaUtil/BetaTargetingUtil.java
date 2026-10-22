// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BetaUtil;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.util.AllianceUtil;


public class BetaTargetingUtil {

    public static Translation2d getVariableTargetPose(Pose2d robotPose) {

        //For Red Change points for passing
        if (AllianceUtil.isRed()) {
            if (robotPose.getY() < 4  && robotPose.getX() < (Constants.Field.redHubPose.getX() - 1.2)) {
                return Constants.Field.redLeftPose;
            } else if (robotPose.getY() > 4 && robotPose.getX() < (Constants.Field.redHubPose.getX() - 1.2)) {
                return Constants.Field.redRightPose;
            } else {
                return Constants.Field.redHubPose;
            }
        } else {
            if (robotPose.getY() < 4  && robotPose.getX() > (Constants.Field.blueHubPose.getX() + 1.2)) {
                return Constants.Field.blueLeftPose;
            } else if (robotPose.getY() > 4 && robotPose.getX() > (Constants.Field.blueHubPose.getX() + 1.2)) {
                return Constants.Field.blueRightPose;
            } else {
                return Constants.Field.blueHubPose;
            }
        }
    }

    public static double getDistanceFromTurretToTarget(Pose2d robotPose, Translation2d targetPose) {

        double distanceOfTurretToCenterMeters = 0.19;
        double cos = Math.cos(robotPose.getRotation().getRadians());
        double sin = Math.sin(robotPose.getRotation().getRadians());
        double translationX = -(distanceOfTurretToCenterMeters * cos);
        double translationY = -distanceOfTurretToCenterMeters * sin;

        //Gets the point along the circle from x = r*cos(theta) and y = r*sin(theta)
        Translation2d turretHoleRobotRelativePose = new Translation2d(translationX, translationY);

        return robotPose.getTranslation().plus(turretHoleRobotRelativePose).getDistance(targetPose);
    }

    public static Translation2d getMotionAdjustedTargetPose(Pose2d robotPose, Translation2d targetPose, ChassisSpeeds robotFieldRelativeVelocity) {
        double distanceToTargetMeters =  robotPose.getTranslation().getDistance(targetPose);
        double timeOfFlightSeconds = BetaShootingUtil.getTOF_atDistance(distanceToTargetMeters, 0.8); 

        Translation2d driftInMeters = new Translation2d(
            -robotFieldRelativeVelocity.vyMetersPerSecond * timeOfFlightSeconds,
            robotFieldRelativeVelocity.vxMetersPerSecond * timeOfFlightSeconds
        );

        return targetPose.plus(driftInMeters);
    }

    public static Translation2d getMovingShotTargetPose(Pose2d robotPose, Translation2d targetPose, ChassisSpeeds robotFieldRelativeVelocity, double minRobotSpeedMeters, double maxRobotSpeedMeters) {
        Translation2d newTargetPose;
        if (
            (robotFieldRelativeVelocity.vxMetersPerSecond + 
            robotFieldRelativeVelocity.vyMetersPerSecond) >= minRobotSpeedMeters) {

            Translation2d lastTargetPose = targetPose;
            for(int i = 0; i < 5; i++) {
                Translation2d addedTargetPose = getMotionAdjustedTargetPose(robotPose, lastTargetPose, robotFieldRelativeVelocity).plus(targetPose);
                Translation2d averagedTargetPose = addedTargetPose.div(2);
                lastTargetPose = averagedTargetPose;
            }

            if (
            (robotFieldRelativeVelocity.vxMetersPerSecond + 
            robotFieldRelativeVelocity.vyMetersPerSecond) < maxRobotSpeedMeters) {

                newTargetPose = lastTargetPose;
                Logger.recordOutput("Can_Make_Current_Shot", true);

            } else {
                newTargetPose = targetPose;
                Logger.recordOutput("Can_Make_Current_Shot", false);
            }
        } else {
            newTargetPose = targetPose;
            Logger.recordOutput("Can_Make_Current_Shot", true);
        }

        return newTargetPose;
    }
}
