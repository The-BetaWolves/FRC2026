// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.services;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
}
