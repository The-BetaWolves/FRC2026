// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.services;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Add your docs here. */
public class ShooterService {
    InterpolatingDoubleTreeMap lookupTable = new InterpolatingDoubleTreeMap();


    public double getShotSpeed(double distance) {
        setLookupTable();

        return lookupTable.get(distance);
    }

    private void setLookupTable() {
        //Key = distance in Meters, value = speed in RPM
        //Distance is center of hub to center of shooter
        lookupTable.put(2.6, 3050.0);
        lookupTable.put(3.6, 3700.0);
    }
}
