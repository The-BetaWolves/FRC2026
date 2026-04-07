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


    public double getShotSpeed(double distance, double fudgeSetFactor) {
        setLookupTable();
        double fudgeFactor = fudgeSetFactor; //If all shots are too short or too long, multiply them by a factor

        return lookupTable.get(distance) * fudgeFactor;
    }

    private void setLookupTable() {
        //Key = distance in Meters, value = speed in RPM
        //Distance is center of hub to center of shooter
        lookupTable.put(1.25, 3100.0);
        lookupTable.put(2.25, 3600.0);
        lookupTable.put(3.0, 4050.0);
        lookupTable.put(4.0, 4600.0);
        lookupTable.put(5.0, 5200.0);
        lookupTable.put(5.5, 5450.0);
        lookupTable.put(6.0, 5900.0);
    }
}
