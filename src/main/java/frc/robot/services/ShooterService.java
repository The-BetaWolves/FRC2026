// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.services;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterService {
    //InterpolatingDoubleTreeMap lookupTable = new InterpolatingDoubleTreeMap();

    /* This is the old system
    public double getShotSpeed(double distance, double fudgeSetFactor) {
        setLookupTable();
        double fudgeFactor = fudgeSetFactor; //If all shots are too short or too long, multiply them by a factor

        return lookupTable.get(distance) * fudgeFactor * 0.8365;
    }

    private void setLookupTable() {
        //Key = distance in Meters, value = speed in RPM
        //Distance is center of hub to center of shooter
        lookupTable.put(1.25, 3100.0); //2500.0
        lookupTable.put(2.25, 3600.0);
        lookupTable.put(3.0, 3950.0);
        lookupTable.put(4.0, 4500.0);
        lookupTable.put(5.0, 5200.0);
        lookupTable.put(5.5, 5450.0);
        lookupTable.put(6.0, 5900.0);
    }
         */

    public double getShotSpeed(double distance, double fudgeSetFactor) {
        //This is the equation for the line of the shot speed over distance, gotten via taking data points and plotting them in desmos, and then using regression.
        return (755.78*(distance) + 1277.33)*fudgeSetFactor;
    }

    public double getTimeOfFlight(double distance, double fudgeFactor) {
        //This is the equation for the line of time of flight over distance, gotten via taking data points and plotting them in desmos, and then using regression.
        return ((1.37626)/1+Math.pow(Math.E, (-(1.71737*(distance)-1.79862))))*fudgeFactor;
    }
    //Distance Equation y=755.78129x+1277.33324
    //TOF Equation y=\frac{1.37626}{1+e^{-\left(1.71737x-1.79862\right)}}      y = (1.37626)/1+e - (1.71737x-1.79862)
}
