// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BetaUtil;

/** This is the class that handles all methods and math relating to the shooting mechanism */
public class BetaShootingUtil {

    //Could also use a lookup table
    public static double getRPM_atDistance(double distance, double fudgeSetFactor) {
        //This is the equation for the line of the shot speed over distance, gotten via taking data points and plotting them in desmos, and then using regression.
        return (755.78*(distance) + 1277.33)*fudgeSetFactor;
    }

    public static double getTOF_atDistance(double distance, double fudgeFactor) {
        //This is the equation for the line of time of flight over distance, gotten via taking data points and plotting them in desmos, and then using regression.
        return ((1.37626)/1+Math.pow(Math.E, (-(1.71737*(distance)-1.79862))))*fudgeFactor;
    }
}
