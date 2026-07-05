// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;


public class VisionConstants {

    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    //static Path jsonPath = Filesystem.getDeployDirectory().toPath().resolve("Betawolves2026LabField.json");
    // public static AprilTagFieldLayout aprilTagLayout = setFieldLayout();
    // private static AprilTagFieldLayout setFieldLayout() {
    //     AprilTagFieldLayout layout;
    //     try {
    //         layout = new AprilTagFieldLayout(jsonPath);
    //     } catch (IOException ex) {
    //         layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    
    //         DriverStation.reportError("Unable to open filee: " + jsonPath, ex.getStackTrace());
    //     }
    //     return layout;
    // }

    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "camera_0";
    public static String camera1Name = "camera_1";
    public static String camera2Name = "limelight";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToCamera0 =
        new Transform3d(-0.34, 0.24, 0.35, new Rotation3d(0.0, Math.toRadians(-17.3), Math.toRadians(135)));
    public static Transform3d robotToCamera1 =
        new Transform3d(-0.34, -0.24, 0.35, new Rotation3d(0.0, Math.toRadians(-17.3), Math.toRadians(225)));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.2;
    public static double maxZError = 0.75;

    // Vision Trust During Rotation
    // observations above maxYarRate are rejected outright. 
    //For MT1: blur + latency.
    // For MT2: its x/y is computed FROM the heading we feed it, fast spin = bad
    // Below it, linear std devs are inflated smoothly by yawRateStdDevFactor per rad/s.
    // Both are first-guess values (tune!).
    public static double maxYawRateRadPerSec = Math.toRadians(540);
    public static double yawRateStdDevFactor = 0.5;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.1; // Meters

    // MT2 and single-tag headings get infinite std dev in Vision
    // multi-tag MT1/Photon headings are trusted through distance, tag count and this baseline.
    // lower it to trust vision heading more
    // raise it to not truse vision as much (slowly correct gyro)
    public static double angularStdDevBaseline = 0.12; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
        new double[] {
            1.0, // Camera 0
            1.0, // Camera 1
            1.0
        };

    public static double linearStdDevMegatag2Factor = 0.5;
}
