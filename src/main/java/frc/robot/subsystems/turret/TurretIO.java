// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

    @AutoLog
    public static class TurretIOInputs  {
        public double rawPositionRotations = 0.0; // raw encoder value
        public double positionRadians = 0.0; // adjusted encoder value
        public double motorEncoderRotations = 0.0; //raw encoder value of the motor's encoder
        public boolean encoderConnected = false;

        // quality control
        public boolean motorControllerIsPowered = false;
    }

    public default void updateInputs(TurretIOInputs inputs) {}
    public void setMotorOutput(double output);
}
