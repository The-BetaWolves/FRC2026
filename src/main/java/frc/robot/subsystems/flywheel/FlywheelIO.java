// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

    @AutoLog
    public static class FlywheelIOInputs  {
        public double velocityRpm = 0.0;

        // quality control
        public boolean motorController1IsPowered = false;
        public boolean motorController2IsPowered = false;

    }

    public default void updateInputs(FlywheelIOInputs inputs) {}
    public void setMotorOutput(double output);
    public void setMotorSetpoint(double setpoint);
    public boolean atSetpoint();
    public void updateFromSmartDashboard(double kP, double kV, double kS, double setpointRPM);
    public double getMotorOutput();
}
