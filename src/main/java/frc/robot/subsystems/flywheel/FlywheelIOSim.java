// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;


/** Add your docs here. */
public class FlywheelIOSim implements FlywheelIO {

    //THIS IS A STUB AND HAS NOT BEEN CONFIGURED FOR SIMULATIONS
    double positionRadians, motorOutput;

    public FlywheelIOSim() {

    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.motorController1IsPowered = true;
        inputs.motorController2IsPowered = true;

    }

    public double getAbsolutePos() {
        return 1.0;
    }

    @Override
    public void setMotorOutput(double motorOutput) {
        this.motorOutput = motorOutput;
    }

    public double getVelocityRPM() {
        return 0.0;
    }

    public double getMotorVoltage() {
        return 0.0;
    }

    public void setMotorSetpoint(double setpoint) {

    }

    public boolean atSetpoint() {
        return true;
    }

    public void updateFromSmartDashboard(double kP, double kV, double kS, double setpointRPM) {

    }
}
