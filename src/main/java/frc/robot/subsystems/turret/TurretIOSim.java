// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;


/** Add your docs here. */
public class TurretIOSim implements TurretIO {

    //THIS IS A STUB AND HAS NOT BEEN CONFIGURED FOR SIMULATIONS
    double positionRadians, motorOutput;

    public TurretIOSim() {

    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.positionRadians =  getAbsolutePos();
        inputs.encoderConnected = true;
        inputs.rawPositionRotations = 0.0;
        inputs.motorControllerIsPowered = true;
    }

    public double getAbsolutePos() {
        return 1.0;
    }

    @Override
    public void setMotorOutput(double motorOutput) {
        this.motorOutput = motorOutput;
    }
}
