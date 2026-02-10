// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

/** Add your docs here. */
public class FlywheelIOReal implements FlywheelIO {
    SparkFlex motor1, motor2;
    int motor1CanId, motor2CanId;

    public FlywheelIOReal() {
        motor1 = new SparkFlex(Constants.Flywheel.motor1CanId, MotorType.kBrushless);
        motor2 = new SparkFlex(Constants.Flywheel.motor2CanId, MotorType.kBrushless);

        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig motorConfig = new SparkMaxConfig();

        motorConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);

        motorConfig
            .apply(globalConfig)
            .inverted(false);

        motor1.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor2.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.motorController1IsPowered = motor1.getBusVoltage() > 6.0;
        inputs.motorController1IsPowered = motor2.getBusVoltage() > 6.0;
        inputs.velocityRpm = getVelocityRPM();
    }

    public void setMotorOutput(double speed) {
        motor1.set(speed);
        motor2.set(speed);
    }

    public double getVelocityRPM() {
        return (motor1.getEncoder().getVelocity() + motor2.getEncoder().getVelocity()) / 2;
    }
}
 