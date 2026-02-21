// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class FlywheelIOReal implements FlywheelIO {
    SparkFlex motor1, motor2;
    RelativeEncoder motor1Encoder, motor2Encoder;
    int motor1CanId, motor2CanId;

    public FlywheelIOReal() {
        motor1 = new SparkFlex(Constants.Flywheel.motor1CanId, MotorType.kBrushless);
        motor2 = new SparkFlex(Constants.Flywheel.motor2CanId, MotorType.kBrushless);
        motor1Encoder = motor1.getEncoder();
        motor2Encoder = motor2.getEncoder();

        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig motor1Config = new SparkMaxConfig();
        SparkMaxConfig motor2FollowerConfig = new SparkMaxConfig();

        globalConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kCoast);

        motor1Config
            .apply(globalConfig)
            .inverted(false);

        motor2FollowerConfig
            .apply(globalConfig)
            .follow(motor1, true);

        motor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor2.configure(motor2FollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.velocityRpm = getVelocityRPM();
        inputs.motorController1IsPowered = motor1.getBusVoltage() > 6.0;
        inputs.motorController2IsPowered = motor2.getBusVoltage() > 6.0;
    }

    public void setMotorOutput(double speed) {
        motor1.set(speed);
    }

    public double getVelocityRPM() {
        return (motor1Encoder.getVelocity() + motor2Encoder.getVelocity()) / 2;
    }
}
 