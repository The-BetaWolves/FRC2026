// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class FlywheelIOReal implements FlywheelIO {
    SparkFlex motor1, motor2;
    RelativeEncoder motor1Encoder, motor2Encoder;
    int motor1CanId, motor2CanId;

    SparkClosedLoopController controller;
    SparkFlexConfig globalConfig;
    SparkFlexConfig motor1Config;
    SparkFlexConfig motor2FollowerConfig;

    private double setpointRPM = 0.0;
    private double tolerenceRPM = 100.0;
    private double maxMotorOutput = 1.0;
    private double kP = 0.00018;
    private double kV = 0.001825;

    public FlywheelIOReal() {
        motor1 = new SparkFlex(Constants.Flywheel.motor1CanId, MotorType.kBrushless);
        motor2 = new SparkFlex(Constants.Flywheel.motor2CanId, MotorType.kBrushless);
        motor1Encoder = motor1.getEncoder();
        motor2Encoder = motor2.getEncoder();

        globalConfig = new SparkFlexConfig();
        motor1Config = new SparkFlexConfig();
        motor2FollowerConfig = new SparkFlexConfig();

        globalConfig
            .smartCurrentLimit(80)
            .idleMode(IdleMode.kCoast);

        motor1Config
            .apply(globalConfig)
            .inverted(false)
            .voltageCompensation(12)
            .closedLoop
                .pid(kP,0.0, 0.0)
                    .maxOutput(maxMotorOutput)
                    .minOutput(0.0)
                    //.allowedClosedLoopError(tolerenceRPM, ClosedLoopSlot.kSlot0)
                .feedForward.kV(kV);
            
        motor2FollowerConfig
            .apply(globalConfig)
            .follow(motor1, true);

        motor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor2.configure(motor2FollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        controller = motor1.getClosedLoopController();
        controller.setSetpoint(setpointRPM, ControlType.kVelocity);
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

    
    public void setMotorSetpoint(double setpoint) {
        controller.setSetpoint(setpoint, ControlType.kVelocity);
    }

    public boolean atSetpoint() {
        return (controller.getSetpoint() - getVelocityRPM()) <= tolerenceRPM;
    }
    

    public double getVelocityRPM() {
        return (motor1Encoder.getVelocity() + motor2Encoder.getVelocity()) / 2;
    }

    public double getMotorVoltage() {
        return (motor1.getAppliedOutput() + motor1.getAppliedOutput()) /2;
    }

    public void updateFromSmartDashboard(double kP, double kV, double setpointRPM) {
        motor1Config.closedLoop
            .pid(kP, 0.0, 0.0)
            .feedForward.kV(kV);

        motor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
        controller.setSetpoint(setpointRPM, ControlType.kVelocity);
    }
}
 