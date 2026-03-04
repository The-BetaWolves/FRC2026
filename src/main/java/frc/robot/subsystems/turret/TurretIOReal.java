// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class TurretIOReal implements TurretIO {

    DutyCycleEncoder encoder;
    double positionRadians, lastMotorOutput;

    private final double zeroOffsetRotations = 0.804;   // if forward reads 0.6, enter 0.6 here
    private final boolean invertEncoder = false;

    SparkMax motor;

    public TurretIOReal() {
        encoder = new DutyCycleEncoder(Constants.Turret.encoderChannel);
        motor = new SparkMax(Constants.Turret.motorCanId, MotorType.kBrushless);
        
        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig motorConfig = new SparkMaxConfig();

        motorConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);

        motorConfig
            .apply(globalConfig)
            .inverted(true);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.rawPositionRotations = encoder.get();
        inputs.positionRadians =  getPositionRadians();
        inputs.encoderConnected = encoder.isConnected();
        inputs.motorControllerIsPowered = motor.getBusVoltage() > 6.0;
        inputs.motorEncoderRotations = motor.getEncoder().getPosition();
    }

    private double getPositionRadians() {
        double rotations = encoder.get(); // rotations 0-1

        // subtract zeroOffset to make encoder 0 = forward
        rotations = rotations - zeroOffsetRotations;

        // if rotations is negative (rotation -(-1)), make positive
        rotations = rotations - Math.floor(rotations);

        // Convert to signed rotations [-0.5, 0.5]
        if (rotations > 0.5) rotations = rotations - 1.0;

        if (invertEncoder) {
            rotations = -rotations;
        }

        // radians in [-pi, pi]
        double radians = rotations * 2.0 * Math.PI;

        return radians;
    }

    @Override
    public void setMotorOutput(double motorOutput) {
        motor.set(motorOutput);
    }
}
