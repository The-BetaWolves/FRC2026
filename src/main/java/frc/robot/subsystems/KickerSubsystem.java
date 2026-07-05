// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KickerSubsystem extends SubsystemBase {

    SparkFlex motor;
    double speed = 0.0;

    private final Alert motorAlert = new Alert("Kicker motor not powered!", AlertType.kError);

    public KickerSubsystem() {
        motor = new SparkFlex(26, MotorType.kBrushless);
        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig motorConfig = new SparkMaxConfig();

        motorConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kCoast);

        motorConfig
            .apply(globalConfig)
            .inverted(true);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        motor.set(speed);
        Logger.recordOutput("Kicker/Speed", speed);
        motorAlert.set(motor.getBusVoltage() < 6.0);
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }
}
