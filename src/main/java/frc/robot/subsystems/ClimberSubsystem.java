// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    SparkFlex climberMotor;
    double speed;
    /** Creates a new testerSubsystem. */
    public ClimberSubsystem() {
        climberMotor = new SparkFlex(35, MotorType.kBrushless);
        speed = 0.0;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if ((climberMotor.getEncoder().getPosition() < 0.0 && speed < 0) || (climberMotor.getEncoder().getPosition() > 425) && speed > 0) {
            climberMotor.set(0.0);
        } else {
            climberMotor.set(speed);
        }
        
        SmartDashboard.putNumber("climberSpeed", speed);
        SmartDashboard.putNumber("climberEncoder", climberMotor.getEncoder().getPosition());
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }
}
