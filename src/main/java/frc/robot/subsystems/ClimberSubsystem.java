// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    SparkFlex climberMotor;
    double speed, setpoint;
    double kP = 0.0;
    PIDController pid = new PIDController(kP, 0.0, 0.0);
    /** Creates a new testerSubsystem. */
    public ClimberSubsystem() {
        climberMotor = new SparkFlex(35, MotorType.kBrushless);
        speed = 0.0;
        setpoint = 0.0;

        SmartDashboard.setDefaultNumber("climber kp", kP);
    }

    @Override
    public void periodic() {
        kP = SmartDashboard.getNumber("climber kp", kP);
        pid.setP(kP);

        speed = pid.calculate(setpoint);

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
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }
}
