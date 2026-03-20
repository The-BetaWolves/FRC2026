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
    double kP = 0.015;
    double maxSetpoint = 425;
    double minSetpoint = 1;
    double levelOneHight = 365; //6.5in per 365 rotations
    
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


        if (setpoint >= maxSetpoint) {
            setpoint = maxSetpoint;
        } else if (setpoint <= minSetpoint) {
            setpoint = minSetpoint;
        }

        speed = pid.calculate(climberMotor.getEncoder().getPosition(), setpoint);


        if ((climberMotor.getEncoder().getPosition() < minSetpoint && speed < 0) || (climberMotor.getEncoder().getPosition() > maxSetpoint) && speed > 0) {
            climberMotor.set(0.0);
        } else {
            climberMotor.set(speed);
        }

        SmartDashboard.putNumber("climberSpeed", speed);
        SmartDashboard.putNumber("climberEncoder", climberMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("climberSetpoint", setpoint);
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }
    public void incrementSetpoint(double rate) {
        setpoint = setpoint + rate;
    }
}
