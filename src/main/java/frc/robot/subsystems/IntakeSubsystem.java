// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  SparkMax intakeRollerMotor;
  SparkMax intakeRotatorMotor;
  DutyCycleEncoder encoder;
  double rollerSpeed, rotatorSpeed, kp, ki, setpoint, motorOutput;

  PIDController pid;
  /** Creates a new testerSubsystem. */
  public IntakeSubsystem() {
    intakeRollerMotor = new SparkMax(15, MotorType.kBrushless);
    intakeRotatorMotor = new SparkMax(4, MotorType.kBrushless);
    encoder = new DutyCycleEncoder(1);
    rollerSpeed = 0.0;
    setpoint = 4.0;
    kp = 0.05; //0.13;
    ki = 0.00;
    pid = new PIDController(kp, ki, 0.0);

    motorOutput = 0.0;

    encoder.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeRollerMotor.set(rollerSpeed);

    if (setpoint < Constants.Intake.minRotatorDegree) {
      setpoint = Constants.Intake.minRotatorDegree;
    } else if (setpoint > Constants.Intake.maxRotatorDegree) {
      setpoint = Constants.Intake.maxRotatorDegree;
    }

    motorOutput = pid.calculate(((encoder.get() - 0.268) * 360), setpoint);
    intakeRotatorMotor.set(motorOutput);

    SmartDashboard.putNumber("intakeRollerSpeed", rollerSpeed);
    SmartDashboard.putNumber("intakeRotatorSpeed", rotatorSpeed);
    SmartDashboard.putNumber("intakeRotatorAbsoluteEncoderValue", encoder.get() - 0.476);
    SmartDashboard.putNumber("intakeRotatorAbsoluteEncoderDegree", (encoder.get() - 0.476) * 360);
    SmartDashboard.putNumber("intakeRotatorSetpoint", setpoint);
  }

  public void setRollerSpeed(double speed) {
    rollerSpeed = speed;
  }
  public void setRotatorSpeed(double speed) {
    rotatorSpeed = speed;
  }

  public void setSetpoint(double setpoint) {
    this.setpoint = setpoint;
  }
  public void increaseSetpoint() {
    setpoint = setpoint + 1.5;
  }
  public void decreaseSetpoint() {
    setpoint = setpoint - 1.5;
  }
}
