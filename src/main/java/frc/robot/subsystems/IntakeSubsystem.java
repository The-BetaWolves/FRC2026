// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
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
  double rollerSpeed, rotatorSpeed, kp, ki, setpoint, motorOutput, intakeEncoderOffset, intakePositionDegrees, maxMotorOutput;

  PIDController pid;
  /** Creates a new testerSubsystem. */
  public IntakeSubsystem() {
    intakeRollerMotor = new SparkMax(15, MotorType.kBrushless);
    intakeRotatorMotor = new SparkMax(4, MotorType.kBrushless);
    encoder = new DutyCycleEncoder(1);
    rollerSpeed = 0.0;
    setpoint = 3.0;
    kp = 0.025; //0.13;
    ki = 0.00;
    pid = new PIDController(kp, ki, 0.0);
    intakeEncoderOffset = 0.33;
    maxMotorOutput = 1.0;

    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(true);

    intakeRollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    encoder.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeRollerMotor.set(rollerSpeed);

    intakePositionDegrees = (encoder.get() - intakeEncoderOffset) * 360;

    if (setpoint < Constants.Intake.minRotatorDegree) {
      setpoint = Constants.Intake.minRotatorDegree;
    } else if (setpoint > Constants.Intake.maxRotatorDegree) {
      setpoint = Constants.Intake.maxRotatorDegree;
    }

    rotatorSpeed = pid.calculate(intakePositionDegrees, setpoint);

    // limit speed of motor to max speed
    if(Math.abs(rotatorSpeed) > maxMotorOutput) rotatorSpeed = maxMotorOutput * Math.signum(rotatorSpeed);

    intakeRotatorMotor.set(rotatorSpeed);

    SmartDashboard.putNumber("intakeRollerSpeed", rollerSpeed);
    SmartDashboard.putNumber("intakeRotatorSpeed", rotatorSpeed);
    SmartDashboard.putNumber("intakeRotatorAbsoluteEncoderValue", encoder.get());
    SmartDashboard.putNumber("intakeRotatorAbsoluteEncoderDegree", intakePositionDegrees);
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
  public void setState(double setpoint, double rollerSpeed) {
    this.setpoint = setpoint;
    this.rollerSpeed = rollerSpeed;
  }
}
