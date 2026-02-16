// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  SparkMax climberMotor;
  double speed;
  /** Creates a new testerSubsystem. */
  public ClimberSubsystem() {
    climberMotor = new SparkMax(17, MotorType.kBrushless);
    speed = 0.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    climberMotor.set(speed);
    SmartDashboard.putNumber("climberSpeed", speed);
  }

  public void setSpeed(double speed) {
    this.speed = speed;
  }
}
