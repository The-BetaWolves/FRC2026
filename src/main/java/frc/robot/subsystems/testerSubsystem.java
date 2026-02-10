// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class testerSubsystem extends SubsystemBase {

  SparkMax testingMotor;
  double speed = .7;
  /** Creates a new testerSubsystem. */
  public testerSubsystem() {
    testingMotor = new SparkMax(1, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    testingMotor.set(speed);
    SmartDashboard.putNumber("testingSpeed", speed);
  }

  public void setSpeed(double speed) {
    this.speed = speed;
  }
}
