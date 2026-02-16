// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.services.QualityControlService;

public class Flywheel extends SubsystemBase {
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    private final QualityControlService qualityControlService = new QualityControlService("Flywheel", 0, 8);
    private final QualityControlService.MonitoredHardware monitoredMotor1, monitoredMotor2;

    private double setpointRpm = 0.0;
    private double tolerenceRPM = 2.0;
    private double maxMotorOutput = 0.6;
    private double kP = 0.0;

    PIDController pid = new PIDController(kP, 0.0, 0.0);

    /** Creates a new testerSubsystem. */
    public Flywheel() {

        // add default rpm and flywheel kp to shuffleboard
        SmartDashboard.setDefaultNumber("flywheel target rpm", setpointRpm);
        SmartDashboard.setDefaultNumber("flywheel kp", kP);

        if (RobotBase.isSimulation()) {
        io = new FlywheelIOSim();
        } else {
        io = new FlywheelIOReal();
        }

        pid.setTolerance(tolerenceRPM);

        monitoredMotor1 = qualityControlService.watch("flywheel motor " + Constants.Flywheel.motor1CanId);
        monitoredMotor2 = qualityControlService.watch("flywheel motor " + Constants.Flywheel.motor2CanId);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);

        
        setpointRpm = SmartDashboard.getNumber("flywheel target rpm", setpointRpm);
        double kPFromShuffleboard = SmartDashboard.getNumber("flywheel kp", kP);
        pid.setP(kPFromShuffleboard);
        
        double motorOutput = pid.calculate(inputs.velocityRpm, setpointRpm );
        motorOutput = MathUtil.clamp(motorOutput, -maxMotorOutput, maxMotorOutput);

        // Command motor
        io.setMotorOutput(motorOutput);

        double errorRpm = setpointRpm - inputs.velocityRpm;

        //Log Stuff
        Logger.recordOutput("Flywheel/SetpointRPM", setpointRpm);
        Logger.recordOutput("Flywheel/MotorOutput", motorOutput);
        Logger.recordOutput("Flywheel/AtSetpoint", pid.atSetpoint());

        // quality control log to shuffleboard
        monitoredMotor1.update(inputs.motorController1IsPowered);
        monitoredMotor2.update(inputs.motorController2IsPowered);

    }

    public void setSetpoint(double setpointRpm) {
        this.setpointRpm = setpointRpm;
    }
}