// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
    private double tolerenceRPM = 300.0;
    private double maxMotorOutput = 1.0;
    private double kP = 0.00018;
    private double kV = 0.0018;
    private double kS = 0.65;

    PIDController pid = new PIDController(kP, 0.0, 0.0);
    SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kS, kV);

    /** Creates a new testerSubsystem. */
    public Flywheel() {

        SmartDashboard.setDefaultNumber("flywheel target rpm", setpointRpm);
        SmartDashboard.setDefaultNumber("flywheel kp", kP);
        SmartDashboard.setDefaultNumber("flywheel kv", kV);
        SmartDashboard.setDefaultNumber("flywheel ks", kS);

        if (RobotBase.isSimulation()) {
            io = new FlywheelIOSim();
        } else {
            io = new FlywheelIOReal();
        }

        // pid.setTolerance(tolerenceRPM);

        monitoredMotor1 = qualityControlService.watch("flywheel motor " + Constants.Flywheel.motor1CanId);
        monitoredMotor2 = qualityControlService.watch("flywheel motor " + Constants.Flywheel.motor2CanId);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);

        double kPFromShuffleboard = SmartDashboard.getNumber("flywheel kp", kP);
        // pid.setP(kPFromShuffleboard);
        double kVFromShuffleboard = SmartDashboard.getNumber("flywheel kv", kV);
        // feedForward.setKv(kVFromShuffleboard);
        double setpointRpmFromShuffleboard = SmartDashboard.getNumber("flywheel target rpm", setpointRpm);

        double kSFromShuffleboard = SmartDashboard.getNumber("flywheel ks", kS);


        //double motorOutput = pid.calculate(inputs.velocityRpm, setpointRpm) + feedForward.calculate(setpointRpm);
        //motorOutput = MathUtil.clamp(motorOutput, 0.0, maxMotorOutput); //The Zero stops it from breaking

        // Command motor speed
        //io.setMotorOutput(0.2);

        // Command motor
        //io.setMotorSetpoint(setpointRpm);

        //Update From SmartDashBoard
        boolean updateFromSmartDashboardIsSet = false;
        if (kP != kPFromShuffleboard || kV != kVFromShuffleboard || kS != kSFromShuffleboard || setpointRpm != setpointRpmFromShuffleboard) {
            io.updateFromSmartDashboard(kPFromShuffleboard, kVFromShuffleboard, kSFromShuffleboard, setpointRpmFromShuffleboard);
            updateFromSmartDashboardIsSet = true;
        }
        kP = kPFromShuffleboard;
        kV = kVFromShuffleboard;
        kS = kSFromShuffleboard;
        setpointRpm = setpointRpmFromShuffleboard;
        

        // //Log Stuff
        Logger.recordOutput("Flywheel/UpdateFromSmartDashboard", updateFromSmartDashboardIsSet);
        Logger.recordOutput("Flywheel/SetpointRPM", setpointRpm);
        // Logger.recordOutput("Flywheel/MotorOutput", motorOutput);
        Logger.recordOutput("Flywheel/AtSetpoint", io.atSetpoint());

        Logger.recordOutput("Flywheel/kS", kS);

        // SmartDashboard.putNumber("flywheelSpeed", motorOutput);
        SmartDashboard.putNumber("flywheelTrueSpeed", io.getMotorVoltage());

        // quality control log to shuffleboard
        monitoredMotor1.update(inputs.motorController1IsPowered);
        monitoredMotor2.update(inputs.motorController2IsPowered);

    }

    public void setSetpoint(double setpointRpm) {
        this.setpointRpm = setpointRpm;
    }
    public boolean isAtSetpoint() {
        return io.atSetpoint();
    }
}