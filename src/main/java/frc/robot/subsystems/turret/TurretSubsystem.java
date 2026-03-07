// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.services.QualityControlService;

public class TurretSubsystem extends SubsystemBase {

    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    private final QualityControlService qualityControlService = new QualityControlService("Turret", 0, 0);
    private final QualityControlService.MonitoredHardware monitoredMotor, monitoredEncoder;

    private double setpointRadians = 0.0; // not an input, it's an output of what we command, so doesn't go in io
    private double toleranceDegrees = 2.0;
    private double maxMotorOutput = 0.8;
    private double kP = 0.0; //1?
    PIDController pid = new PIDController( kP, 0.0, 0.0);

    public TurretSubsystem() {
        // add default radians and turret kp to shuffleboard
        SmartDashboard.setDefaultNumber("turret setpoint radians", setpointRadians);
        SmartDashboard.setDefaultNumber("turret kp", kP);
        
        if (RobotBase.isSimulation()) {
            io = new TurretIOSim();
        } else {
            io = new TurretIOReal();
        }

        pid.setTolerance(Math.toRadians(toleranceDegrees));

        // create the quality control motor
        monitoredMotor = qualityControlService.watch("turret motor " + Constants.Turret.motorCanId);
        monitoredEncoder = qualityControlService.watch("turret encoder");
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        //pid.setSetpoint((setpointRadians + 180) / 360);
         
        //setpointRadians = SmartDashboard.getNumber("turret setpoint radians", setpointRadians);
        //double kPFromShuffleboard = SmartDashboard.getNumber("turret kp", kP);
        //pid.setP(kPFromShuffleboard);

        double motorOutput = pid.calculate(inputs.positionRadians, setpointRadians);

        motorOutput = MathUtil.clamp(motorOutput, -maxMotorOutput, maxMotorOutput);

        // Command motor
        setMotorOutput(motorOutput);

        double errorRadians = setpointRadians - inputs.positionRadians;

        // Log outputs and commanded values
        Logger.recordOutput("Turret/SetpointRadians", setpointRadians);
        Logger.recordOutput("Turret/MotorOutput", motorOutput);
        Logger.recordOutput("Turret/AtSetpoint", pid.atSetpoint());
        Logger.recordOutput("Turret/ErrorRadians", errorRadians);

        // quality control log to shuffleboard
        monitoredMotor.update(inputs.motorControllerIsPowered);
        monitoredEncoder.update(inputs.encoderConnected);
    }

    public void setMotorOutput(double output) {
        // if we are going ccw and we are at or beyond the max, stop
        if(output > 0 && (inputs.positionRadians >= (Units.degreesToRadians(Constants.Turret.turretRotationLimitDegrees)))) {
            output = 0;
        
        // if we are going cw and are at or beyond max, stop
        } else if(output < 0 && (inputs.positionRadians <= (Units.degreesToRadians(-Constants.Turret.turretRotationLimitDegrees)))) {
            output = 0;
        }
        io.setMotorOutput(output);
    }

    public void setSetpoint(double setpointRadians) {
        this.setpointRadians = setpointRadians;
    }

    public void incrementSetpoint(double rate) {
        setpointRadians = setpointRadians + rate;
    }

    public boolean atSetpoint() {
        return pid.atSetpoint();
    }

    public void stop() {
        io.setMotorOutput(0.0);
    }
}
