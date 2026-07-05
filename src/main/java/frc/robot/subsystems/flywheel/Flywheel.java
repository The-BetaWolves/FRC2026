// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.flywheel.SysId.FlywheelSysId;

public class Flywheel extends SubsystemBase {
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    private double setpointRpm = 0.0;
    private double kP = 0.00002;
    private double kV = 0.001815;
    private double kS = 0.0;

    PIDController pid = new PIDController(kP, 0.0, 0.0);
    SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kS, kV);

    private final FlywheelSysId sysId;
    private final Alert motor1Alert = new Alert("Flywheel motor 1 not powered!", AlertType.kError);
    private final Alert motor2Alert = new Alert("Flywheel motor 2 not powered!", AlertType.kError);

    public Flywheel() {
        if (RobotBase.isSimulation()) {
            io = new FlywheelIOSim();
        } else {
            io = new FlywheelIOReal();
        }
        sysId = new FlywheelSysId(this, io);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);
        motor1Alert.set(!inputs.motorController1IsPowered);
        motor2Alert.set(!inputs.motorController2IsPowered);

        double maxRPM = 5900.0;
        if (setpointRpm > maxRPM) {
            setpointRpm = maxRPM;
        }

        io.setMotorSetpoint(setpointRpm);
        
        Logger.recordOutput("Flywheel/SetpointRPM", setpointRpm);
        Logger.recordOutput("Flywheel/AtSetpoint", io.atSetpoint());
        Logger.recordOutput("Flywheel/MotorOutput", io.getMotorOutput());
    }

    public void setSetpoint(double setpointRpm) {
        this.setpointRpm = setpointRpm;
    }
    public boolean isAtSetpoint() {
        return io.atSetpoint();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }
}