// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase {
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    private double setpointRpm = 0.0;
    //private double tolerenceRPM = 300.0;
    private double maxMotorOutput = 1.0;
    private double kP = 0.00002;
    private double kV = 0.001815;
    private double kS = 0.0;

    PIDController pid = new PIDController(kP, 0.0, 0.0);
    SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kS, kV);

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutAngle m_angle = Radians.mutable(0);
     // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);

    private final SysIdRoutine sysIdRoutine;

    /** Creates a new testerSubsystem. */
    public Flywheel() {

        // SmartDashboard.setDefaultNumber("flywheel target rpm", setpointRpm);
        // SmartDashboard.setDefaultNumber("flywheel kp", kP);
        // SmartDashboard.setDefaultNumber("flywheel kv", kV);
        // SmartDashboard.setDefaultNumber("flywheel ks", kS);

        if (RobotBase.isSimulation()) {
            io = new FlywheelIOSim();
        } else {
            io = new FlywheelIOReal();
        }

        // pid.setTolerance(tolerenceRPM);

        sysIdRoutine =
            new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                    // Tell SysId how to plumb the driving voltage to the motor(s).
                    (io::setMotorVoltage),
                    // Tell SysId how to record a frame of data for each motor on the mechanism being
                    // characterized.
                    log -> {
                        // Record a frame for the shooter motor.
                        log.motor("shooter-wheel")
                            .voltage(
                                m_appliedVoltage.mut_replace(
                                    io.getMotorOutput() * RobotController.getBatteryVoltage(), Volts))
                            .angularPosition(m_angle.mut_replace(io.getEncoder().getPosition(), Rotations))
                            .angularVelocity(
                                m_velocity.mut_replace(io.getEncoder().getVelocity(), RotationsPerSecond));
                    },
                    // Tell SysId to make generated commands require this subsystem, suffix test state in
                    // WPILog with this subsystem's name ("shooter")
                    this));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);

        //double kPFromShuffleboard = SmartDashboard.getNumber("flywheel kp", kP);
        // pid.setP(kPFromShuffleboard);
        //double kVFromShuffleboard = SmartDashboard.getNumber("flywheel kv", kV);
        // feedForward.setKv(kVFromShuffleboard);
        //double setpointRpmFromShuffleboard = SmartDashboard.getNumber("flywheel target rpm", setpointRpm);

        //double kSFromShuffleboard = SmartDashboard.getNumber("flywheel ks", kS);


        //double motorOutput = pid.calculate(inputs.velocityRpm, setpointRpm) + feedForward.calculate(setpointRpm);
        //motorOutput = MathUtil.clamp(motorOutput, 0.0, maxMotorOutput); //The Zero stops it from breaking

        // Command motor speed
        //io.setMotorOutput(0.2);

        double maxRPM = 5900.0;
        if (setpointRpm > maxRPM) {
            setpointRpm = maxRPM;
        }
        // Command motor
        io.setMotorSetpoint(setpointRpm);

        //Update From SmartDashBoard
        /* 
        if (kP != kPFromShuffleboard || kV != kVFromShuffleboard || kS != kSFromShuffleboard || setpointRpm != setpointRpmFromShuffleboard) {
            io.updateFromSmartDashboard(kPFromShuffleboard, kVFromShuffleboard, kSFromShuffleboard, setpointRpmFromShuffleboard);
        }
        kP = kPFromShuffleboard;
        kV = kVFromShuffleboard;
        kS = kSFromShuffleboard;
        setpointRpm = setpointRpmFromShuffleboard;
        */
        

        // //Log Stuff
        Logger.recordOutput("Flywheel/SetpointRPM", setpointRpm);
        // Logger.recordOutput("Flywheel/MotorOutput", motorOutput);
        Logger.recordOutput("Flywheel/AtSetpoint", io.atSetpoint());

        Logger.recordOutput("Flywheel/kS", kS);

        SmartDashboard.putNumber("flywheelTrueSpeed", io.getMotorOutput());
    }

    public void setSetpoint(double setpointRpm) {
        this.setpointRpm = setpointRpm;
    }
    public boolean isAtSetpoint() {
        return io.atSetpoint();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}