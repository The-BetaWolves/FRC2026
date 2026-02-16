// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SuperStateSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.turret.TurretSubsystem;

public class RobotContainer {
    private final SuperStateSubsystem superState = new SuperStateSubsystem();
    private final SwerveDrive swerveDrive = new SwerveDrive();

    //public testerSubsystem tester = new testerSubsystem();
    public IntakeSubsystem intake = new IntakeSubsystem();
    //public indexerSubsystem indexer = new indexerSubsystem();
    public Flywheel flywheel = new Flywheel();
    public TurretSubsystem turret = new TurretSubsystem();
    //public climberSubsystem climber = new climberSubsystem();
    public KickerSubsystem kicker = new KickerSubsystem();

    Joystick driverJoyStick = new Joystick(0);

    //private final SendableChooser<Command> autoChooser;


    public RobotContainer() {
        // processes service logic and stores in gettable variables
        superState.setDefaultCommand(
            Commands.run(()->{
                superState.updateValues(swerveDrive::getPose);
            }, superState)
        );

        turret.setDefaultCommand(
            Commands.run(()->{
                turret.setSetpoint(superState.getTurretSetpointRadians());
            }, turret)
        );

        flywheel.setDefaultCommand(
            Commands.run(()->{
                flywheel.setSetpoint(superState.getFlywheelSetpointRpm());
            }, flywheel)
        );

        configureBindings();
        //configureBindings();
        //autoChooser = AutoBuilder.buildAutoChooser();


        // Default driving command (joystick)
         
        swerveDrive.setDefaultCommand(
            new TeleopDriveCommand(
                swerveDrive,
                ()-> -(MathUtil.applyDeadband(driverJoyStick.getY(), Constants.Controls.Y_DEADBAND)),
                ()-> -(MathUtil.applyDeadband(driverJoyStick.getX(), Constants.Controls.Y_DEADBAND)),
                ()-> -(MathUtil.applyDeadband(driverJoyStick.getTwist(), Constants.Controls.ANGLE_JOYSTICK_DEADBAND))
            )
        );
        

        printDebugValues();
        //SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        /* 
        new JoystickButton(driverJoyStick, 3).onTrue(
        new InstantCommand(()-> tester.setSpeed(0.7), tester)
        ).onFalse(new InstantCommand(()-> tester.setSpeed(0.0), tester));
        */

        
        new JoystickButton(driverJoyStick, 3).onTrue(
        new InstantCommand(()-> intake.setSpeed(0.5), intake)
        ).onFalse(new InstantCommand(()-> intake.setSpeed(0.0), intake));
        new JoystickButton(driverJoyStick, 4).onTrue(
        new InstantCommand(()-> intake.setSpeed(-0.5), intake)
        ).onFalse(new InstantCommand(()-> intake.setSpeed(0.0), intake));

        new JoystickButton(driverJoyStick, 7).onTrue(
        new InstantCommand(()-> intake.setSpeed(0.5), intake)
        ).onFalse(new InstantCommand(()-> intake.setSpeed(0.0), intake));
        new JoystickButton(driverJoyStick, 8).onTrue(
        new InstantCommand(()-> intake.setSpeed(-0.5), intake)
        ).onFalse(new InstantCommand(()-> intake.setSpeed(0.0), intake));

        /* 
        new JoystickButton(driverJoyStick, 2).onTrue(
        new InstantCommand(()-> indexer.setSpeed(0.5), indexer)
        ).onFalse(new InstantCommand(()-> indexer.setSpeed(0.0), indexer));
        new JoystickButton(driverJoyStick, 1).onTrue(
        new InstantCommand(()-> shooter.setSpeed(0.7), shooter)
        ).onFalse(new InstantCommand(()-> shooter.setSpeed(0.0), shooter));
        */
        /* 
        new JoystickButton(driverJoyStick, 5).onTrue(
        new InstantCommand(()-> turret.setSpeed(-1.0), turret)
        ).onFalse(new InstantCommand(()-> turret.setSpeed(0.0), turret));
        new JoystickButton(driverJoyStick, 6).onTrue(
        new InstantCommand(()-> turret.setSpeed(1.0), turret)
        ).onFalse(new InstantCommand(()-> turret.setSpeed(0.0), turret));
        */
    }

    private void printDebugValues() {
            // add smart dashboard debug calls here instead of in subsystems
        }

    public Command getAutonomousCommand() {
        //return autoChooser.getSelected();
        //return new TestBangBang(swerveDrive);
        return Commands.print("No autonomous command configured");
    }
}
