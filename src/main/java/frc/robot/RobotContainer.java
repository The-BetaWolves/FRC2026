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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
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

    public IntakeSubsystem intake = new IntakeSubsystem();
    public IndexerSubsystem indexer = new IndexerSubsystem();
    public Flywheel flywheel = new Flywheel();
    public TurretSubsystem turret = new TurretSubsystem();
    public ClimberSubsystem climber = new ClimberSubsystem();
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

        kicker.setDefaultCommand(
            Commands.run(()->{
                kicker.setSpeed(superState.getKickerSpeed());
            }, kicker)
        );

        indexer.setDefaultCommand(
            Commands.run(()->{
                indexer.setSpeed(superState.getIndexerSpeed());
            }, indexer)
        );

        swerveDrive.setDefaultCommand(
            new TeleopDriveCommand(
                swerveDrive,
                ()-> -(MathUtil.applyDeadband(driverJoyStick.getY(), Constants.Controls.Y_DEADBAND)),
                ()-> -(MathUtil.applyDeadband(driverJoyStick.getX(), Constants.Controls.Y_DEADBAND)),
                ()-> -(MathUtil.applyDeadband(driverJoyStick.getTwist(), Constants.Controls.ANGLE_JOYSTICK_DEADBAND))
            )
        );

        configureBindings();
        //autoChooser = AutoBuilder.buildAutoChooser();


        printDebugValues();
        //SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        /* 
        new JoystickButton(driverJoyStick, 3).onTrue(
        new InstantCommand(()-> tester.setSpeed(0.7), tester)
        ).onFalse(new InstantCommand(()-> tester.setSpeed(0.0), tester));
        */

        
        // Intake In
        new JoystickButton(driverJoyStick, 3).onTrue(
            new InstantCommand(()-> intake.setRollerSpeed(0.8), intake)
        ).onFalse(new InstantCommand(()-> intake.setRollerSpeed(0.0), intake));
        new JoystickButton(driverJoyStick, 4).onTrue(
            new InstantCommand(()-> intake.setRollerSpeed(-0.8), intake)
        ).onFalse(new InstantCommand(()-> intake.setRollerSpeed(0.0), intake));

        // Intake Rotator
        new JoystickButton(driverJoyStick, 5).whileTrue(
            new RunCommand(()-> intake.increaseSetpoint(), intake));
        new JoystickButton(driverJoyStick, 10).whileTrue(
            new RunCommand(()-> intake.decreaseSetpoint(), intake));


        // SHOOT! - Kicker and Flywheel
        new JoystickButton(driverJoyStick, 1).onTrue(
            new InstantCommand(()->superState.setFireIntent(SuperStateSubsystem.FireIntent.FIRE))
        ).
        onFalse(
            new ParallelCommandGroup(
                new InstantCommand(()->superState.setFireIntent(SuperStateSubsystem.FireIntent.IDLE)))
        );
        
        
        new JoystickButton(driverJoyStick, 7).onTrue(
            new InstantCommand(()-> climber.setSpeed(0.3), climber)
        ).onFalse(new InstantCommand(()-> climber.setSpeed(0.0), climber));
        new JoystickButton(driverJoyStick, 8).onTrue(
            new InstantCommand(()-> climber.setSpeed(-0.3), climber)
        ).onFalse(new InstantCommand(()-> climber.setSpeed(0.0), climber));
        

        /* 
        new JoystickButton(driverJoyStick, 2).onTrue(
        new InstantCommand(()-> indexer.setSpeed(0.5), indexer)
        ).onFalse(new InstantCommand(()-> indexer.setSpeed(0.0), indexer));
        new JoystickButton(driverJoyStick, 1).onTrue(
        new InstantCommand(()-> shooter.setSpeed(0.7), shooter)
        ).onFalse(new InstantCommand(()-> shooter.setSpeed(0.0), shooter));
        */
        
        // Turret CW
        /* 
        new JoystickButton(driverJoyStick, 6).whileTrue(
            new RunCommand(()-> turret.incrementSetpoint(0.05), turret)
        );
        
        // Turret CCW
        new JoystickButton(driverJoyStick, 9).whileTrue(
            new RunCommand(()-> turret.incrementSetpoint(-0.05), turret)
        );
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
