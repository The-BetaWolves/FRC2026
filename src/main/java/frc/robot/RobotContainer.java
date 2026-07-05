// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.camera2Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.SuperStateSubsystem;
import frc.robot.subsystems.SuperStateSubsystem.FireIntent;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

public class RobotContainer {
    private final SuperStateSubsystem superState = new SuperStateSubsystem();
    private final SwerveDrive swerveDrive = new SwerveDrive();

    public IntakeSubsystem intake = new IntakeSubsystem();
    public IndexerSubsystem indexer = new IndexerSubsystem();
    public final Flywheel flywheel = new Flywheel();
    public TurretSubsystem turret = new TurretSubsystem();
    public KickerSubsystem kicker = new KickerSubsystem();
    public Vision vision;

    Joystick driverJoyStick = new Joystick(0);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        //Vision stuffs
        if(!RobotBase.isSimulation()) {
            vision =
            new Vision(
                swerveDrive::addVisionMeasurement,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1),
                new VisionIOLimelight(camera2Name, swerveDrive::getHeading)
            );
        } else {
            vision =
            new Vision(
                swerveDrive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, swerveDrive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, swerveDrive::getPose),
                new VisionIOLimelight(camera2Name, swerveDrive::getHeading));
        }
        

        // processes service logic and stores in gettable variables
        superState.setDefaultCommand(
            Commands.run(()->{
                superState.updateValues(swerveDrive::getPose, swerveDrive::getFieldRelativeChassisSpeeds, flywheel::isAtSetpoint, turret::atSetpoint);
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

        intake.setDefaultCommand(
            Commands.run(()->{
                intake.setState(superState.getIntakeRotatorSetpoint(), superState.getIntakeSpeed());
            }, intake)
        );

        // Deadband, cubing, and alliance-based inversion all live in TeleopDriveCommand
        swerveDrive.setDefaultCommand(
            new TeleopDriveCommand(
                swerveDrive,
                driverJoyStick::getY,
                driverJoyStick::getX,
                driverJoyStick::getZ,
                superState
            )
        );

        //NamedCommands.registerCommand("setToStop", new InstantCommand(()->superState.setFireIntent(FireIntent.STOP)));
        NamedCommands.registerCommand("setToIdle", new InstantCommand(()->superState.setFireIntent(FireIntent.IDLE)));
        NamedCommands.registerCommand("setToFire", new InstantCommand(()->superState.setFireIntent(FireIntent.FIRE)));
        NamedCommands.registerCommand("setToClear", new InstantCommand(()->superState.setFireIntent(FireIntent.CLEAR)));
        NamedCommands.registerCommand("setToIntake", new InstantCommand(()->superState.setFireIntent(FireIntent.INTAKE)));
        NamedCommands.registerCommand("setToFireAndIntake", new InstantCommand(()->superState.setFireIntent(FireIntent.FIREANDINTAKE)));

        NamedCommands.registerCommand("setIntakeUp", new InstantCommand(()-> intake.setSetpoint(Constants.Intake.maxRotatorDegree)));

        // NamedCommands.registerCommand("staticFire", new ParallelCommandGroup(
        //     new WaitCommand(Constants.Flywheel.firingTimeSeconds),
        //     new InstantCommand(()->superState.setFireIntent(FireIntent.FIRE))).andThen(
        //         new InstantCommand(()->superState.setFireIntent(FireIntent.CLEAR))
        //     ));

        NamedCommands.registerCommand("staticFire", new ParallelCommandGroup(
            new WaitCommand(Constants.Flywheel.firingTimeSeconds).andThen(
                    new InstantCommand(()->superState.setFireIntent(FireIntent.CLEAR))
                ),
            new InstantCommand(()->superState.setFireIntent(FireIntent.FIREANDINTAKE)),
            new WaitCommand(2).andThen(
                    new InstantCommand(()->superState.setFireIntent(FireIntent.FIRE))
                )
            )
        );

        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Intake In and Out
        new JoystickButton(driverJoyStick, Constants.Controls.BTN_INTAKE).onTrue(
             new InstantCommand(()-> superState.setFireIntent(FireIntent.INTAKE))
        ).onFalse(new InstantCommand(()-> superState.setFireIntent(FireIntent.IDLE)));
        new JoystickButton(driverJoyStick, Constants.Controls.BTN_SPIT).onTrue(
             new InstantCommand(()-> superState.setFireIntent(FireIntent.SPIT))
        ).onFalse(new InstantCommand(()-> superState.setFireIntent(FireIntent.IDLE)));

        // SHOOT + Intake! - Kicker and Flywheel and Intake
        new JoystickButton(driverJoyStick, Constants.Controls.BTN_FIRE_AND_INTAKE).onTrue(
            new InstantCommand(()->superState.setFireIntent(SuperStateSubsystem.FireIntent.FIREANDINTAKE))
        ).
        onFalse(
            new InstantCommand(()->superState.setFireIntent(SuperStateSubsystem.FireIntent.CLEAR)) //set to clear in comps
        );

        // SHOOT + Jostle! - Kicker and Flywheel and Intake Rotator Jostle
        new JoystickButton(driverJoyStick, Constants.Controls.BTN_FIRE).onTrue(
            new InstantCommand(()->superState.setFireIntent(SuperStateSubsystem.FireIntent.FIRE))
        ).
        onFalse(
            new InstantCommand(()->superState.setFireIntent(SuperStateSubsystem.FireIntent.CLEAR)) //set to clear in comps
        );
        
        //ResetGyro
        new JoystickButton(driverJoyStick, Constants.Controls.BTN_RESET_GYRO).onTrue(
            new InstantCommand(()-> swerveDrive.setYaw(0))
        );

        new JoystickButton(driverJoyStick, Constants.Controls.BTN_TOGGLE_TURRET_LOCK).onTrue(
            new InstantCommand(()-> superState.toggleTurretLock())
        );
        
        new JoystickButton(driverJoyStick, Constants.Controls.BTN_LOCK_WHEELS).whileTrue(
            new RunCommand(()-> swerveDrive.lockWheels(), swerveDrive)
        );

        // SysId characterization routines
        new SysIdBindings(driverJoyStick, swerveDrive, flywheel);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void setToIdle() {
        superState.setFireIntent(FireIntent.IDLE);
    }
}