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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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

        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            swerveDrive.setDefaultCommand(
                new TeleopDriveCommand(
                    swerveDrive,
                    ()-> (MathUtil.applyDeadband(driverJoyStick.getY(), Constants.Controls.Y_DEADBAND)),
                    ()-> (MathUtil.applyDeadband(driverJoyStick.getX(), Constants.Controls.Y_DEADBAND)),
                    ()-> -(MathUtil.applyDeadband(driverJoyStick.getZ(), Constants.Controls.ANGLE_JOYSTICK_DEADBAND)),
                    superState
                )
            );
        } else {
            swerveDrive.setDefaultCommand(
                new TeleopDriveCommand(
                    swerveDrive,
                    ()-> -(MathUtil.applyDeadband(driverJoyStick.getY(), Constants.Controls.Y_DEADBAND)),
                    ()-> -(MathUtil.applyDeadband(driverJoyStick.getX(), Constants.Controls.Y_DEADBAND)),
                    ()-> -(MathUtil.applyDeadband(driverJoyStick.getTwist(), Constants.Controls.ANGLE_JOYSTICK_DEADBAND)),
                    superState
                )
            );
        }

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

        printDebugValues();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Intake In and Out 
        new JoystickButton(driverJoyStick, 3).onTrue(
             new InstantCommand(()-> superState.setFireIntent(FireIntent.INTAKE))
        ).onFalse(new InstantCommand(()-> superState.setFireIntent(FireIntent.IDLE)));
        new JoystickButton(driverJoyStick, 4).onTrue(
             new InstantCommand(()-> superState.setFireIntent(FireIntent.SPIT))
        ).onFalse(new InstantCommand(()-> superState.setFireIntent(FireIntent.IDLE)));

        // SHOOT + Intake! - Kicker and Flywheel and Intake
        new JoystickButton(driverJoyStick, 1).onTrue(
            new InstantCommand(()->superState.setFireIntent(SuperStateSubsystem.FireIntent.FIREANDINTAKE))
        ).
        onFalse(
            new InstantCommand(()->superState.setFireIntent(SuperStateSubsystem.FireIntent.CLEAR)) //set to clear in comps
        );

        // SHOOT + Jostle! - Kicker and Flywheel and Intake Rotator Jostle
        new JoystickButton(driverJoyStick, 2).onTrue(
            new InstantCommand(()->superState.setFireIntent(SuperStateSubsystem.FireIntent.FIRE))
        ).
        onFalse(
            new InstantCommand(()->superState.setFireIntent(SuperStateSubsystem.FireIntent.CLEAR)) //set to clear in comps
        );
        
        //ResetGyro
        new JoystickButton(driverJoyStick, 15).onTrue(
            new InstantCommand(()-> swerveDrive.setYaw(0))
        );

        new JoystickButton(driverJoyStick, 16).onTrue(
            new InstantCommand(()-> superState.toggleTurretLock())
        );
         
        /*
        // Turret CW
        new JoystickButton(driverJoyStick, 10).whileTrue(
            new RunCommand(()-> turret.incrementOffset(0.25), turret)
        );
        // Turret CCW
        new JoystickButton(driverJoyStick, 5).whileTrue(
            new RunCommand(()-> turret.incrementOffset(-0.25), turret)
        );
          */

        
        new JoystickButton(driverJoyStick, 14).whileTrue(
            new RunCommand(()-> swerveDrive.lockWheels(), swerveDrive)
        );

        /*
        new JoystickButton(driverJoyStick, 5).whileTrue(
            flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        );
        new JoystickButton(driverJoyStick, 6).whileTrue(
            flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        );
        new JoystickButton(driverJoyStick, 10).whileTrue(
            flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward)
        );
        new JoystickButton(driverJoyStick, 9).whileTrue(
            flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        );
         */
        
        double quasistaticTimeout = 4;
        double dynamicTimeout = 3;
        new JoystickButton(driverJoyStick, 5).whileTrue(
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    swerveDrive.sysIdQuasistatic(SysIdRoutine.Direction.kForward),
                    new WaitCommand(quasistaticTimeout)
                ),

                new WaitCommand(2),
                new ParallelRaceGroup(
                    swerveDrive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
                    new WaitCommand(quasistaticTimeout)
                ),

                new WaitCommand(2),
                new ParallelRaceGroup(
                    swerveDrive.sysIdDynamic(SysIdRoutine.Direction.kForward),
                    new WaitCommand(dynamicTimeout)
                ),

                new WaitCommand(2),
                new ParallelRaceGroup(
                    swerveDrive.sysIdDynamic(SysIdRoutine.Direction.kReverse),
                    new WaitCommand(dynamicTimeout)
                )
            )
        );
    }

    private void printDebugValues() {
        // add smart dashboard debug calls here instead of in subsystems

        //double[] adjustedTargetArray = {superState.getAdjustedTargetPose().getX(), superState.getAdjustedTargetPose().getY()};
        //SmartDashboard.putNumberArray("Adjusted Target Position Meters", adjustedTargetArray);
        //SmartDashboard.putNumber("Distance to Target", superState.getDistanceToTarget());

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void setToIdle() {
        superState.setFireIntent(FireIntent.IDLE);
    }
}