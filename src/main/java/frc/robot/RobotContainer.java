// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.camera2Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.subsystems.ClimberSubsystem;
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
    public Flywheel flywheel = new Flywheel();
    public TurretSubsystem turret = new TurretSubsystem();
    public ClimberSubsystem climber = new ClimberSubsystem();
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
                    ()-> -(MathUtil.applyDeadband(driverJoyStick.getTwist(), Constants.Controls.ANGLE_JOYSTICK_DEADBAND))
                )
            );
        } else {
            swerveDrive.setDefaultCommand(
                new TeleopDriveCommand(
                    swerveDrive,
                    ()-> -(MathUtil.applyDeadband(driverJoyStick.getY(), Constants.Controls.Y_DEADBAND)),
                    ()-> -(MathUtil.applyDeadband(driverJoyStick.getX(), Constants.Controls.Y_DEADBAND)),
                    ()-> -(MathUtil.applyDeadband(driverJoyStick.getTwist(), Constants.Controls.ANGLE_JOYSTICK_DEADBAND))
                )
            );
        }
        

        //swerveDrive.resetPose(new Pose2d(3.7, 4.02, (new Rotation2d())));
        //swerveDrive.resetPose(new Pose2d(0.5, 7.25, (new Rotation2d()))); //Math.PI
        //swerveDrive.resetPose(new Pose2d(0, 0, (new Rotation2d((Math.PI)))));
        
        //NamedCommands.registerCommand("setToStop", new InstantCommand(()->superState.setFireIntent(FireIntent.STOP)));
        NamedCommands.registerCommand("setToIdle", new InstantCommand(()->superState.setFireIntent(FireIntent.IDLE)));
        NamedCommands.registerCommand("setToFire", new InstantCommand(()->superState.setFireIntent(FireIntent.FIRE)));
        NamedCommands.registerCommand("setToClear", new InstantCommand(()->superState.setFireIntent(FireIntent.CLEAR)));
        NamedCommands.registerCommand("setToIntake", new InstantCommand(()->superState.setFireIntent(FireIntent.INTAKE)));
        NamedCommands.registerCommand("setToFireAndIntake", new InstantCommand(()->superState.setFireIntent(FireIntent.FIREANDINTAKE)));
        
        NamedCommands.registerCommand("setClimberUp", new InstantCommand(()->climber.setSetpoint(365)));
        NamedCommands.registerCommand("setClimberDown", new InstantCommand(()->climber.setSetpoint(1)));

        NamedCommands.registerCommand("staticFire", new ParallelCommandGroup(
            new WaitCommand(Constants.Flywheel.firingTimeSeconds),
            new InstantCommand(()->superState.setFireIntent(FireIntent.FIRE))).andThen(
                new InstantCommand(()->superState.setFireIntent(FireIntent.CLEAR))
            ));

        NamedCommands.registerCommand("setIntakeUp", new InstantCommand(()-> intake.setSetpoint(Constants.Intake.maxRotatorDegree)));

        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();

        printDebugValues();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        /* 
        new JoystickButton(driverJoyStick, 3).onTrue(
        new InstantCommand(()-> tester.setSpeed(0.7), tester)
        ).onFalse(new InstantCommand(()-> tester.setSpeed(0.0), tester));
        */

        // Intake In
        // new JoystickButton(driverJoyStick, 4).onTrue(
        //     new InstantCommand(()-> intake.setRollerSpeed(0.8), intake)
        // ).onFalse(new InstantCommand(()-> intake.setRollerSpeed(0.0), intake));
        new JoystickButton(driverJoyStick, 3).onTrue(
             new InstantCommand(()-> superState.setFireIntent(FireIntent.INTAKE))
        ).onFalse(new InstantCommand(()-> superState.setFireIntent(FireIntent.IDLE)));

        new JoystickButton(driverJoyStick, 4).onTrue(
             new InstantCommand(()-> superState.setFireIntent(FireIntent.SPIT))
        ).onFalse(new InstantCommand(()-> superState.setFireIntent(FireIntent.IDLE)));

        // Intake Rotator
        /* 
        new JoystickButton(driverJoyStick, 5).whileTrue(
            new RunCommand(()-> intake.increaseSetpoint(), intake));
        new JoystickButton(driverJoyStick, 10).whileTrue(
            new RunCommand(()-> intake.decreaseSetpoint(), intake));
        */

        //May need to add the intake as a required subsystem
        new JoystickButton(driverJoyStick, 5).whileTrue(
            new RunCommand(()-> superState.incrementIntakeRotatorSetpoint(1.5)));
        new JoystickButton(driverJoyStick, 10).whileTrue(
            new RunCommand(()-> superState.incrementIntakeRotatorSetpoint(-1.5)));
        
        
        new JoystickButton(driverJoyStick, 6).onTrue(
            new InstantCommand(()-> intake.setSetpoint(Constants.Intake.maxRotatorDegree))
        );
        new JoystickButton(driverJoyStick, 9).onTrue(
            new InstantCommand(()-> intake.setSetpoint(Constants.Intake.minRotatorDegree))
        );
         

        // SHOOT! - Kicker and Flywheel
        new JoystickButton(driverJoyStick, 1).onTrue(
            new InstantCommand(()->superState.setFireIntent(SuperStateSubsystem.FireIntent.FIREANDINTAKE))
        ).
        onFalse(
            new InstantCommand(()->superState.setFireIntent(SuperStateSubsystem.FireIntent.CLEAR)) //set to clear in comps
        );

        new JoystickButton(driverJoyStick, 2).onTrue(
            new InstantCommand(()->superState.setFireIntent(SuperStateSubsystem.FireIntent.FIRE))
        ).
        onFalse(
            new InstantCommand(()->superState.setFireIntent(SuperStateSubsystem.FireIntent.CLEAR)) //set to clear in comps
        );
        
        //ResetGyro
        new JoystickButton(driverJoyStick, 16).onTrue(
            new InstantCommand(()-> swerveDrive.setYaw(0))
        );

        //Climber
        /*
        new JoystickButton(driverJoyStick, 7).onTrue(
            new InstantCommand(()-> climber.setSpeed(0.3), climber)
        ).onFalse(new InstantCommand(()-> climber.setSpeed(0.0), climber));
        new JoystickButton(driverJoyStick, 8).onTrue(
            new InstantCommand(()-> climber.setSpeed(-0.3), climber)
        ).onFalse(new InstantCommand(()-> climber.setSpeed(0.0), climber));
         */

        new JoystickButton(driverJoyStick, 7).whileTrue(
            new RunCommand(()-> climber.incrementSetpoint(3.0), climber));
        new JoystickButton(driverJoyStick, 8).whileTrue(
            new RunCommand(()-> climber.incrementSetpoint(-3.0), climber));

        new JoystickButton(driverJoyStick, 13).onTrue(new InstantCommand(()-> climber.setSetpoint(365)));
        new JoystickButton(driverJoyStick, 14).onTrue(new InstantCommand(()-> climber.setSetpoint(1)));


        /* 
        new JoystickButton(driverJoyStick, 2).onTrue(
        new InstantCommand(()-> indexer.setSpeed(0.5), indexer)
        ).onFalse(new InstantCommand(()-> indexer.setSpeed(0.0), indexer));
        new JoystickButton(driverJoyStick, 1).onTrue(
        new InstantCommand(()-> shooter.setSpeed(0.7), shooter)
        ).onFalse(new InstantCommand(()-> shooter.setSpeed(0.0), shooter));
        */
        
        // Turret CW
        
        new JoystickButton(driverJoyStick, 12).whileTrue(
            new RunCommand(()-> turret.incrementOffset(0.05), turret)
        );
        // Turret CCW
        new JoystickButton(driverJoyStick, 15).whileTrue(
            new RunCommand(()-> turret.incrementOffset(-0.05), turret)
        );
        
        PathConstraints constraints = new PathConstraints(1.0, 3.0, 2 * Math.PI, 3 * Math.PI);
        PathPlannerPath toTowerPath;
        if (swerveDrive.getPose().getY() < 4) {
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                swerveDrive.getPose(),
                new Pose2d(0.990, 3.9, new Rotation2d(0)),
                new Pose2d(0.990, 4.4, new Rotation2d(-90))
            );

            toTowerPath = new PathPlannerPath(
                waypoints,
                constraints,
                null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );
        } else {
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                swerveDrive.getPose(),
                new Pose2d(0.990, 3.5, new Rotation2d(0)),
                new Pose2d(0.990, 3, new Rotation2d(90))
            );

            toTowerPath = new PathPlannerPath(
                waypoints,
                constraints,
                null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                new GoalEndState(0.0, Rotation2d.fromDegrees(180)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );
        }
        //toTowerPath.preventFlipping = true;

        new JoystickButton(driverJoyStick, 11).whileTrue(
            AutoBuilder.followPath(toTowerPath)
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
        //return new TestBangBang(swerveDrive);
        //return Commands.print("No autonomous command configured");
    }

    public void setToIdle() {
        superState.setFireIntent(FireIntent.IDLE);
    }
}
