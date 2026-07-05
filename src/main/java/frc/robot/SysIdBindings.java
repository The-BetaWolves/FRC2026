package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * SysId bindings will only work in Test mode (selected on the Driver Station).
 * Pick a routine on the dashboard "SysId Routine" chooser, then hold
 * button 5 to run it. Let go to stop.
 */
public class SysIdBindings {

    private final SendableChooser<Command> sysIdChooser = new SendableChooser<>();

    public SysIdBindings(Joystick joystick, SwerveDrive swerveDrive, Flywheel flywheel) {
        Trigger test = RobotModeTriggers.test();

        // Routine menu — add new characterizations here, not new buttons
        sysIdChooser.setDefaultOption("None", Commands.none());
        sysIdChooser.addOption("Drive Characterization", driveCharacterization(swerveDrive));
        sysIdChooser.addOption("Flywheel Characterization", flywheelCharacterization(flywheel));
        SmartDashboard.putData("SysId Routine (run in Test Mode)", sysIdChooser);

        // One button runs whatever the chooser has selected
        test.and(new JoystickButton(joystick, Constants.Controls.BTN_SYSID_RUN))
            .whileTrue(Commands.deferredProxy(sysIdChooser::getSelected));
    }

    private Command driveCharacterization(SwerveDrive swerveDrive) {
        double quasistaticTimeout = 4;
        double dynamicTimeout = 3;

        return new SequentialCommandGroup(
            new RunCommand(swerveDrive::pointWheelsForward, swerveDrive).withTimeout(1.0),
            swerveDrive.sysIdQuasistatic(SysIdRoutine.Direction.kForward).withTimeout(quasistaticTimeout),
            new WaitCommand(2),
            swerveDrive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).withTimeout(quasistaticTimeout),
            new WaitCommand(2),
            swerveDrive.sysIdDynamic(SysIdRoutine.Direction.kForward).withTimeout(dynamicTimeout),
            new WaitCommand(2),
            swerveDrive.sysIdDynamic(SysIdRoutine.Direction.kReverse).withTimeout(dynamicTimeout)
        );
    }

    private Command flywheelCharacterization(Flywheel flywheel) {
        double quasistaticTimeout = 5;
        double dynamicTimeout = 3;

        return new SequentialCommandGroup(
            flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward).withTimeout(quasistaticTimeout),
            new WaitCommand(3),
            flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).withTimeout(quasistaticTimeout),
            new WaitCommand(3),
            flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward).withTimeout(dynamicTimeout),
            new WaitCommand(3),
            flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse).withTimeout(dynamicTimeout)
        );
    }
}
