package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

    public SysIdBindings(Joystick joystick, SwerveDrive swerveDrive, Flywheel flywheel, IntakeSubsystem intake) {
        Trigger test = RobotModeTriggers.test();

        // In test mode the intake holds wherever it physically is — this command owns
        // the subsystem so the SuperState default command can't drive the rotator down
        test.whileTrue(Commands.startRun(intake::holdCurrentPosition, () -> {}, intake));

        // Routine menu: add new characterizations here
        sysIdChooser.setDefaultOption("None", Commands.none());
        sysIdChooser.addOption("Drive Characterization", swerveDrive.getSysId().fullCharacterization());
        sysIdChooser.addOption("Flywheel Characterization", flywheel.getSysId().fullCharacterization());
        SmartDashboard.putData("SysId Routine (run in Test Mode)", sysIdChooser);

        // One button runs whatever the chooser has selected
        test.and(new JoystickButton(joystick, Constants.Controls.BTN_SYSID_RUN))
            .whileTrue(Commands.deferredProxy(sysIdChooser::getSelected));
    }

}
