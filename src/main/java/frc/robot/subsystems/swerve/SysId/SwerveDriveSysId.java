package frc.robot.subsystems.swerve.SysId;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveModule.SwerveModule;

public class SwerveDriveSysId {

    private static final String[] MODULE_NAMES = { 
        "front-left", 
        "front-right", 
        "back-left", 
        "back-right"
    };

    // Mutable holders, persists to avoid reallocation.
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    private final MutDistance m_distance = Meters.mutable(0);
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

    private final SwerveModule[] modules;
    private final SysIdRoutine routine;

    public SwerveDriveSysId(SwerveDrive drive, SwerveModule[] modules) {
        this.modules = modules;
        routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(2.0).per(Seconds), // ramp rate
                Volts.of(6), // step voltage
                Seconds.of(10) // timeout
            ),
            new SysIdRoutine.Mechanism(
                this::driveAllModules,
                this::logAllModules,
                drive
            )
        );
    }

    public Command quasistatic(SysIdRoutine.Direction direction) { 
        return routine.quasistatic(direction); 
    }

    public Command dynamic(SysIdRoutine.Direction direction) { 
        return routine.dynamic(direction);
    }

    private void driveAllModules(Voltage volts) {
        for (int i = 0; i < modules.length; i++) {
            modules[i].setAzimuth(Rotation2d.kZero);
            modules[i].setDriveMotorVoltage(volts);
        }
    }

    private void logAllModules(SysIdRoutineLog log) {
        for(int i = 0; i < modules.length; i++) {
            log.motor(MODULE_NAMES[i])
                .voltage(m_appliedVoltage.mut_replace(modules[i].getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                .linearPosition(m_distance.mut_replace(
                modules[i].getRelativeEncoder().getPosition(), Meters))
                .linearVelocity(m_velocity.mut_replace(
                modules[i].getRelativeEncoder().getVelocity(), MetersPerSecond));
        }
    }
}
