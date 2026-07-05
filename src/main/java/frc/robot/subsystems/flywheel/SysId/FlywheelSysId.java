package frc.robot.subsystems.flywheel.SysId;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;

public class FlywheelSysId {

    // Mutable holders for unit-safe values, persisted to avoid reallocation
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    private final MutAngle m_angle = Rotations.mutable(0);
    private final MutAngularVelocity m_velocity = RPM.mutable(0);

    private final FlywheelIO io;
    private final SysIdRoutine routine;

    public FlywheelSysId(Flywheel flywheel, FlywheelIO io) {
        this.io = io;
        routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Seconds), // ramp rate
                Volts.of(4),                // step voltage
                Seconds.of(5)               // timeout (now matches the quasistatic timout in SysIdBindings)
            ),
            new SysIdRoutine.Mechanism(
                io::setMotorVoltage,
                this::logMotor,
                flywheel
            )
        );
    }

    public Command quasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command dynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    private void logMotor(SysIdRoutineLog log) {
        log.motor("shooter-wheel")
            .voltage(m_appliedVoltage.mut_replace(
                io.getMotorOutput() * RobotController.getBatteryVoltage(), Volts))
            .angularPosition(m_angle.mut_replace(io.getEncoder().getPosition(), Rotations))
            // Spark encoder velocity is native RPM (no conversion factor configured)
            .angularVelocity(m_velocity.mut_replace(io.getEncoder().getVelocity(), RPM));
    }
}
