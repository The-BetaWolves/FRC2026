package frc.robot.subsystems.swerve.SwerveModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;

import frc.robot.subsystems.swerve.SwerveModule.SwerveModuleIO.SwerveModuleIOInputs;

public class SwerveModule {
    private final String logPath;
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    public SwerveModule(String logPath, int moduleNumber, SwerveModuleConstants constants) {
        this.logPath = logPath;

        if(RobotBase.isSimulation()) {
            io = new SwerveModuleIOSim(constants);
        } else {
            io = new SwerveModuleIOReal(moduleNumber, constants);
        }
    }

    public void setDesiredState(SwerveModuleState desired) {
        io.setDesiredState(desired);
    }

    public void setDriveMotorVoltage(Voltage voltage) {
        io.setDriveMotorVoltage(voltage);
    }

    public RelativeEncoder getRelativeEncoder() {
        return io.getRelativeEncoder();
    }

    public double getAppliedOutput() {
        return io.getAppliedOutput();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            inputs.driveVelocityMetersPerSecond,
            Rotation2d.fromRotations(inputs.turnAngleRotations)
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            inputs.drivePositionMeters,
            Rotation2d.fromRotations(inputs.turnAngleRotations)
        );
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs(logPath, inputs);

        // 0-360 view of the absolute encoder, matches the angleOffset convention for zeroing
        Logger.recordOutput(logPath + "/RawAbsoluteEncoderDegrees",
            MathUtil.inputModulus(inputs.absoluteAngleDegrees, 0, 360));
    }

    public Rotation2d getAbsoluteAngle() {
        return io.getAbsoluteAngle();
    }

    public void setAzimuth(Rotation2d angle) {
        io.setAzimuth(angle);
    }

    public SwerveModuleIOInputs getInputs() {
        return inputs;
    }
}
