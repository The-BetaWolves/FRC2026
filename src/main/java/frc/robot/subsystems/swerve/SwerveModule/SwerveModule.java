package frc.robot.subsystems.swerve.SwerveModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;

import frc.robot.subsystems.swerve.SwerveModule.SwerveModuleIO.SwerveModuleIOInputs;

public class SwerveModule {
    private final String logPath; // Logger path: "Swerve/FL"
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    private final Alert driveMotorAlert;
    private final Alert angleMotorAlert;
    private final Alert absoluteEncoderAlert;

    public SwerveModule(String moduleName, int moduleNumber, SwerveModuleConstants constants) {
        this.logPath = "Swerve/" + moduleName;

        driveMotorAlert = new Alert(moduleName + " drive motor not powered!", AlertType.kError);
        angleMotorAlert = new Alert(moduleName + " angle motor not powered!", AlertType.kError);
        absoluteEncoderAlert = new Alert(moduleName + " CANcoder disconnected!", AlertType.kError);

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

        driveMotorAlert.set(!inputs.driveMotorIsPowered);
        angleMotorAlert.set(!inputs.angleMotorIsPowered);
        absoluteEncoderAlert.set(!inputs.absoluteEncoderIsConnected);
    }

    public Rotation2d getAbsoluteAngle() {
        return io.getAbsoluteAngle();
    }

    public void setAzimuth(Rotation2d angle) {
        io.setAzimuth(angle);
    }
}
