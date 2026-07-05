package frc.robot.subsystems.swerve.SwerveModule;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SwerveModuleIOInputs implements LoggableInputs {
    public double drivePositionMeters = 0.0;
    public double driveVelocityMetersPerSecond = 0.0;
    public double turnAngleRotations = 0.0;
    public double absoluteAngleDegrees = 0.0;
    public double absoluteAdjustedAngleDegrees = 0.0;
    public double relativeAngleDegrees = 0.0;
    public double desiredMetersPerSecond = 0.0;
    public boolean driveMotorIsPowered = false;
    public boolean angleMotorIsPowered = false;
    public boolean absoluteEncoderIsConnected = false;

    @Override
    public void toLog(LogTable table) {
        table.put("DrivePositionMeters", drivePositionMeters);
        table.put("DriveVelocityMetersPerSecond", driveVelocityMetersPerSecond);
        table.put("TurnAngleRotations", turnAngleRotations);
        table.put("AbsoluteAngleDegrees", absoluteAngleDegrees);
        table.put("AbsoluteAdjustedAngleDegrees", absoluteAdjustedAngleDegrees);
        table.put("RelativeAngleDegrees", relativeAngleDegrees);
        table.put("DesiredMetersPerSecond", desiredMetersPerSecond);
        table.put("DriveMotorIsPowered", driveMotorIsPowered);
        table.put("AngleMotorIsPowered", angleMotorIsPowered);
        table.put("AbsoluteEncoderIsConnected", absoluteEncoderIsConnected);
    }

    @Override
    public void fromLog(LogTable table) {
        drivePositionMeters = table.get("DrivePositionMeters", drivePositionMeters);
        driveVelocityMetersPerSecond = table.get("DriveVelocityMetersPerSecond", driveVelocityMetersPerSecond);
        turnAngleRotations = table.get("TurnAngleRotations", turnAngleRotations);
        absoluteAngleDegrees = table.get("AbsoluteAngleDegrees", absoluteAngleDegrees);
        absoluteAdjustedAngleDegrees = table.get("AbsoluteAdjustedAngleDegrees", absoluteAdjustedAngleDegrees);
        relativeAngleDegrees = table.get("RelativeAngleDegrees", relativeAngleDegrees);
        desiredMetersPerSecond = table.get("DesiredMetersPerSecond", desiredMetersPerSecond);
        driveMotorIsPowered = table.get("DriveMotorIsPowered", driveMotorIsPowered);
        angleMotorIsPowered = table.get("AngleMotorIsPowered", angleMotorIsPowered);
        absoluteEncoderIsConnected = table.get("AbsoluteEncoderIsConnected", absoluteEncoderIsConnected);
    }
}