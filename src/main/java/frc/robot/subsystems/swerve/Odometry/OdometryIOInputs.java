package frc.robot.subsystems.swerve.Odometry;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose2d;

public class OdometryIOInputs implements LoggableInputs {
    public Pose2d robotPose = new Pose2d();

    @Override
    public void toLog(LogTable table) {
        table.put("RobotPose", robotPose);
    }

    @Override
    public void fromLog(LogTable table) {
        robotPose = table.get("RobotPose", robotPose);
    }
}
