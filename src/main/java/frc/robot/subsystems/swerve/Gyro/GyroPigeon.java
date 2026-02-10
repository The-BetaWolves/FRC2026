package frc.robot.subsystems.swerve.Gyro;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class GyroPigeon implements GyroIO {
    private final Pigeon2 gyro;
    
    public GyroPigeon() {
        this.gyro = new Pigeon2(Constants.Drivetrain.GYRO_ID);
        configureGyro();
    }

    public void configureGyro() {
        
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.yaw =  getYaw();
        inputs.yawDegrees = getYaw().getDegrees();
        inputs.gyroIsPowered = gyro.getYaw().getStatus().isOK();
    }

    @Override
    public Rotation2d getYaw() {
        return gyro.getRotation2d();
    }

    @Override
    public void zeroYaw() {
        gyro.reset();
    }
}
