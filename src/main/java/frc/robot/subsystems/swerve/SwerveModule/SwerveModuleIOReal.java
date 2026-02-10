package frc.robot.subsystems.swerve.SwerveModule;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConfig;
import frc.robot.util.SwerveModuleAngleOptimizer;

/**
 * Represents a single swerve drive module (wheel pod) on the robot.
 *
 * This class controls the module's drive motor (for translation) and turning motor (for steering),
 * and uses an absolute encoder to track the module's steering angle.
 *
 * Responsibilities:
 * - Converts a desired speed and angle into optimized motor commands (setDesiredState)
 * - Reports the module's current speed and angle (getState)
 * - Reports the module's distance traveled and angle for odometry (getPosition)
 *
 * This class is used by the overall SwerveDrive subsystem to coordinate field-relative movement
 * and pose tracking using WPILib's kinematics and odometry tools.
 */


public class SwerveModuleIOReal implements SwerveModuleIO {
    private int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d currentReferenceAngle;

    private final SparkMax driveMotor;
    private final SparkMax angleMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder angleEncoder;
    private final CANcoder absoluteEncoder;


    private final SparkClosedLoopController drivePID;
    private final SparkClosedLoopController anglePID;

    private SwerveModuleConstants constants;

    public SwerveModuleIOReal(int moduleNumber, SwerveModuleConstants constants) {
        this.constants = constants;
        this.moduleNumber = moduleNumber;
        this.angleOffset = constants.angleOffset;

        driveMotor = new SparkMax(constants.driveMotorId, MotorType.kBrushless);
        drivePID = driveMotor.getClosedLoopController();
        driveEncoder = driveMotor.getEncoder();

        angleMotor = new SparkMax(constants.angleMotorId, MotorType.kBrushless);
        anglePID = angleMotor.getClosedLoopController();
        angleEncoder = angleMotor.getEncoder();

        absoluteEncoder = new CANcoder(constants.absoluteEncoderId);

        configureDriveMotor();
        configureCANcoder();
        configureAngleMotor();

        currentReferenceAngle = getAdjustedAbsoluteAngle();

    }

    public void configureDriveMotor() {
        driveMotor.configure(
            new SparkMaxConfig()
                .idleMode(IdleMode.kBrake)
                .inverted(constants.driveInverted)
                .smartCurrentLimit(SwerveConfig.DRIVE_CURRENT_LIMIT)
                .openLoopRampRate(SwerveConfig.DRIVE_RAMP)
            .apply(
                new ClosedLoopConfig()
                .pid(SwerveConfig.DRIVE_kP, SwerveConfig.DRIVE_kI, SwerveConfig.DRIVE_kD)
            )
            .apply(
                new EncoderConfig()
                .positionConversionFactor(SwerveConfig.DRIVE_POSITION_CONVERSION)
                .velocityConversionFactor(SwerveConfig.DRIVE_VELOCITY_CONVERSION)
            ), 
            ResetMode.kNoResetSafeParameters, 
            PersistMode.kNoPersistParameters
        ); 
    }

    private void configureAngleMotor() {
        angleMotor.configure(
            new SparkMaxConfig()
                .idleMode(SparkBaseConfig.IdleMode.kCoast)
                .inverted(constants.angleInverted)
                .smartCurrentLimit(SwerveConfig.ANGLE_CURRENT_LIMIT)
                .openLoopRampRate(SwerveConfig.ANGLE_RAMP)
            .apply(
                new EncoderConfig()
                .positionConversionFactor(SwerveConfig.ANGLE_POSITION_CONVERSION)
                .velocityConversionFactor(SwerveConfig.ANGLE_POSITION_CONVERSION / 60.0)
            )
            .apply(
                new ClosedLoopConfig()
                .pid(
                    SwerveConfig.ANGLE_kP, 
                    SwerveConfig.ANGLE_kI,
                    SwerveConfig.ANGLE_kD
                )
            ),
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters);

        resetToAbsolute();
    }

    private void configureCANcoder() {
        absoluteEncoder.getConfigurator()
        .apply(
            new CANcoderConfiguration()
            .withMagnetSensor(
                new MagnetSensorConfigs()
                    .withAbsoluteSensorDiscontinuityPoint(0.5) // Sets range to [-0.5, 0.5) rotations
                    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                    //.withMagnetOffset(constants.angleOffset.getRotations()) // Apply the pre-determined offset here
            )
        );
    }

    public Rotation2d getRelativeAngle() {
        return Rotation2d.fromRotations(angleEncoder.getPosition());
    }

    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromRotations(absoluteEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public Rotation2d getAdjustedAbsoluteAngle() {
        return Rotation2d.fromDegrees(getAbsoluteAngle().getDegrees() - angleOffset.getDegrees());
    }

    public void resetToAbsolute() {
        angleEncoder.setPosition(getAdjustedAbsoluteAngle().getRotations());
    }

    // Returns instantaneous speed and angle
    // used by driving logic, calculating chassis speed, telemetry
    // public SwerveModuleState getState() {
    //     return new SwerveModuleState(driveEncoder.getVelocity(), getRelativeAngle());
    // }

    // Returns total distance driven and angle
    // used by odometry/pose estimation
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getRelativeAngle());
    }

    // Tell the pod what direction and speed you want it to go
    @Override
    public void setDesiredState(SwerveModuleState desired) {
        SwerveModuleState optimizedState = SwerveModuleAngleOptimizer.optimize(desired, getRelativeAngle());

        //Velocity in RPM  (convert from m/s if needed)
        //Position in rotations
        double targetDriveRPM = optimizedState.speedMetersPerSecond / SwerveConfig.DRIVE_VELOCITY_CONVERSION;
        drivePID.setSetpoint(
            targetDriveRPM, SparkBase.ControlType.kVelocity);

        //Prevent rotating module if speed is less than 1%. Prevents Jittering.
        // Rotation2d angle = (Math.abs(desired.speedMetersPerSecond) <= (Constants.Drivetrain.MAXIMUM_CHASSIS_VELOCITY * 0.01)) ?
        //     currentReferenceAngle :
        //     desired.angle; 

        anglePID.setSetpoint(optimizedState.angle.getRotations(), ControlType.kPosition);

        currentReferenceAngle = optimizedState.angle;
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.drivePositionMeters = driveEncoder.getPosition();
        inputs.driveVelocityMetersPerSecond = driveEncoder.getVelocity();
        inputs.turnAngleRotations = getRelativeAngle().getRotations();
        inputs.absoluteAngleDegrees = getAbsoluteAngle().getDegrees();
        inputs.absoluteAdjustedAngleDegrees = getAdjustedAbsoluteAngle().getDegrees();
        inputs.relativeAngleDegrees = getRelativeAngle().getDegrees();
    }
}