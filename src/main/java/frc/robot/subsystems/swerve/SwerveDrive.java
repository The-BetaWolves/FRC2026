package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import frc.robot.Constants;
import frc.robot.subsystems.swerve.Gyro.GyroIO;
import frc.robot.subsystems.swerve.Gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.swerve.Gyro.GyroPigeon;
import frc.robot.subsystems.swerve.Gyro.GyroSim;
import frc.robot.subsystems.swerve.Odometry.OdometryIO;
import frc.robot.subsystems.swerve.Odometry.OdometryIOInputs;
import frc.robot.subsystems.swerve.Odometry.OdometryReal;
import frc.robot.subsystems.swerve.Odometry.OdometrySim;
import frc.robot.subsystems.swerve.SwerveModule.SwerveModule;
import frc.robot.subsystems.swerve.SwerveModule.SwerveModuleIOInputs;


public class SwerveDrive extends SubsystemBase {

    private ChassisSpeeds desiredChassisSpeeds;

    private final GyroIO gyro;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final OdometryIO odometry;
    private final OdometryIOInputs odometryInputs = new OdometryIOInputs();

    private RobotConfig config;

    SysIdRoutine sysIdRoutine;

    // Modules
    private final SwerveModule[] modules = {
        new SwerveModule("Swerve/FL",0, Constants.SwerveModules.FRONT_LEFT),
        new SwerveModule("Swerve/FR", 1, Constants.SwerveModules.FRONT_RIGHT),
        new SwerveModule("Swerve/BL", 2, Constants.SwerveModules.BACK_LEFT),
        new SwerveModule("Swerve/BR", 3, Constants.SwerveModules.BACK_RIGHT)
    };

    private final SwerveModuleIOInputs[] moduleInputs = {
        new SwerveModuleIOInputs(),
        new SwerveModuleIOInputs(),
        new SwerveModuleIOInputs(),
        new SwerveModuleIOInputs()
    };

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        Constants.SwerveModules.FRONT_LEFT.modulePosition,
        Constants.SwerveModules.FRONT_RIGHT.modulePosition,
        Constants.SwerveModules.BACK_LEFT.modulePosition,
        Constants.SwerveModules.BACK_RIGHT.modulePosition
    );

    private Pose2d startingPose = new Pose2d();

    private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, gyroInputs.yaw, getModulePositions(), startingPose);

      // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutAngle m_angle = Radians.mutable(0);
     // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);


    public SwerveDrive() {

        // Gyro
        if(RobotBase.isSimulation()) {
            gyro = new GyroSim();
            odometry = new OdometrySim(kinematics, getHeading(), getModulePositions());
        } else {
            gyro = new GyroPigeon();
            odometry = new OdometryReal(kinematics, getHeading(), getModulePositions());
        }
        gyro.zeroYaw();

        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
   
        // Configure AutoBuilder last
        AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
            },
            this // Reference to this subsystem to set requirements
        );

        sysIdRoutine =
            new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(
                    Volts.of(2).per(Seconds),
                    Volts.of(6),
                    Time.ofRelativeUnits(3, Seconds)
                ),
                new SysIdRoutine.Mechanism(
                    // Tell SysId how to plumb the driving voltage to the motor(s).
                    (volts) -> {
                        driveModuleWithVoltage(0, volts);
                        driveModuleWithVoltage(1, volts);
                        driveModuleWithVoltage(2, volts);
                        driveModuleWithVoltage(3, volts);
                    },
                    // Tell SysId how to record a frame of data for each motor on the mechanism being
                    // characterized.
                    log -> {
                        // Record a frame for the shooter motor.
                        log.motor("front-left")
                            .voltage(
                                m_appliedVoltage.mut_replace(
                                    modules[0].getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                            .angularPosition(m_angle.mut_replace(modules[0].getRelativeEncoder().getPosition(), Rotations))
                            .angularVelocity(
                                m_velocity.mut_replace((modules[0].getRelativeEncoder().getVelocity()), RotationsPerSecond));
                        log.motor("front-right")
                            .voltage(
                                m_appliedVoltage.mut_replace(
                                    modules[1].getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                            .angularPosition(m_angle.mut_replace(modules[1].getRelativeEncoder().getPosition(), Rotations))
                            .angularVelocity(
                                m_velocity.mut_replace((modules[1].getRelativeEncoder().getVelocity()), RotationsPerSecond));
                        log.motor("back-left")
                            .voltage(
                                m_appliedVoltage.mut_replace(
                                    modules[2].getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                            .angularPosition(m_angle.mut_replace(modules[2].getRelativeEncoder().getPosition(), Rotations))
                            .angularVelocity(
                                m_velocity.mut_replace((modules[2].getRelativeEncoder().getVelocity()), RotationsPerSecond));
                        log.motor("back-right")
                            .voltage(
                                m_appliedVoltage.mut_replace(
                                    modules[3].getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                            .angularPosition(m_angle.mut_replace(modules[3].getRelativeEncoder().getPosition(), Rotations))
                            .angularVelocity(
                                m_velocity.mut_replace((modules[3].getRelativeEncoder().getVelocity()), RotationsPerSecond));
                    },
                    // Tell SysId to make generated commands require this subsystem, suffix test state in
                    // WPILog with this subsystem's name ("shooter")
                    this));
    }

    public Rotation2d getHeading() {
        return gyro.getYaw();
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        if (RobotState.isAutonomous()) {
            gyro.setYaw(pose.getRotation().getDegrees());
            poseEstimator.resetPosition(pose.getRotation(), getModulePositions(), pose);
        } else {
            poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
        }
    }

    public void setYaw(double yaw) {
        gyro.setYaw(yaw);
    }

    /** Resets odometry to a known pose */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPose(getHeading(), getModulePositions(), pose);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(this.getModuleStates());
    }

    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), gyro.getYaw());
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    // Drive Field Relative (x, y z)
    public void drive(double desiredXVelocity, double desiredYVelocity, double desiredRotationalVelocity) {
        this.drive(desiredXVelocity, desiredYVelocity, desiredRotationalVelocity, true);
    }

    // Drive (x, y, z, isFieldRelative)
    public void drive(double desiredXVelocity, double desiredYVelocity, double desiredRotationalVelocity, boolean isFieldRelative) {
        if(isFieldRelative) {
            this.desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredXVelocity, desiredYVelocity, desiredRotationalVelocity, getPose().getRotation());
        } else {
            this.desiredChassisSpeeds = new ChassisSpeeds(desiredXVelocity, desiredYVelocity, desiredRotationalVelocity);
        }            
    }

    public void driveWithVoltage(Voltage voltage) {
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDriveMotorVoltage(voltage);
        }
    }

    public void driveModuleWithVoltage(int id, Voltage voltage) {
        modules[id].setDriveMotorVoltage(voltage);
    }

    public void lockWheels() {
        SwerveModuleState[] desiredStates = new SwerveModuleState[]{
            new SwerveModuleState(
                LinearVelocity.ofRelativeUnits(
                    0.0, 
                    LinearVelocityUnit.combine(Units.Meters, Units.Seconds)),
                        new Rotation2d(45)),
            new SwerveModuleState(
                LinearVelocity.ofRelativeUnits(
                    0.0, 
                    LinearVelocityUnit.combine(Units.Meters, Units.Seconds)),
                        new Rotation2d(-45)),
            new SwerveModuleState(
                LinearVelocity.ofRelativeUnits(
                    0.0, 
                    LinearVelocityUnit.combine(Units.Meters, Units.Seconds)),
                        new Rotation2d(-45)),
            new SwerveModuleState(
                LinearVelocity.ofRelativeUnits(
                    0.0, 
                    LinearVelocityUnit.combine(Units.Meters, Units.Seconds)),
                        new Rotation2d(45))
        
        };
        setModuleStates(desiredStates);
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        this.desiredChassisSpeeds = chassisSpeeds;
    }

    // send the values to the pods
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(desiredStates[i]);
        }

        // Log Desired Angle and Speed of Modules for Advantage Scope swerve pod tuning
        Logger.recordOutput("Swerve/MyDesiredStates", desiredStates);
    }

    @Override
    public void periodic() {
        if(desiredChassisSpeeds != null) {
            SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(desiredChassisSpeeds);
            setModuleStates(desiredStates);
        }
        log();
        desiredChassisSpeeds = null;

        poseEstimator.update(getHeading(), getModulePositions());
    }

    public void log() {
         // Update inputs for logging
         for (int i = 0; i < modules.length; i++) {
            modules[i].updateInputs(moduleInputs[i]);
            Logger.processInputs("Swerve/Module" + i, moduleInputs[i]);
            //Just for checking if PIDF values are correct
            Logger.recordOutput("Swerve/AbsoluteDesiredSpeed" + i, Math.abs(moduleInputs[i].driveVelocityMetersPerSecond));



            double rawAbsoluteEncoderValue = modules[i].getAbsoluteAngle().getDegrees();
            if(rawAbsoluteEncoderValue < 0) {
                SmartDashboard.putNumber("swerve raw absolute encoder value " + i, 360 + modules[i].getAbsoluteAngle().getDegrees());
            } else {
                SmartDashboard.putNumber("swerve raw absolute encoder value " + i, modules[i].getAbsoluteAngle().getDegrees());
            }
            
        }
        Logger.recordOutput("Swerve/MyStates", getModuleStates());

        //gyro.updateInputs(gyroInputs, getChassisSpeeds().omegaRadiansPerSecond);
        gyro.updateInputs(gyroInputs);
        Logger.processInputs("Swerve/Gyro", gyroInputs);

        Logger.recordOutput("Swerve/Pose", getPose());

        // Update odometry
        odometry.updateInputs(odometryInputs, getHeading(), getModulePositions());
        Logger.processInputs("Swerve/Odometry", odometryInputs);

        //SmartDashboard.putNumber("odometry X", odometry.getPoseMeters().getX()); 
        //SmartDashboard.putNumber("odometry Y", odometry.getPoseMeters().getY());
    }

    /** Adds a new timestamped vision measurement. */
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs) {
        //poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
        poseEstimator.addVisionMeasurement(new Pose2d(visionRobotPoseMeters.getX(), visionRobotPoseMeters.getY(), getHeading()), timestampSeconds, visionMeasurementStdDevs);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}
