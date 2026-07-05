package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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
import frc.robot.subsystems.swerve.SysId.SwerveDriveSysId;


public class SwerveDrive extends SubsystemBase {

    private ChassisSpeeds desiredChassisSpeeds;

    private final GyroIO gyro;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final OdometryIO odometry;
    private final OdometryIOInputs odometryInputs = new OdometryIOInputs();

    private RobotConfig config;

    private final SwerveDriveSysId sysId;

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

        sysId = new SwerveDriveSysId(this, modules);
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

    public void lockWheels() {
        SwerveModuleState[] desiredStates = new SwerveModuleState[] {
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)),   // FL
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),  // FR
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),  // BL
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(45))    // BR
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

        // loop the swerve modules!
        for (int i = 0; i < modules.length; i++) {
            modules[i].updateInputs(moduleInputs[i]);
            Logger.processInputs("Swerve/Module" + i, moduleInputs[i]);
            
            // Absolute value of module velocity
            // Used for comparing commanded and actual speeds
            Logger.recordOutput("Swerve/AbsoluteSpeed" + i, Math.abs(moduleInputs[i].driveVelocityMetersPerSecond));

            // log the absolute values of the modules for zeroing
            Logger.recordOutput("Swerve/RawAbsoluteEncoderDegrees" + i,
                MathUtil.inputModulus(moduleInputs[i].absoluteAngleDegrees, 0, 360));
            
        }
        Logger.recordOutput("Swerve/MyStates", getModuleStates());

        gyro.updateInputs(gyroInputs);
        Logger.processInputs("Swerve/Gyro", gyroInputs);

        Logger.recordOutput("Swerve/Pose", getPose());

        // Update odometry
        odometry.updateInputs(odometryInputs, getHeading(), getModulePositions());
        Logger.processInputs("Swerve/Odometry", odometryInputs);
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
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }

    // helper to point wheels before sysid runs (so they are ready freddy)
    public void pointWheelsForward() {
        for (int i = 0; i < modules.length; i++) {
            modules[i].setAzimuth(Rotation2d.kZero);
        }
    }
}
