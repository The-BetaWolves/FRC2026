package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.util.AllianceUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
import frc.robot.subsystems.swerve.SwerveModule.SwerveModule;
import frc.robot.subsystems.swerve.SysId.SwerveDriveSysId;

public class SwerveDrive extends SubsystemBase {

    private ChassisSpeeds desiredChassisSpeeds;

    private final GyroIO gyro;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Alert gyroAlert = new Alert("Gyro not powered!", AlertType.kError);

    private SwerveDriveOdometry wheelOdometry;

    private final SwerveDriveSysId sysId;

    private RobotConfig config; // pathplanner physical description of robot

    // Modules
    private final SwerveModule[] modules = {
        new SwerveModule("FL", 0, Constants.SwerveModules.FRONT_LEFT),
        new SwerveModule("FR", 1, Constants.SwerveModules.FRONT_RIGHT),
        new SwerveModule("BL", 2, Constants.SwerveModules.BACK_LEFT),
        new SwerveModule("BR", 3, Constants.SwerveModules.BACK_RIGHT)
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
        } else {
            gyro = new GyroPigeon();
        }
        gyro.zeroYaw();

        wheelOdometry = new SwerveDriveOdometry(kinematics, getHeading(), getModulePositions());

        sysId = new SwerveDriveSysId(this, modules);

        try{
            config = RobotConfig.fromGUISettings(); // load from pathplanner file
        } catch (Exception e) {
            e.printStackTrace();
        }
   
        // Configure Pathplanner's AutoBuilder last
        AutoBuilder.configure(
            this::getPose, // pose supplier
            this::resetPose, // will be called if your auto has a starting pose
            this::getChassisSpeeds, // ROBOT RELATIVE ChassisSpeeds supplier
            (speeds, feedforwards) -> driveRobotRelative(speeds), // method for ROBOT RELATIVE driving
            new PPHolonomicDriveController( 
                    new PIDConstants(5, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration from saved deploy file
            AllianceUtil::isRed, // mirror if red, origin stays on blue side
            this // Reference to this subsystem to set requirements
        );
    }

    public Rotation2d getHeading() {
        return gyro.getYaw();
    }

    // READ: used by Vision to reduce trust in observations while spinning
    public double getYawVelocityRadiansPerSecond() {
        return gyroInputs.yawVelocityRadiansPerSecond;
    }

    // while disabled, seed the heading if robot sees 2 tags with mt1
    // otherwise, gyro is used and gyro will be 0
    public void seedHeading(Rotation2d heading) {
        gyro.setYaw(heading.getDegrees());
        poseEstimator.resetRotation(heading);
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
        wheelOdometry.resetPosition(getHeading(), getModulePositions(), pose);
    }

    public void setYaw(double yaw) {
        gyro.setYaw(yaw);
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

    // Drive Field Relative (telep joysticks)
    public void drive(double xVelocity, double yVelocity, double rotationalVelocity) {
        this.desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, rotationalVelocity, getPose().getRotation());
    }

    // Drive Robot Relative (pathplanner)
    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        this.desiredChassisSpeeds = chassisSpeeds; 
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

    // send the values to the pods
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(desiredStates[i]);
        }

        // For AdvantageScope Module Tuning
        Logger.recordOutput("Swerve/DesiredModuleStates", desiredStates);
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

        // update the swerve modules!
        for (int i = 0; i < modules.length; i++) {
            modules[i].updateInputs();
        }

        // For AdvantageScope Module Tuning
        Logger.recordOutput("Swerve/ModuleStates", getModuleStates());

        gyro.updateInputs(gyroInputs);
        Logger.processInputs("Swerve/Gyro", gyroInputs);
        gyroAlert.set(!gyroInputs.gyroIsPowered);

        Logger.recordOutput("Swerve/Pose", getPose());

        wheelOdometry.update(getHeading(), getModulePositions());
        Logger.recordOutput("Swerve/WheelOdometryPose", wheelOdometry.getPoseMeters());

    }

    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs); // manage heading w/ stn devs
    }

    public SwerveDriveSysId getSysId() {
        return sysId;
    }
}
