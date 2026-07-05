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
import frc.robot.subsystems.swerve.SwerveModule.SwerveModule;
import frc.robot.subsystems.swerve.SysId.SwerveDriveSysId;


public class SwerveDrive extends SubsystemBase {

    private ChassisSpeeds desiredChassisSpeeds;

    private final GyroIO gyro;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private RobotConfig config;

    private SwerveDriveOdometry wheelOdometry;

    private final SwerveDriveSysId sysId;

    // Modules
    private final SwerveModule[] modules = {
        new SwerveModule("Swerve/FL",0, Constants.SwerveModules.FRONT_LEFT),
        new SwerveModule("Swerve/FR", 1, Constants.SwerveModules.FRONT_RIGHT),
        new SwerveModule("Swerve/BL", 2, Constants.SwerveModules.BACK_LEFT),
        new SwerveModule("Swerve/BR", 3, Constants.SwerveModules.BACK_RIGHT)
    };

    private static final String[] MODULE_NAMES = {"FL", "FR", "BL", "BR"};
    private final Alert[] driveMotorAlerts = new Alert[4];
    private final Alert[] angleMotorAlerts = new Alert[4];
    private final Alert[] absoluteEncoderAlerts = new Alert[4];
    private final Alert gyroAlert = new Alert("Gyro not powered!", AlertType.kError);

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

        for (int i = 0; i < MODULE_NAMES.length; i++) {
            driveMotorAlerts[i] = new Alert(MODULE_NAMES[i] + " drive motor not powered!", AlertType.kError);
            angleMotorAlerts[i] = new Alert(MODULE_NAMES[i] + " angle motor not powered!", AlertType.kError);
            absoluteEncoderAlerts[i] = new Alert(MODULE_NAMES[i] + " CANcoder disconnected!", AlertType.kError);
        }

        // Gyro
        if(RobotBase.isSimulation()) {
            gyro = new GyroSim();
        } else {
            gyro = new GyroPigeon();
        }
        gyro.zeroYaw();

        wheelOdometry = new SwerveDriveOdometry(kinematics, getHeading(), getModulePositions());

        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
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
            // Mirror paths for the red alliance (the field origin stays on the blue side)
            AllianceUtil::isRed,
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

    // Resets the "wheel-only" odometry to a known pose
    public void resetOdometry(Pose2d pose) {
        wheelOdometry.resetPosition(getHeading(), getModulePositions(), pose);
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

        // loop the swerve modules!
        for (int i = 0; i < modules.length; i++) {
            modules[i].updateInputs();

            driveMotorAlerts[i].set(!modules[i].getInputs().driveMotorIsPowered);
            angleMotorAlerts[i].set(!modules[i].getInputs().angleMotorIsPowered);
            absoluteEncoderAlerts[i].set(!modules[i].getInputs().absoluteEncoderIsConnected);
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
