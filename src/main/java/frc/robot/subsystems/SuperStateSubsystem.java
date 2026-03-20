package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.services.FieldService;
import frc.robot.services.ShooterService;
import frc.robot.services.TurretService;
import frc.robot.subsystems.flywheel.Flywheel;

public class SuperStateSubsystem extends SubsystemBase {
    public SuperStateSubsystem () {
        SmartDashboard.setDefaultNumber("ballSpeedConstant", 1);
    }

    // instantiate logic services
    private FieldService fieldService = new FieldService();
    private TurretService turretService = new TurretService();
    private ShooterService shooterService = new ShooterService();

    // TODO: add the kicker. make it's rpm based on the FireIntent state

    // setup state enums
    public enum FireIntent { STOP, IDLE, FIRE, CLEAR, INTAKE, FIREANDINTAKE}
    private FireIntent fireIntent = FireIntent.STOP;
    public void setFireIntent(FireIntent intent) { this.fireIntent = intent; }
    public FireIntent getFireIntent() { return fireIntent; }

    // loop-hydrated variables
    private double turretSetpointRadians = 0.0;
    private Translation2d fieldTargetPose = new Translation2d();
    private Translation2d adjustedTargetPose = new Translation2d();
    private double flywheelSetpointRpm = 0.0;
    private double kickerSpeed, indexerSpeed, intakeSpeed = 0.0;
    private double intakeRotatorSetpoint = 0.0;
    private double distanceToTarget;

    // public getters
    public double getTurretSetpointRadians() {
        return turretSetpointRadians;
    }
    public Translation2d getFieldTargetPose() {
        return fieldTargetPose;
    }
    public double getFlywheelSetpointRpm() {
        return flywheelSetpointRpm;
    }
    public double getKickerSpeed() {
        return kickerSpeed;
    }
    public double getIndexerSpeed() {
        return indexerSpeed;
    }
    public Translation2d getAdjustedTargetPose() {
        return adjustedTargetPose;
    }
    public double getDistanceToTarget() {
        return distanceToTarget;
    }
    public double getIntakeSpeed() {
        return intakeSpeed;
    }
    public double getIntakeRotatorSetpoint() {
        return intakeRotatorSetpoint;
    }
    public void incrementIntakeRotatorSetpoint(double rate) {
        intakeRotatorSetpoint = intakeRotatorSetpoint + rate;
    }

    int clearTimer, rotatorTimer = 0;

    // run on a loop to keep variables hydrated
    public void updateValues(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> fieldRelativeChassisSpeeds, Supplier<Boolean> flywheelIsAtSetpoint, Supplier<Boolean> turretIsAtSetpoint) {       
        fieldTargetPose = fieldService.getTargetPose(robotPose.get());
        adjustedTargetPose = fieldService.getAdjustedTargetPose(robotPose.get(), Constants.Field.realBlueHubPose, fieldRelativeChassisSpeeds.get());
        turretSetpointRadians = turretService.getSetpointRadians(robotPose.get(), adjustedTargetPose);
        //turretSetpointRadians = turretService.getSetpointRadians(robotPose.get(), fieldTargetPose);
        distanceToTarget = fieldService.getDistanceFromTurretToTarget(robotPose.get(), adjustedTargetPose);
        //distanceToTarget = fieldService.getDistanceFromTurretToTarget(robotPose.get(), fieldTargetPose);

        double[] adjustedTargetArray = {adjustedTargetPose.getX(), adjustedTargetPose.getY()};
        SmartDashboard.putNumberArray("adjustedTargetPose", adjustedTargetArray);
        SmartDashboard.putNumber("distanceToTarget", fieldService.getDistanceToTarget(robotPose.get(), fieldTargetPose));
        SmartDashboard.putNumber("distanceFromTurretToTarget", distanceToTarget);
        
        Logger.recordOutput("SuperState/AdjustedTarget", new Pose2d(adjustedTargetPose, new Rotation2d()));

        if(fireIntent == FireIntent.STOP) {
            flywheelSetpointRpm = 0.0;
            kickerSpeed = 0.0;
            indexerSpeed = 0.0;
            intakeSpeed = 0.0;

        } else if( fireIntent == FireIntent.IDLE) {
            flywheelSetpointRpm = 500.0;
            kickerSpeed = 0.0;
            indexerSpeed = 0.0;
            intakeSpeed = 0.0;

        } else if (fireIntent == FireIntent.CLEAR) {
            clearTimer++;
            if(clearTimer > 10) {
                fireIntent = FireIntent.IDLE;
                clearTimer = 0;
            } 

            flywheelSetpointRpm = 500;
            kickerSpeed = -0.8;
            //kickerSpeed = 0.0;
            indexerSpeed = -1.0;
            intakeSpeed = 0.0;

        } else if (fireIntent == FireIntent.INTAKE) {
            flywheelSetpointRpm = 500.0;
            kickerSpeed = 0.0;
            indexerSpeed = 0.0;
            intakeSpeed = 0.9;
            intakeRotatorSetpoint = Constants.Intake.minRotatorDegree;
        } else if (fireIntent == FireIntent.FIRE) {
            flywheelSetpointRpm = shooterService.getShotSpeed(distanceToTarget); //Change switch targets later
            kickerSpeed = 0.8;
            intakeSpeed = 0.9;

            if (flywheelIsAtSetpoint.get() && turretIsAtSetpoint.get()) {
                indexerSpeed = 0.9;
            } else {
                indexerSpeed = 0.0;
            }

            rotatorTimer++;
            if(rotatorTimer < 50) {
                intakeRotatorSetpoint = Constants.Intake.maxRotatorDegree;
            } else if (rotatorTimer > 50) {
                intakeRotatorSetpoint = 5;
            } 
            if (rotatorTimer > 100) {
                rotatorTimer = 0;
            }
        } else if (fireIntent == FireIntent.FIREANDINTAKE) {
            flywheelSetpointRpm = shooterService.getShotSpeed(distanceToTarget); //Change switch targets later
            kickerSpeed = 0.8;
            intakeRotatorSetpoint = Constants.Intake.minRotatorDegree;
            intakeSpeed = 0.9;

            /*
            rotatorTimer++;
            if(rotatorTimer > 60) {
                intakeRotatorSetpoint = 100;
            } else if (rotatorTimer > 120) {
                intakeRotatorSetpoint = 4;
                rotatorTimer = 0;
            }
             */

            if (flywheelIsAtSetpoint.get() && turretIsAtSetpoint.get()) {
                indexerSpeed = 0.9;
            } else {
                indexerSpeed = 0.0;
            }
        }
    }
}
