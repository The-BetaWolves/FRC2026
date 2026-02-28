package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.services.FieldService;
import frc.robot.services.ShooterService;
import frc.robot.services.TurretService;
import frc.robot.subsystems.flywheel.Flywheel;

public class SuperStateSubsystem extends SubsystemBase {
    public SuperStateSubsystem () {}

    // instantiate logic services
    private FieldService fieldService = new FieldService();
    private TurretService turretService = new TurretService();
    private ShooterService shooterService = new ShooterService();

    // TODO: add the kicker. make it's rpm based on the FireIntent state

    // setup state enums
    public enum FireIntent { STOP, IDLE, FIRE, CLEAR }
    private FireIntent fireIntent = FireIntent.STOP;
    public void setFireIntent(FireIntent intent) { this.fireIntent = intent; }
    public FireIntent getFireIntent() { return fireIntent; }

    // loop-hydrated variables
    private double turretSetpointRadians = 0.0;
    private Translation2d fieldTargetPose = new Translation2d();
    private double flywheelSetpointRpm = 0.0;
    private double kickerSpeed, indexerSpeed = 0.0;

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

    int clearTimer = 0;

    // run on a loop to keep variables hydrated
    public void updateValues(Supplier<Pose2d> robotPose, Supplier<Boolean> flywheelIsAtSetpoint, Supplier<Double> flywheelSmartDashboardRpm, Supplier<Boolean> turretIsAtSetpoint) {       
        fieldTargetPose = fieldService.getTargetPose(robotPose.get());
        turretSetpointRadians = turretService.getSetpointRadians(robotPose.get(), new Translation2d(0.6, 3));
        

        if(fireIntent == FireIntent.STOP) {
            flywheelSetpointRpm = 0.0;
            kickerSpeed = 0.0;
            indexerSpeed = 0.0;
        } else if( fireIntent == FireIntent.IDLE) {
            flywheelSetpointRpm = 500.0;
            kickerSpeed = 0.0;
            indexerSpeed = 0.0;
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


        } else if (fireIntent == FireIntent.FIRE) {
            //flywheelSetpointRpm = shooterService.getShotSpeed(3.1); //Change switch targets later
            flywheelSetpointRpm = flywheelSmartDashboardRpm.get();
            kickerSpeed = 0.8;

            if (flywheelIsAtSetpoint.get() && turretIsAtSetpoint.get()) {
                indexerSpeed = 0.9;
            } else {
                indexerSpeed = 0.0;
            }
        } 
    }
}
