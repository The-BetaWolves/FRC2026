package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.services.FieldService;
import frc.robot.services.TurretService;

public class SuperStateSubsystem extends SubsystemBase {
    public SuperStateSubsystem () {}

    // instantiate logic services
    private FieldService fieldService = new FieldService();
    private TurretService turretService = new TurretService();

    // TODO: add the kicker. make it's rpm based on the FireIntent state

    // setup state enums
    public enum FireIntent { STOP, IDLE, FIRE }
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

    // run on a loop to keep variables hydrated
    public void updateValues(Supplier<Pose2d> robotPose) {       
        fieldTargetPose = fieldService.getTargetPose(robotPose.get());
        turretSetpointRadians = turretService.getSetpointRadians(robotPose.get(), fieldTargetPose);

        if(fireIntent == FireIntent.STOP) {
            flywheelSetpointRpm = 0.0;
            kickerSpeed = 0.0;
            indexerSpeed = 0.0;
        } else if( fireIntent == FireIntent.IDLE) {
            flywheelSetpointRpm = 500.0;
            kickerSpeed = 0.0;
            indexerSpeed = 0.0;
        } else if (fireIntent == FireIntent.FIRE) {
            // TODO: make a distance based lookup table with wpilib's lookup feature
            // https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/interpolation/InterpolatingDoubleTreeMap.html
            flywheelSetpointRpm = 5000.0;

            kickerSpeed = 0.8;
            indexerSpeed = 0.9;
        } 
    }
}
