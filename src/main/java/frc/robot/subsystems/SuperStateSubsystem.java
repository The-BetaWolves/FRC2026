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

    // setup state enums
    public enum FireIntent { IDLE, FIRE }
    private FireIntent fireIntent = FireIntent.IDLE;
    public void setFireIntent(FireIntent intent) { this.fireIntent = intent; }
    public FireIntent getFireIntent() { return fireIntent; }

    // loop-hydrated variables
    private double turretSetpointRadians = 0.0;
    private Translation2d fieldTargetPose = new Translation2d();

    // public getters
    public double getTurretSetpointRadians() {
        return turretSetpointRadians;
    }
    public Translation2d getFieldTargetPose() {
        return fieldTargetPose;
    }

    // run on a loop to keep variables hydrated
    public void updateValues(Supplier<Pose2d> robotPose) {       
        fieldTargetPose = fieldService.getTargetPose(robotPose.get());

        turretSetpointRadians = turretService.getSetpointRadians(robotPose.get(), fieldTargetPose);
    }
    
}
