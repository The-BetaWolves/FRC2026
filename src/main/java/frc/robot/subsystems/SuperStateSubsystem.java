// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.services.FieldService;
import frc.robot.services.ShooterService;
import frc.robot.services.TurretService;

public class SuperStateSubsystem extends SubsystemBase {
  /** Creates a new NewSuperStateSubsystem. */
  public SuperStateSubsystem() {
  }

  // instantiate logic services
  private FieldService fieldService = new FieldService();
  private TurretService turretService = new TurretService();
  private ShooterService shooterService = new ShooterService();

  public enum FireIntent { STOP, IDLE, FIRE, CLEAR, INTAKE, SPIT, FIREANDINTAKE}
  private FireIntent fireIntent = FireIntent.STOP;
  public FireIntent getFireIntent() { return fireIntent; }

  // loop-hydrated variables
  private double turretSetpointRadians = 0.0;
  private Translation2d fieldTargetPose = new Translation2d();
  private Translation2d adjustedTargetPose = new Translation2d();
  private double flywheelSetpointRpm = 0.0;
  private double kickerSpeed, indexerSpeed, intakeSpeed = 0.0;
  private double intakeRotatorSetpoint = 0.0;
  private double distanceToTarget;
  double setFudgeFactor = 1.0;

  private double phaseSeconds = 0.0;
  private boolean phaseState = false;
    public boolean isTurretLocked = false;

  private final double flywheelIdleRPM = 0;

  private int clearTimer, rotatorTimer = 0;


  public void setFireIntent(FireIntent intent) {
    this.fireIntent = intent;
    setDefaultValues();
    setStaticValues();
  }

  private void setDefaultValues() {
    flywheelSetpointRpm = flywheelIdleRPM;
    kickerSpeed = 0.0;
    indexerSpeed = 0.0;
    intakeSpeed = 0.0;
    rotatorTimer = 0;
  }

  private void setStaticValues() {
    if (fireIntent == FireIntent.STOP) {
      flywheelSetpointRpm = 0.0; 
    } else if( fireIntent == FireIntent.IDLE) {
      flywheelSetpointRpm = flywheelIdleRPM;
      intakeRotatorSetpoint = Constants.Intake.maxRotatorDegree;
    } else if (fireIntent == FireIntent.CLEAR) {
      kickerSpeed = -0.8;
      indexerSpeed = -1.0;
      intakeRotatorSetpoint = Constants.Intake.minRotatorDegree;
    } else if (fireIntent == FireIntent.INTAKE) {
      intakeSpeed = 0.9;
      intakeRotatorSetpoint = Constants.Intake.minRotatorDegree;
    } else if (fireIntent == FireIntent.SPIT) {
      intakeSpeed = -0.9;
      intakeRotatorSetpoint = Constants.Intake.minRotatorDegree;
    } else if (fireIntent == FireIntent.FIRE) {
      kickerSpeed = 0.8;
      intakeSpeed = 0.5;
    } else if (fireIntent == FireIntent.FIREANDINTAKE) {
      kickerSpeed = 0.8;
      intakeSpeed = 0.9;
      intakeRotatorSetpoint = Constants.Intake.minRotatorDegree;
    }
  }

  // run on a loop to keep variables hydrated
    public void updateValues(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> fieldRelativeChassisSpeeds, Supplier<Boolean> flywheelIsAtSetpoint, Supplier<Boolean> turretIsAtSetpoint) {       
        fieldTargetPose = fieldService.getTargetPose(robotPose.get());
        Logger.recordOutput("SuperState/FieldTargetPose", fieldTargetPose);
        adjustedTargetPose = fieldService.getAdjustedTargetPose(robotPose.get(), fieldTargetPose, fieldRelativeChassisSpeeds.get());
        if (isTurretLocked) {
            turretSetpointRadians = 0.0;
        } else {
            turretSetpointRadians = 0.0; //turretService.getSetpointRadians(robotPose.get(), adjustedTargetPose);
        }
        distanceToTarget = fieldService.getDistanceFromTurretToTarget(robotPose.get(), adjustedTargetPose);

        Logger.recordOutput("SuperState/DistanceToTarget", fieldService.getDistanceToTarget(robotPose.get(), fieldTargetPose));
        Logger.recordOutput("SuperState/DistanceFromTurretToTarget", distanceToTarget);
        
        Logger.recordOutput("SuperState/AdjustedTarget", new Pose2d(adjustedTargetPose, new Rotation2d()));

        //Make a timer on Smartdashboard for the phases
        if (!(DriverStation.getGameSpecificMessage() == null)) {
            double stationTime = DriverStation.getMatchTime();

            if (
                (!(DriverStation.getGameSpecificMessage() == "R" && DriverStation.getAlliance().get() == Alliance.Red)) ||
                (!(DriverStation.getGameSpecificMessage() == "B" && DriverStation.getAlliance().get() == Alliance.Blue))
            ) {
                if (stationTime >= 130) {
                    phaseSeconds = stationTime - 130;
                    phaseState = true;
                } else if(stationTime >= 105) {
                    phaseSeconds = stationTime - 105;
                    phaseState = false;
                } else if(stationTime >= 80) {
                    phaseSeconds = stationTime - 80;
                    phaseState = true;
                } else if(stationTime >= 55) {
                    phaseSeconds = stationTime - 55;
                    phaseState = false;
                } else if(stationTime >= 30) {
                    phaseSeconds = stationTime - 30;
                    phaseState = true;
                } else if(stationTime < 30) {
                    phaseSeconds = stationTime;
                    phaseState = true;
                }
            } else {
                if (stationTime >= 130) {
                    phaseSeconds = stationTime - 130;
                    phaseState = false;
                } else if(stationTime >= 105) {
                    phaseSeconds = stationTime - 105;
                    phaseState = true;
                } else if(stationTime >= 80) {
                    phaseSeconds = stationTime - 80;
                    phaseState = false;
                } else if(stationTime >= 55) {
                    phaseSeconds = stationTime - 55;
                    phaseState = true;
                } else if(stationTime >= 30) {
                    phaseSeconds = stationTime - 30;
                    phaseState = false;
                } else if(stationTime < 30) {
                    phaseSeconds = stationTime;
                    phaseState = true;
                }
            }
        }
        
        // Driver-facing dashboard widgets — intentionally SmartDashboard, not Logger
        SmartDashboard.putNumber("Phase Seconds", phaseSeconds);
        SmartDashboard.putBoolean("Phase State", phaseState);

        //Set periodicly updated values for each state that needs it
        if (fireIntent == FireIntent.CLEAR) {
            clearTimer++;
            if(clearTimer > 10) {
                setFireIntent(FireIntent.IDLE);
                clearTimer = 0;
            } 
        } else if (fireIntent == FireIntent.FIRE) {
            flywheelSetpointRpm = shooterService.getShotSpeed(distanceToTarget, setFudgeFactor); //Change switch targets later

            if (flywheelIsAtSetpoint.get() && turretIsAtSetpoint.get()) {
                indexerSpeed = 1.0;
            } else {
                indexerSpeed = 0.0;
            }

            rotatorTimer++;
            if(rotatorTimer < 50) {
                intakeRotatorSetpoint = Constants.Intake.maxRotatorDegree;
            } else if (rotatorTimer > 50) {
                intakeRotatorSetpoint = Constants.Intake.minRotatorDegree;
            } 
            if (rotatorTimer > 100) {
                rotatorTimer = 0;
            }
        } else if (fireIntent == FireIntent.FIREANDINTAKE) {
            flywheelSetpointRpm = shooterService.getShotSpeed(distanceToTarget, setFudgeFactor); //Change switch targets later

            if (flywheelIsAtSetpoint.get() && turretIsAtSetpoint.get()
            ) {
                indexerSpeed = 1.0;
            } else {
                indexerSpeed = 0.0;
            }
        }

        if (isTurretLocked) {
            flywheelSetpointRpm = 4050.0;
        }
    }

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
  public void incrementSetFudgeFactor(double rate) {
      setFudgeFactor += rate;
  }
  public void toggleTurretLock() {
      if (isTurretLocked) {
          isTurretLocked = false;
      } else {
          isTurretLocked = true;
      }
  }
}
