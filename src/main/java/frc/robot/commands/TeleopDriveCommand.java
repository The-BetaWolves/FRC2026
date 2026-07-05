// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SuperStateSubsystem;
import frc.robot.subsystems.SuperStateSubsystem.FireIntent;
import frc.robot.subsystems.swerve.SwerveDrive;

public class TeleopDriveCommand extends Command {
    private final SwerveDrive  swerve;
    private final SuperStateSubsystem superState;
    private final DoubleSupplier   vX;
    private final DoubleSupplier   vY;
    private final DoubleSupplier   vZ;
    private FireIntent rawfireState;

    private final double maxSwerveVelocity = Constants.SwerveConfig.MAXIMUM_CHASSIS_VELOCITY;
    private final double maxSwerveAngularVelocity = Constants.SwerveConfig.MAXIMUM_CHASSIS_ANGULAR_VELOCITY;

    private double speedModifier = 1.0;
    private double rotationModifier = 1.0;

    public TeleopDriveCommand(
        SwerveDrive swerve, 
        DoubleSupplier vX, 
        DoubleSupplier vY, 
        DoubleSupplier vZ,
        SuperStateSubsystem superState
    ) {
            this.swerve = swerve;
            this.vX = vX;
            this.vY = vY;
            this.vZ = vZ;
            this.superState = superState;
            addRequirements(swerve);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double xVelocity   = Math.pow(vX.getAsDouble(), 3);
        double yVelocity   = Math.pow(vY.getAsDouble(), 3);
        double angVelocity =  Math.pow(vZ.getAsDouble(), 3);

        rawfireState = superState.getFireIntent();
        Logger.recordOutput("SuperState/FireIntent", rawfireState.toString());
        if (rawfireState == FireIntent.FIRE || rawfireState == FireIntent.FIREANDINTAKE) {
            double maxSpeedMeters = 0.75;
            swerve.drive(
                xVelocity * maxSpeedMeters * speedModifier,
                yVelocity * maxSpeedMeters * speedModifier,
                angVelocity * maxSwerveAngularVelocity * rotationModifier
            );
        } else {
            if (xVelocity == 0.0 && yVelocity == 0.0 && angVelocity == 0.0) {
                swerve.lockWheels();
            } else {
                swerve.drive(
                xVelocity * maxSwerveVelocity * speedModifier,
                yVelocity * maxSwerveVelocity * speedModifier,
                angVelocity * maxSwerveAngularVelocity * rotationModifier
            );
            }
            
        }
             
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}